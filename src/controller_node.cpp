#include <signal.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <xiaoche/SteeringAngle.h>

#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>


struct CommandsFilter {
    size_t N;
    size_t idx;

    struct Command {
        // these default values are guaranteed by C++11
        // see https://en.cppreference.com/w/cpp/language/data_members#Member_initialization
        double linear_x = 0.0;
        double angular_z = 0.0;
        double yaw = 0.0;
    };

    std::vector<Command> buffer;

    CommandsFilter(size_t _N) : N(_N), idx(0), buffer(N, Command()) {}

    Command update(const Command& cmd) {
        buffer[idx] = cmd;
        idx = (idx + 1) % N;
        // average
        Command result;
        for (auto& c : buffer) {
            result.linear_x += c.linear_x;
            result.angular_z += c.angular_z;
            result.yaw += c.yaw;
        }
        result.linear_x /= N;
        result.angular_z /= N;
        result.yaw /= N;
        return result;
    }
};


class Controller {
public:
    Controller()
        : state(State::NOT_INIT)
        , listener()
        , cmd_vel_msg()
        , angle_msg()
        , filter(0)
        {}

    // return true on success, false otherwise
    bool setup(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

    // step forward
    void step();

    // currently it only sets linear.{x,y}, angular.z and angle to 0
    void stop();

private:
    enum State {
        NOT_INIT,
        FOLLOWING,
        LOST
    } state;

    bool lookup_transforms();

    void clear_messages();

    // ros specific
    // use pointer type so that object can be init before ros::init()
    std::unique_ptr<ros::Publisher> cmd_vel_pub, angle_pub;
    std::unique_ptr<tf::TransformListener> listener;

    // data that are assigned only once and never change
    std::string base_link_frame, camera_frame, target_frame;
    // TODO: make it configurable; make it ros::Duration
    double transform_valid_time = 1.0;
    double angular_vel, linear_vel;
    double follow_dist, follow_dist_tol, follow_angle_tol;
    xiaoche::SteeringAngle angle_max, angle_min;

    // volatile
    tf::StampedTransform base_to_target, camera_to_target;
    geometry_msgs::Twist cmd_vel_msg;
    xiaoche::SteeringAngle angle_msg;
    CommandsFilter filter;
};


bool Controller::setup(ros::NodeHandle& nh, ros::NodeHandle& priv_nh) {
    priv_nh.param<std::string>("base_link_frame", this->base_link_frame, "base_link");
    priv_nh.param<std::string>("camera_frame", this->camera_frame, "camera_color_frame");
    priv_nh.param<std::string>("target_frame", this->target_frame, "target");

    if (!priv_nh.getParam("angular_velocity", this->angular_vel) ||
        !priv_nh.getParam("linear_velocity", this->linear_vel)) {
        ROS_ERROR("angular or linear velocity not set!");
        return false;
    }
    assert(angular_vel > 0 && linear_vel > 0);

    if (!priv_nh.getParam("follow_distance", this->follow_dist) ||
        !priv_nh.getParam("follow_distance_tolerance", this->follow_dist_tol) ||
        !priv_nh.getParam("follow_angle_tolerance", this->follow_angle_tol)) {
        ROS_ERROR("following parameters not set!");
        return false;
    }
    assert(follow_dist_tol > 0 && follow_angle_tol > 0);

    int filter_N;
    if (!priv_nh.getParam("mean_filter_number", filter_N)) {
        ROS_ERROR("N of mean filter not set!");
        return false;
    }
    assert(filter_N > 0);
    this->filter = CommandsFilter(filter_N);

    // standard way to create new publisher is through `nh.advertise()',
    // which returns rvalue whose address can not be taken,
    // so we must use `make_unique()' to convert it to pointer
    this->cmd_vel_pub = std::make_unique<ros::Publisher>(nh.advertise<geometry_msgs::Twist>("cmd_vel", 10));
    this->angle_pub = std::make_unique<ros::Publisher>(nh.advertise<xiaoche::SteeringAngle>("servo_angle", 10));
    // transform listener can be created via `new', so a `reset()' is enough
    this->listener.reset(new tf::TransformListener());

    // wait and call service to center camera
    std::string servo_node;
    priv_nh.param<std::string>("servo_node_name", servo_node, "servo_node");
    std_srvs::Empty srv;
    ros::service::waitForService(servo_node + "/center");
    if (!ros::service::call(servo_node + "/center", srv)) {
        ROS_ERROR("failed to call service %s/center", servo_node.c_str());
        return false;
    }

    // get upper & lower bound of servo angle
    // should be after centering camera servo
    if (!nh.getParam(servo_node + "/yaw_max", this->angle_max.yaw) ||
        !nh.getParam(servo_node + "/yaw_min", this->angle_min.yaw) ||
        !nh.getParam(servo_node + "/pitch_max", this->angle_max.pitch) ||
        !nh.getParam(servo_node + "/pitch_min", this->angle_min.pitch)) {
        ROS_ERROR("failed to get parameters %s/{yaw,pitch}_{max,min}", servo_node.c_str());
        return false;
    }
    // show them
    ROS_INFO_STREAM("max angle: " << angle_max);
    ROS_INFO_STREAM("min angle: " << angle_min);

    this->state = State::LOST;
    return true;
}


void Controller::step() {
    double linear_x = 0, angular_z = 0, yaw = 0;

    switch (this->state) {
    case State::NOT_INIT:
        throw ros::Exception("you should setup() before step()");

    case State::LOST:
        if (this->lookup_transforms()) {
            this->state = State::FOLLOWING;
        }
        break;

    case State::FOLLOWING:
        if (!this->lookup_transforms()) {
            this->state = State::LOST;

        } else {  // else statement is required to create a separate scope
            // TODO: also move camera servo
            const tf::Vector3& origin = this->base_to_target.getOrigin();
            double dist = origin.length();
            if (dist < follow_dist - follow_dist_tol) {
                linear_x = -linear_vel;
            }
            if (dist > follow_dist + follow_dist_tol) {
                linear_x =  linear_vel;
            }
            double angle = std::atan2(origin.y(), origin.x());
            if (angle < -follow_angle_tol) {
                angular_z = -angular_vel;
            }
            if (angle >  follow_angle_tol) {
                angular_z =  angular_vel;
            }
	    }
        break;

    default:
        throw ros::Exception("unknown controller state");
    }

    // average filter
    CommandsFilter::Command cmd = filter.update({
        linear_x,
        angular_z,
        yaw
    });

    // publish messages
    cmd_vel_msg.linear.x = linear_x;
    cmd_vel_msg.angular.z = angular_z;
    angle_msg.yaw = yaw;
    this->cmd_vel_pub->publish(cmd_vel_msg);
    //this->angle_pub.publish(angle_msg);
}


void Controller::stop() {
    if (state != State::NOT_INIT) {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.angular.z = 0;
        angle_msg.yaw = 0;
        angle_msg.pitch = 0;
        cmd_vel_pub->publish(cmd_vel_msg);
        angle_pub->publish(angle_msg);
    }
}


bool Controller::lookup_transforms() {
    try {
        this->listener->lookupTransform(base_link_frame, target_frame, ros::Time(0), this->base_to_target);
        this->listener->lookupTransform(camera_frame, target_frame, ros::Time(0), this->camera_to_target);
        ros::Duration base_diff = ros::Time::now() - this->base_to_target.stamp_;
        ros::Duration camera_diff = ros::Time::now() - this->camera_to_target.stamp_;
        return base_diff.toSec() < this->transform_valid_time && camera_diff.toSec() < this->transform_valid_time;
    } catch (tf::TransformException e) {
        ROS_ERROR("failed to lookup transform (%s)", e.what());
        return false;
    }
}



// in order to be used in signal handler, controller must be global.
// no, you can't use lambda expr to capture controller,
// only functional lambda is convertible to function pointer
Controller g_controller;
void sigint_handler(int signal) {
    g_controller.stop();
    ros::shutdown();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh, priv_nh("~");

    Controller& controller = g_controller;
    if (!controller.setup(nh, priv_nh)) {
        return 1;
    }

    // replace default SIGINT handler, so we can stop robot when exitting
    signal(SIGINT, sigint_handler);

    ros::Rate rate(60);
    while (ros::ok()) {
        controller.step();
        rate.sleep();
        // no need to spin, cause controller register no callback,
        // it only ask for transform when needed
    }

    return 0;
}
