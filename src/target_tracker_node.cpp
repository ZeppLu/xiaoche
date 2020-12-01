#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>

#include <xiaoche/SteeringAngle.h>


class PID {
public:
    PID() = default;

    PID(double Kp, double Ki, double Kd, double max) : Kp(Kp), Ki(Ki), Kd(Kd), err_acc(0), err_prev(0), max(max) {}

    double update(double dt, double err) {
        double output = Kp * err +
                        Ki * (err_prev * dt + err_acc) +
                        Kd * (err - err_prev) / dt;

        if (output < -max || output > max) {
            // too many clampings indicates bad-tuning parameters
            ROS_WARN("PID output %lf too large!", output);
            // (output > 0) <==> (output > max)
            output = output > 0 ?  max : output;
            output = output < 0 ? -max : output;
        }

        // note err_prev here
        err_acc += err_prev * dt;
        err_prev = err;

        return output;
    }

    void reset() {
        err_acc = 0;
        err_prev = 0;
    }

private:
    double Kp, Ki, Kd;
    double err_acc, err_prev;  // accumulated error (used in I) and previous error (used in D)
    double max;  // range of output: [-max, max]
};


class Tracker {
public:
    Tracker(ros::NodeHandle nh, ros::NodeHandle priv_nh) {
        bool all_params_recved = true;

        double Kp_x, Kp_y, Ki_x, Ki_y, Kd_x, Kd_y, pid_max_x, pid_max_y;
        all_params_recved &=
            priv_nh.getParam("pid/x/Kp", Kp_x) &
            priv_nh.getParam("pid/x/Ki", Ki_x) &
            priv_nh.getParam("pid/x/Kd", Kd_x) &
            priv_nh.getParam("pid/x/max", pid_max_x) &
            priv_nh.getParam("pid/y/Kp", Kp_y) &
            priv_nh.getParam("pid/y/Ki", Ki_y) &
            priv_nh.getParam("pid/y/Kd", Kd_y) &
            priv_nh.getParam("pid/y/max", pid_max_y);

        double view_width, view_height;
        all_params_recved &=
            priv_nh.getParam("view/width", view_width) &
            priv_nh.getParam("view/height", view_height);

        double tol;
        all_params_recved &=
            priv_nh.getParam("tolerance", tol);

        std::string servo_center_srv;  // = "/servo_node/center";
        all_params_recved &=
            priv_nh.getParam("servo_center_service_name", servo_center_srv);

        // check whether all parameters are set
        if (!all_params_recved) {
            ROS_ERROR("some parameters not set, failed to initialize tracker");
            throw ros::Exception("failed to retrieve all parameters needed for tracker");
        }

        // initialize members
        this->controller_x = PID(Kp_x, Ki_x, Kd_x, pid_max_x);
        this->controller_y = PID(Kp_y, Ki_y, Kd_y, pid_max_y);
        // ros-generated messages default to 0, no need to change this->angle
        this->target.x = view_width / 2;
        this->target.y = view_height / 2;
        this->tolerance = tol;

        // wait till servo is ready
        ros::service::waitForService(servo_center_srv);
        //ros::Duration(1.0).sleep();
        std_srvs::Empty srv;
        if (!ros::service::call(servo_center_srv, srv)) {
            ROS_ERROR("failed to center servos, exitting...");
            throw ros::Exception("failed to center servos");
        }

        // subscriber and publisher
        target_sub = nh.subscribe("target_filtered", 1, &Tracker::target_pose_callback, this);
        angle_pub = nh.advertise<xiaoche::SteeringAngle>("servo_angle", 1, true);
    }

    void target_pose_callback(const geometry_msgs::Pose2DConstPtr& pose) {
        // no detection
        if (pose->x == 0 && pose->y == 0) {
            ROS_WARN_THROTTLE(5, "target coordinate is (0, 0)");
            controller_x.reset();
            controller_y.reset();
            return;
        }

        double abserr_x = std::max(std::abs(target.x - pose->x) - tolerance, 0.0);
        double abserr_y = std::max(std::abs(target.y - pose->y) - tolerance, 0.0);

        double err_x = target.x > pose->x ? abserr_x : -abserr_x;
        double err_y = target.y > pose->y ? abserr_y : -abserr_y;

        // update controllers
        double dt = (ros::Time::now() - last_update).toSec();
        double disp_x = controller_x.update(dt, err_x);
        double disp_y = controller_y.update(dt, err_y);
        last_update = ros::Time::now();

        // update angle and publish
        // yaw>0 turns left, pitch>0 goes upwards
        angle.yaw += disp_x;
        angle.pitch += disp_y;
        angle_pub.publish(angle);
    }

private:
    // controller for two axes
    PID controller_x, controller_y;
    // you want to keep target around target's coordinate
    geometry_msgs::Pose2D target;
    // used to store the position of servo
    xiaoche::SteeringAngle angle;
    // don't apply control until error exceeds [-tolerance, tolerance]
    double tolerance;
    // used to calculate dt, which is passed to PID controller
    ros::Time last_update;
    // publisher and subscriber
    ros::Publisher angle_pub;
    ros::Subscriber target_sub;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "target_tracker_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // everything is done inside its constructor
    Tracker tracker(nh, private_nh);

    ros::spin();

    return 0;
}