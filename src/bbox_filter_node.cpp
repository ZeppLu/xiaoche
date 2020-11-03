#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <vision_msgs/Detection2DArray.h>

#include <algorithm>
#include <list>


// add to pose timestamp info, so that we're able to distinguish out-dated messages
struct Pose2DStamped {
    ros::Time stamp;
    geometry_msgs::Pose2D pose;
};


class AverageFilter {
public:
    AverageFilter(double secs, int id) : poses(), period(secs), target_id(id) {}

    void update(const vision_msgs::Detection2DArrayConstPtr& msg) {
        const std::vector<vision_msgs::Detection2D>& detections = msg->detections;

        // copy detections with expected id
        std::vector<vision_msgs::Detection2D> targets;
        auto is_target = [this](const vision_msgs::Detection2D& det) {
            return det.results[0].id == this->target_id;
        };
        std::copy_if(detections.begin(), detections.end(), std::back_inserter(targets), is_target);

        // prevent segfault when pushing into the queue
        if (targets.empty()) {
            // just keep calm & waiting, don't complain too much
            ROS_WARN_THROTTLE(10.0, "no detection matches id %d", target_id);
            return;
        }

        // calculate the area of each bbox, then choose the largest one
        std::vector<double> areas(targets.size());
        // just like `map' in Haskell
        std::transform(targets.begin(), targets.end(), areas.begin(), [](const vision_msgs::Detection2D& det) {
            return det.bbox.size_x * det.bbox.size_y;
        });
        int max_index = std::distance(areas.begin(), std::max_element(areas.begin(), areas.end()));
        poses.push_back({
            .stamp  = targets[max_index].header.stamp,
            .pose   = targets[max_index].bbox.center
        });
    }

    void flush() {
        // remove out-dated poses
        for (auto it = poses.begin(); it != poses.end();) {
            if ((ros::Time::now() - it->stamp) > period) {
                // list::erase() returns next iterator
                it = poses.erase(it);
            } else {
                // items in `poses' are in chronological order,
                // so no need to continue
                break;
            }
        }
    }

    geometry_msgs::Pose2D average() {
        flush();

        geometry_msgs::Pose2D result;
        // prevent singularity in averaging
        if (poses.empty()) {
            // node /detectnet needs time to load network, so this warning is expected at the begging
            // print it every 10 seconds to make it less annoying
            ROS_WARN_THROTTLE(10.0, "no poses history, returning 0");
            return result;
        }

        for (const auto& pose : poses) {
            result.x += pose.pose.x;
            result.y += pose.pose.y;
            // for now, don't care theta
            //result.theta += pose.theta;
        }
        result.x /= poses.size();
        result.y /= poses.size();
        //result.theta /= poses.size();

        return result;
    }

private:
    ros::Duration period;  // length of time window
    int target_id;  // can be found in `rosparam get /detectnet/class_labels_xxxxxxx'
    std::list<Pose2DStamped> poses;  // use list to minic a queue
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "bbox_filter");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // time window to perform average on
    double secs = 0.5;
    private_nh.param("average_secs", secs, secs);

    int target_id = 1;  // person

    ros::Rate rate(30);

    AverageFilter filter(secs, target_id);

    ros::Subscriber detections_sub = private_nh.subscribe("detections", 30, &AverageFilter::update, &filter);
    ros::Publisher target_pub = nh.advertise<geometry_msgs::Pose2D>("target_filtered", 30, true);

    while (ros::ok()) {
        ros::spinOnce();
        target_pub.publish(filter.average());
        rate.sleep();
    }

    return 0;
}