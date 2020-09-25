#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <vision_msgs/Detection2DArray.h>

#include <algorithm>
#include <list>


class AverageFilter {
public:
    AverageFilter(int average_number, int id) : poses(), N(average_number), target_id(id) {}

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
            return;
        }

        // calculate the area of each bbox, then choose the largest one
        std::vector<double> areas(targets.size());
        std::transform(targets.begin(), targets.end(), areas.begin(), [](const vision_msgs::Detection2D& det) {
            return det.bbox.size_x * det.bbox.size_y;
        });
        int max_index = std::distance(areas.begin(), std::max_element(areas.begin(), areas.end()));
        poses.push_back(targets[max_index].bbox.center);

        // remove out-dated points
        if (poses.size() >= N) {
            poses.pop_front();
        }
    }

    geometry_msgs::Pose2D average() const {
        geometry_msgs::Pose2D result;
        // prevent singularity in averaging
        if (poses.empty()) {
            return result;
        }

        for (const auto& pose : poses) {
            result.x += pose.x;
            result.y += pose.y;
            // for now, don't care theta
            //result.theta += pose.theta;
        }
        result.x /= poses.size();
        result.y /= poses.size();
        //result.theta /= poses.size();

        return result;
    }

private:
    int N;  // length of the queue
    int target_id;  // can be found in `rosparam get /detectnet/class_labels_xxxxxxx'
    std::list<geometry_msgs::Pose2D> poses;  // use list to minic a queue
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "bbox_filter");

    //ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // the number to perform average on, default to 3
    int N = 5;
    private_nh.param("average_number", N, N);

    int target_id = 1;  // person

    ros::Rate rate(30);

    AverageFilter filter(N, target_id);

    ros::Subscriber detections_sub = private_nh.subscribe("detections", N, &AverageFilter::update, &filter);
    ros::Publisher target_pub = private_nh.advertise<geometry_msgs::Pose2D>("target_filtered", 30, true);

    while (ros::ok()) {
        ros::spinOnce();
        target_pub.publish(filter.average());
        rate.sleep();
    }

    return 0;
}