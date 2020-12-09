#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


void detections_and_depth_callback(
		const vision_msgs::Detection2DArrayConstPtr& detections,
		const sensor_msgs::ImageConstPtr& depth,
		double horizontal_rate,
		double vertical_rate,
		const ros::Publisher& dist_pub
		) {

	// detection network should output only 1 class, so 0 or 1 bbox
	assert(detections->detections.size() <= 1);
	if (detections->detections.empty()) {
		return;
	}
	const vision_msgs::BoundingBox2D& bbox = detections->detections[0].bbox;

	// initialize cv_bridge pointer
	cv_bridge::CvImageConstPtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvShare(depth, depth->encoding);
		//cv_ptr = cv_bridge::toCvCopy(*depth, depth->encoding);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	// get ROI (x_topleft, y_topleft, width, height)
	double width = bbox.size_x * horizontal_rate;
	double height = bbox.size_y * vertical_rate;
	double x = bbox.center.x - width / 2.0;
	double y = bbox.center.y - height / 2.0;
	// crop
	cv::Mat target_depth = cv_ptr->image(cv::Rect(x, y, width, height));

	// calculate mean distance
	cv::Scalar distance = cv::mean(target_depth);
	// TODO: use distance to calculate TF
	// 1m == 1000mm
	dist_pub.publish(distance.val[0] / 1000.0);

	// distance measured in `camera_color_optical_frame', x right, y down, z forward
	// TODO: how to map image coord & distance to 3D coord?
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "test_depth");

	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");
	image_transport::ImageTransport it(priv_nh);


	// parameters used to help messages matching faster
	// see http://wiki.ros.org/message_filters/ApproximateTime for details
	int lower_bound_ms;
	double age_penalty;
	// TODO: raise exception
	if (!priv_nh.getParam("lower_bound_ms", lower_bound_ms)) {
		ROS_ERROR("parameter `lower_bound_ms' not set! exitting...");
		return 0;
	}
	if (!priv_nh.getParam("age_penalty", age_penalty)) {
		ROS_ERROR("parameter `age_penalty' not set! exitting...");
		return 0;
	}


	// parameters used in extracting target distance
	double horizontal_rate = 0.8, vertical_rate = 0.8;
	priv_nh.param("horizontal_rate", horizontal_rate, horizontal_rate);
	priv_nh.param("vertical_rate", vertical_rate, vertical_rate);


	// result publisher
	ros::Publisher dist_pub = priv_nh.advertise<std_msgs::Float64>("dist_pub", 10);


	// subscribers for camera captures & detections output from jetson nano
	// use message_filters to match messages from different topics by timestamps,
	// note that image_transport::SubscriberFilter() is used for sensor_msgs/Image type
	// TODO: what does last argument (queue_size) in these 3 lines do?
	// XXX: when remapping topics, be sure to remap /detections_viz/image_in/compressed
	message_filters::Subscriber<vision_msgs::Detection2DArray> detect_sub(priv_nh, "detections", 1);
	image_transport::SubscriberFilter depth_sub(it, "depth_image_in", 1);

	// see http://wiki.ros.org/message_filters/ApproximateTime for details
	typedef message_filters::sync_policies::ApproximateTime<vision_msgs::Detection2DArray, sensor_msgs::Image> SyncPol;
	// queue_size = 10
	message_filters::Synchronizer<SyncPol> sync(SyncPol(3), detect_sub, depth_sub);
	// 1s == 1000ms
	sync.setInterMessageLowerBound(ros::Duration(lower_bound_ms / 1000.0));
	sync.setAgePenalty(age_penalty);
	// take const reference of `image_pub' to avoid copying
	sync.registerCallback(boost::bind(&detections_and_depth_callback, _1, _2, horizontal_rate, vertical_rate, boost::cref(dist_pub)));


	ros::spin();

	return 0;
}
