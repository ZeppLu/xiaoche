#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_broadcaster.h>

#include <librealsense2/rsutil.h>



class TargetLocator {
public:
	TargetLocator()
		: nh()
		, priv_nh("~")
		, it(nh)
		, priv_it(priv_nh)
		, sync(SyncPol(3))  // queue_size = 3
		, tf_broadcaster()
		, transform(tf::Quaternion::getIdentity())
		{}
	void setup();

	// see http://wiki.ros.org/message_filters/ApproximateTime for details
	using SyncPol = message_filters::sync_policies::ApproximateTime<vision_msgs::Detection2DArray, sensor_msgs::Image>;
	using Synchronizer = message_filters::Synchronizer<SyncPol>;

private:
	void get_intrinsics(const sensor_msgs::CameraInfoConstPtr& cam_info);

	void detections_and_depth_callback(
		const vision_msgs::Detection2DArrayConstPtr& detections,
		const sensor_msgs::ImageConstPtr& depth);

	ros::NodeHandle nh;
	ros::NodeHandle priv_nh;
	image_transport::ImageTransport it;
	image_transport::ImageTransport priv_it;

	Synchronizer sync;

	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform transform;

	message_filters::Subscriber<vision_msgs::Detection2DArray> detect_sub;
	image_transport::SubscriberFilter depth_sub;
	ros::Publisher dist_pub;

	double horizontal_rate;
	double vertical_rate;

	rs2_intrinsics intrin;

	std::string src_frame;
	std::string dst_frame;
};


void TargetLocator::setup() {

	// parameters used to help messages matching faster
	// see http://wiki.ros.org/message_filters/ApproximateTime for details
	int lower_bound_ms;
	double age_penalty;
	if (!priv_nh.getParam("lower_bound_ms", lower_bound_ms)) {
		throw ros::InvalidParameterException("parameter `lower_bound_ms' not set!");
	}
	if (!priv_nh.getParam("age_penalty", age_penalty)) {
		throw ros::InvalidParameterException("parameter `age_penalty' not set!");
	}

	// parameters used in extraction of target distance
	this->horizontal_rate = 1.0;
	this->vertical_rate = 1.0;
	priv_nh.param("horizontal_rate", horizontal_rate, horizontal_rate);
	priv_nh.param("vertical_rate", vertical_rate, vertical_rate);

	// distance publisher
	this->dist_pub = priv_nh.advertise<std_msgs::Float64>("distance", 10);

	// get camera info once, ignore subsequent update
	double timeout_sec = 1.0;
	priv_nh.param("camera_info_timeout", timeout_sec, timeout_sec);
	const auto cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("depth_camera_info", priv_nh, ros::Duration(timeout_sec));
	if (cam_info == nullptr) {
		throw ros::Exception("cannot receive CameraInfo message!");
	}
	this->get_intrinsics(cam_info);
	this->src_frame = cam_info->header.frame_id;

	// target frame name
	this->dst_frame = "target";


	// subscribers for camera captures & detections output from jetson nano
	// use message_filters to match messages from different topics by timestamps,
	// note that image_transport::SubscriberFilter() is used for sensor_msgs/Image type
	// TODO: what does last argument (queue_size) in these 3 lines do?
	this->detect_sub.subscribe(priv_nh, "detections", 1);
	this->depth_sub.subscribe(priv_it, "depth_image_raw", 1);

	this->sync.connectInput(detect_sub, depth_sub);
	// 1s == 1000ms
	this->sync.setInterMessageLowerBound(ros::Duration(lower_bound_ms / 1000.0));
	this->sync.setAgePenalty(age_penalty);
	this->sync.registerCallback(&TargetLocator::detections_and_depth_callback, this);
}



void TargetLocator::get_intrinsics(const sensor_msgs::CameraInfoConstPtr& cam_info) {
	// for now, do some assumptions
	assert(cam_info->distortion_model == "plumb_bob");
	for (auto coeff : cam_info->D) {
		assert(coeff == 0.0);
	}
	// apply assumption
	this->intrin.model	= RS2_DISTORTION_NONE;
	for (auto& coeff : this->intrin.coeffs) {
		coeff = 0.0;
	}

	//     [fx  0 cx]
	// K = [ 0 fy cy]
	//     [ 0  0  1]
	this->intrin.width	= (int)cam_info->width;
	this->intrin.height	= (int)cam_info->height;
	this->intrin.ppx	= (float)cam_info->K[2];
	this->intrin.ppy	= (float)cam_info->K[5];
	this->intrin.fx	= (float)cam_info->K[0];
	this->intrin.fy	= (float)cam_info->K[4];
}



void TargetLocator::detections_and_depth_callback(
		const vision_msgs::Detection2DArrayConstPtr& detections,
		const sensor_msgs::ImageConstPtr& depth) {

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
	// TODO: ignore depth hole (value==0)
	double distance = cv::mean(target_depth).val[0] / 1000.0;
	// 1m == 1000mm
	dist_pub.publish(distance);

	// deproject image coordinate & distance to 3D coordinate
	float pixel[2] = { (float)bbox.center.x, (float)bbox.center.y };
	float point[3] = { 0, 0, 0};
	rs2_deproject_pixel_to_point(point, &intrin, pixel, distance);

	// send transform
	this->transform.setOrigin(tf::Vector3(point[0], point[1], point[2]));
	// XXX: send which timestamp?
	tf_broadcaster.sendTransform(
		tf::StampedTransform(transform, detections->header.stamp, this->src_frame, this->dst_frame));
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "test_depth");

	TargetLocator locator;
	locator.setup();

	ros::spin();

	return 0;
}
