#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle)
	: 	nodeHandle_(nodeHandle),
		scanTopic_("/scan"),
		subscriberQueueSize_(10),
		twistTopic_("/twist"),
		publisherQueueSize_(10),
		commandAngularKp_(0.5),
		commandLinearX_(0.5)
{
	if (!readParameters()){
		ROS_ERROR("Could not read params.");
		ros::requestShutdown();
	}
	scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscriberQueueSize_, &HuskyHighlevelController::scanCallback, this);
	twistPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(twistTopic_, publisherQueueSize_);
	visPublisher =  nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
}

HuskyHighlevelController::~HuskyHighlevelController(){

}

bool HuskyHighlevelController::readParameters(){
	if(!nodeHandle_.getParam("/husky_highlevel_controller/scan_subscriber_queue_size", subscriberQueueSize_)){
		return false;
	}
	if(!nodeHandle_.getParam("/husky_highlevel_controller/scan_subscriber_topic_name", scanTopic_)){
		return false;
	}
	if(!nodeHandle_.getParam("/husky_highlevel_controller/twist_publisher_queue_size", publisherQueueSize_)){
		return false;
	}
	if(!nodeHandle_.getParam("/husky_highlevel_controller/twist_publisher_topic_name", twistTopic_)){
		return false;
	}
	if(!nodeHandle_.getParam("/husky_highlevel_controller/command_angular_kp", commandAngularKp_)){
		return false;
	}
	if(!nodeHandle_.getParam("/husky_highlevel_controller/command_linear_x", commandLinearX_)){
		return false;
	}
	return true;
}

void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	float minDistance = 100.0;
	int minDistanceIndex = 0;
	float minAngle = 0;

	float markerPosX = 0.0;
	float markerPosY = 0.0;

	int i =0;
	float angle = msg->angle_min;

	while(angle <= msg->angle_max){
		if(msg->ranges[i]<minDistance){
			minDistance= msg->ranges[i];
			minDistanceIndex= i;
		}
		i++;
		angle += msg->angle_increment;
	}
	
	minAngle = msg->angle_min+minDistanceIndex*msg->angle_increment;
	ROS_INFO("Min Distance: %f, angle: %f", minDistance, minAngle);

	geometry_msgs::Twist msgTwist;
	msgTwist.linear.x = commandLinearX_;
	msgTwist.angular.z = commandAngularKp_*minAngle;

	twistPublisher_.publish(msgTwist);
	HuskyHighlevelController::publishMarker(minDistance, minAngle, msg->header.frame_id);
}

void HuskyHighlevelController::publishMarker(float distance, float angle, const std::string& frame_id){
	visualization_msgs::Marker marker;

	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = distance*cos(angle);
	marker.pose.position.y = distance*sin(angle);
	marker.pose.position.z = 1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	visPublisher.publish(marker);
}

} /* namespace */
