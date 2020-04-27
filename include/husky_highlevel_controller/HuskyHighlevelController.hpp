#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <string.h>
#include "math.h"

using namespace std;

namespace husky_highlevel_controller {
class HuskyHighlevelController {
	public:
		/*!
		 * Constructor.
		 */
		HuskyHighlevelController(ros::NodeHandle& nodeHandle);

		/*!
		 * Destructor.
		 */
		virtual ~HuskyHighlevelController();

	private:
		bool readParameters();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		void publishMarker(float distance, float angle, const std::string& frame_id);
		
		ros::NodeHandle nodeHandle_;
		ros::Subscriber scanSubscriber_;
		ros::Publisher twistPublisher_;
		ros::Publisher visPublisher;
		string scanTopic_;
		int subscriberQueueSize_;
		string twistTopic_;
		int publisherQueueSize_;
		float commandAngularKp_;
		float commandLinearX_;
};
}
