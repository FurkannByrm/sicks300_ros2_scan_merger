
#ifndef _SICKS300_NODE_HPP_
#define _SICKS300_NODE_HPP_

// C++
#include <string>
#include <array>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>


// ROS
#include "sensor_msgs/msg/laser_scan.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/qos.hpp"
#include "nav2_util/node_utils.hpp"

// Common
#include "common/ScannerSickS300.hpp"

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rclcpp_CallReturn;

class SickS300: public rclcpp::Node, rclcpp_lifecycle::LifecycleNode{
	public:
		SickS300(const std::string& name, bool intra_process_comms = false);
		~SickS300();
		rclcpp_CallReturn on_configure(const rclcpp_lifecycle::State &);
		rclcpp_CallReturn on_activate(const rclcpp_lifecycle::State & state);
		rclcpp_CallReturn on_deactivate(const rclcpp_lifecycle::State & state);
		rclcpp_CallReturn on_cleanup(const rclcpp_lifecycle::State &);
		rclcpp_CallReturn on_shutdown(const rclcpp_lifecycle::State & state);

	private:
		rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
		rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr in_standby_pub_;
		rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
  		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
  		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
		std_msgs::msg::Bool in_standby_;
		rclcpp::Time synced_ros_time_;

		std::string frame_id_, scan_topic_, port_;
		int baud_, scan_id_;
		bool inverted_, debug_, synced_time_ready_;
		unsigned int synced_sick_stamp_;
		double scan_duration_, scan_cycle_time_, scan_delay_, communication_timeout_;
		ScannerSickS300 scanner_;
		rclcpp::TimerBase::SharedPtr timer_;
		sensor_msgs::msg::LaserScan::SharedPtr laser1_;
  		sensor_msgs::msg::LaserScan::SharedPtr laser2_;


		std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
  		bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_;
  		float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
  		uint8_t laser1R_, laser1G_, laser1B_;

  		float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
  		uint8_t laser2R_, laser2G_, laser2B_;

		bool open();
		bool receiveScan();
		void publishStandby(bool in_standby);
		void publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, 
							std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, 
							unsigned int iSickNow);
		void publishError(std::string error);
		void publishWarn(std::string warn);

		float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle);
		float GET_THETA(float x, float y);
		float GET_R(float x, float y);
     	void update_point_cloud_rgb();
		void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
		void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
};

#endif