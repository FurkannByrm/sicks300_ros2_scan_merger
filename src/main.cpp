/*
 * SICK S300 2 ROS NODE
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of sicks300_2 project.
 * 
 * All rights reserved.
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sicks300_ros2_scan_merger/sicks300_node.hpp"

/* Main */
int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor exe;
	auto node = std::make_shared<SickS300>("sicks300");
	exe.add_node(node->get_node_base_interface());
	exe.spin();
	rclcpp::shutdown();
	return 0;
}
