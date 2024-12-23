

#include "rclcpp/rclcpp.hpp"
#include "sicks300_ros2_scan_merger/sicks300_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);


    auto scan_merger_node = std::make_shared<ScanMerger>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(scan_merger_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}