

// C++
#include <chrono>
#include <thread>

// ROS
#include "rclcpp/qos.hpp"
#include "nav2_util/node_utils.hpp"

#include "sicks300_ros2_scan_merger/sicks300_node.hpp"

using namespace std::chrono_literals;

SickS300::SickS300(const std::string& name, bool intra_process_comms) : 
					rclcpp_lifecycle::LifecycleNode(name, rclcpp::NodeOptions()
					.use_intra_process_comms(intra_process_comms)), 
					synced_ros_time_(this->now()), 
					synced_time_ready_(false), 
					synced_sick_stamp_(0){
}

SickS300::~SickS300(){
	if (timer_) {
		timer_->cancel();
		timer_.reset();
	}
}

rclcpp_CallReturn SickS300::on_configure(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Configuring the node...");

	// Declare and read parameters
	nav2_util::declare_parameter_if_not_declared(this, "port", 
		rclcpp::ParameterValue("/dev/ttyUSB0"), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("USB port of the scanner"));
	this->get_parameter("port", port_);
	RCLCPP_INFO(this->get_logger(), "The parameter port is set to: %s", port_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, 
		"baud", rclcpp::ParameterValue(500000), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Baudrate to communicate with the laser scanner"));
	this->get_parameter("baud", baud_);
	RCLCPP_INFO(this->get_logger(), "The parameter baud is set to: %i", baud_);

	nav2_util::declare_parameter_if_not_declared(this, 
		"scan_id", rclcpp::ParameterValue(7), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Identifier of the scanner"));
	this->get_parameter("scan_id", scan_id_);
	RCLCPP_INFO(this->get_logger(), "The parameter scan_id is set to: %i", scan_id_);

	nav2_util::declare_parameter_if_not_declared(this, 
		"inverted", rclcpp::ParameterValue(false), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Option to invert the direction of the measurements"));
	this->get_parameter("inverted", inverted_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter inverted is set to: %s", inverted_ ? "true" : "false");

	nav2_util::declare_parameter_if_not_declared(this, 
		"scan_topic", rclcpp::ParameterValue("scan"), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The topic where the laser scan will be published"));
	this->get_parameter("scan_topic", scan_topic_);
	RCLCPP_INFO(this->get_logger(), "The parameter scan_topic is set to: %s", scan_topic_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, 
		"frame_id", rclcpp::ParameterValue("base_laser_link"), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The frame of the scanner"));
	this->get_parameter("frame_id", frame_id_);
	RCLCPP_INFO(this->get_logger(), "The parameter frame_id is set to: %s", frame_id_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, 
		"scan_duration", rclcpp::ParameterValue(0.025), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Time between laser scans"));
	this->get_parameter("scan_duration", scan_duration_);
	RCLCPP_INFO(this->get_logger(), "The parameter scan_duration is set to: %f", scan_duration_);

	nav2_util::declare_parameter_if_not_declared(this, 
		"scan_cycle_time", rclcpp::ParameterValue(0.040), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Cycle time of the scan"));
	this->get_parameter("scan_cycle_time", scan_cycle_time_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter scan_cycle_time is set to: %f", scan_cycle_time_);

	nav2_util::declare_parameter_if_not_declared(this, 
		"scan_delay", rclcpp::ParameterValue(0.075), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Delay of the scan"));
	this->get_parameter("scan_delay", scan_delay_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter scan_delay is set to: %f", scan_delay_);

	nav2_util::declare_parameter_if_not_declared(this, 
		"debug", rclcpp::ParameterValue(false), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Option to toggle scanner debugging information"));
	this->get_parameter("debug", debug_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter debug is set to: %s", debug_ ? "true" : "false");

	nav2_util::declare_parameter_if_not_declared(this, 
		"communication_timeout", rclcpp::ParameterValue(0.2), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Timeout to shutdown the node"));
	this->get_parameter("communication_timeout", communication_timeout_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter communication_timeout is set to: %f", communication_timeout_);

	// Read 'fields' params. Set 1 by default to be backwards compatible
	// TODO: Change this when ROS will support YAML mixed types
	ScannerSickS300::ParamType param;
	param.range_field = 1;
	nav2_util::declare_parameter_if_not_declared(this, 
		"fields.1.scale", rclcpp::ParameterValue(0.01), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Scale of the field"));
	this->get_parameter("fields.1.scale", param.dScale);
	RCLCPP_INFO(this->get_logger(), "The parameter field.1.scale is set to: %f", param.dScale);

	nav2_util::declare_parameter_if_not_declared(this, 
		"fields.1.start_angle", rclcpp::ParameterValue(-135.0 / 180.0 * M_PI), 
		rcl_interfaces::msg::ParameterDescriptor()
		.set__description("Start angle of the field"));
	this->get_parameter("fields.1.start_angle", param.dStartAngle);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter field.1.start_angle is set to: %f", param.dStartAngle);

	nav2_util::declare_parameter_if_not_declared(this, 
		"fields.1.stop_angle", rclcpp::ParameterValue(135.0 / 180.0 * M_PI), 
		rcl_interfaces::msg::ParameterDescriptor()
		.set__description("Stop angle of the field"));
	this->get_parameter("fields.1.stop_angle", param.dStopAngle);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter field.1.stop_angle is set to: %f", param.dStopAngle);
	scanner_.setRangeField(1, param);

	// Configure the publishers
	auto latched_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
	laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
		scan_topic_, rclcpp::SensorDataQoS());
	in_standby_pub_ = this->create_publisher<std_msgs::msg::Bool>(
		scan_topic_ + "/standby", latched_profile);
	diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
		"/diagnostics", rclcpp::QoS(1));

	// Open the laser scanner
	bool bOpenScan = this->open();
	if (!bOpenScan){
		RCLCPP_ERROR(this->get_logger(), 
			"...scanner not available on port %s. Please, try again.", port_.c_str());
		return rclcpp_CallReturn::FAILURE;
	}else{
		// Wait for scan to get ready if successful
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		RCLCPP_INFO(this->get_logger(), 
			"...scanner opened successfully on port %s", port_.c_str());
		
		return rclcpp_CallReturn::SUCCESS;
	}
}

rclcpp_CallReturn SickS300::on_activate(const rclcpp_lifecycle::State & state){
	LifecycleNode::on_activate(state);
	RCLCPP_INFO(this->get_logger(), "Activating the node...");

	timer_ = this->create_wall_timer(std::chrono::duration<double>(scan_cycle_time_), 
		std::bind(&SickS300::receiveScan, this));

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS300::on_deactivate(const rclcpp_lifecycle::State & state){
	LifecycleNode::on_deactivate(state);
	RCLCPP_INFO(this->get_logger(), "Deactivating the node...");

	if (timer_) {
		timer_->cancel();
		timer_.reset();
	}

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS300::on_cleanup(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Cleaning the node...");

	// Release the shared pointers
	laser_scan_pub_.reset();
	in_standby_pub_.reset();
	diag_pub_.reset();
	timer_.reset();

	return rclcpp_CallReturn::SUCCESS;
}

rclcpp_CallReturn SickS300::on_shutdown(const rclcpp_lifecycle::State & state){
	RCLCPP_INFO(this->get_logger(), "Shutdown the node from state %s.", state.label().c_str());

	// Release the shared pointers
	laser_scan_pub_.reset();
	in_standby_pub_.reset();
	diag_pub_.reset();
	timer_.reset();

	return rclcpp_CallReturn::SUCCESS;
}

bool SickS300::open(){
	return scanner_.open(port_.c_str(), baud_, scan_id_);
}

bool SickS300::receiveScan(){
	std::vector<double> ranges, rangeAngles, intensities;
	unsigned int iSickTimeStamp, iSickNow;

	int result = scanner_.getScan(ranges, rangeAngles, intensities, 
		iSickTimeStamp, iSickNow, debug_);
	static rclcpp::Time pointTimeCommunicationOK(this->now());

	if (result){
		if (scanner_.isInStandby()){
			publishWarn("scanner in standby");
			RCLCPP_WARN_THROTTLE(this->get_logger(), 
				*this->get_clock(), 30, "scanner on port %s in standby", port_.c_str());
			publishStandby(true);
		}else{
			publishStandby(false);
			publishLaserScan(ranges, rangeAngles, intensities, iSickTimeStamp, iSickNow);
		}

		pointTimeCommunicationOK = this->now();
	}else{
		rclcpp::Duration diff(this->now() - pointTimeCommunicationOK);

		if (diff.seconds() > communication_timeout_){
			RCLCPP_WARN(this->get_logger(), "Communication timeout");
			return false;
		}
	}

	return true;
}

void SickS300::publishStandby(bool in_standby){
	in_standby_.data = in_standby;
	in_standby_pub_->publish(in_standby_);
}

void SickS300::publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, 
	std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, unsigned int iSickNow){
	// Fill message
	int start_scan = 0;
	int num_readings = vdDistM.size(); // initialize with max scan size
	int stop_scan = vdDistM.size();

	// Sync handling: find out exact scan time by using the syncTime-syncStamp pair:
	// Timestamp: "This counter is internally incremented at each scan, i.e. every 40 ms (S300)"
	if (iSickNow != 0){
		synced_ros_time_ = this->now() - rclcpp::Duration::from_seconds(scan_cycle_time_);
		synced_sick_stamp_ = iSickNow;
		synced_time_ready_ = true;

		RCLCPP_DEBUG(this->get_logger(), "Got iSickNow, store sync-stamp: %d", synced_sick_stamp_);
	}else{
		synced_time_ready_ = false;
	}

	// Create LaserScan message
	sensor_msgs::msg::LaserScan laserScan;
	if (synced_time_ready_){
		double timeDiff = static_cast<int>(iSickTimeStamp - synced_sick_stamp_) * scan_cycle_time_;
		laserScan.header.stamp = synced_ros_time_ + rclcpp::Duration::from_seconds(timeDiff);

		RCLCPP_DEBUG(this->get_logger(), "Time::now() - calculated sick time stamp = %f", 
			(this->now() - laserScan.header.stamp).seconds());
	}else{
		laserScan.header.stamp = this->now();
	}

	// Fill message
	laserScan.header.frame_id = frame_id_;
	laserScan.angle_increment = vdAngRAD[start_scan + 1] - vdAngRAD[start_scan];
	laserScan.range_min = 0.001;
	laserScan.range_max = 29.5; // though the specs state otherwise, the max range reported by the scanner is 29.96m
	laserScan.time_increment = (scan_duration_) / (vdDistM.size());

	// Rescale scan
	num_readings = vdDistM.size();
	laserScan.angle_min = vdAngRAD[start_scan]; // first ScanAngle
	laserScan.angle_max = vdAngRAD[stop_scan - 1]; // last ScanAngle
	laserScan.ranges.resize(num_readings);
	laserScan.intensities.resize(num_readings);

	// Check for inverted laser
	if (inverted_){
		// to be really accurate, we now invert time_increment
		// laserScan.header.stamp = rclcpp::Time(laserScan.header.stamp) + 
		// rclcpp::Duration::from_seconds(scanDuration_); //adding of the sum over all negative increments would be mathematically correct, but looks worse.
		laserScan.time_increment = - laserScan.time_increment;
	}else{
		laserScan.header.stamp = rclcpp::Time(laserScan.header.stamp) - 
			rclcpp::Duration::from_seconds(scan_duration_) - 
			rclcpp::Duration::from_seconds(scan_delay_); //to be consistent with the omission of the addition above
	}

	for (int i = 0; i < (stop_scan - start_scan); i++){
		if (inverted_){
			laserScan.ranges[i] = vdDistM[stop_scan-1-i];
			laserScan.intensities[i] = vdIntensAU[stop_scan-1-i];
		}else{
			laserScan.ranges[i] = vdDistM[start_scan + i];
			laserScan.intensities[i] = vdIntensAU[start_scan + i];
		}
	}

	// Publish Laserscan-message
	laser_scan_pub_->publish(laserScan);

	// Diagnostics
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = "sick scanner running";
	diag_pub_->publish(diagnostics);
}

void SickS300::publishError(std::string error){
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = error;
	diag_pub_->publish(diagnostics);
}

void SickS300::publishWarn(std::string warn){
	diagnostic_msgs::msg::DiagnosticArray diagnostics;
	diagnostics.header.stamp = this->now();
	diagnostics.status.resize(1);
	diagnostics.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
	diagnostics.status[0].name = this->get_namespace();
	diagnostics.status[0].message = warn;
	diag_pub_->publish(diagnostics);
}

 ScanMerger::ScanMerger() : Node("scan_merger")
  {
    initialize_params();
    refresh_params();

    laser1_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    laser2_ = std::make_shared<sensor_msgs::msg::LaserScan>();

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic1_, default_qos, std::bind(&ScanMerger::scan_callback1, this, std::placeholders::_1));
    sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic2_, default_qos, std::bind(&ScanMerger::scan_callback2, this, std::placeholders::_1));

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS());
  }

  void ScanMerger::scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    laser1_ = _msg;
    update_point_cloud_rgb();
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
    //         _msg->ranges[100]);
  }
  void ScanMerger::scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    laser2_ = _msg;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
    //         _msg->ranges[100]);
  }

  void ScanMerger::update_point_cloud_rgb()
  {
    refresh_params();
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    std::vector<std::array<float, 2>> scan_data;
    int count = 0;
    float min_theta = 0;
    float max_theta = 0;
    if (show1_ && laser1_)
    {
      float temp_min_, temp_max_;
      if( laser1_->angle_min < laser1_->angle_max){
        temp_min_ = laser1_->angle_min;
        temp_max_ = laser1_->angle_max;
      } else{
        temp_min_ = laser1_->angle_max;
        temp_max_ = laser1_->angle_min;
      }
      for (float i = temp_min_; i <= temp_max_ && count < laser1_->ranges.size();
           i += laser1_->angle_increment)
      {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(laser1R_, laser1G_, laser1B_);
        int used_count_ = count;
        if (flip1_)
        {
          used_count_ = (int)laser1_->ranges.size() - 1 - count;
        }
        float temp_x = laser1_->ranges[used_count_] * std::cos(i);
        float temp_y = laser1_->ranges[used_count_] * std::sin(i);
        pt.x =
            temp_x * std::cos(laser1Alpha_ * M_PI / 180) - temp_y * std::sin(laser1Alpha_ * M_PI / 180) + laser1XOff_;
        pt.y =
            temp_x * std::sin(laser1Alpha_ * M_PI / 180) + temp_y * std::cos(laser1Alpha_ * M_PI / 180) + laser1YOff_;
        pt.z = laser1ZOff_;
        if ((i < (laser1AngleMin_ * M_PI / 180)) || (i > (laser1AngleMax_ * M_PI / 180)))
        {
          if (inverse1_)
          {
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        else
        {
          if (!inverse1_)
          {
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        count++;
      }
    }

    count = 0;
    if (show2_ && laser2_)
    {
      float temp_min_, temp_max_;
      if( laser2_->angle_min < laser2_->angle_max){
        temp_min_ = laser2_->angle_min;
        temp_max_ = laser2_->angle_max;
      } else{
        temp_min_ = laser2_->angle_max;
        temp_max_ = laser2_->angle_min;
      }
      for (float i = temp_min_; i <= temp_max_ && count < laser2_->ranges.size();
           i += laser2_->angle_increment)
      {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(laser2R_, laser2G_, laser2B_);

        int used_count_ = count;
        if (flip2_)
        {
          used_count_ = (int)laser2_->ranges.size() - 1 - count;
        }

        float temp_x = laser2_->ranges[used_count_] * std::cos(i);
        float temp_y = laser2_->ranges[used_count_] * std::sin(i);
        pt.x =
            temp_x * std::cos(laser2Alpha_ * M_PI / 180) - temp_y * std::sin(laser2Alpha_ * M_PI / 180) + laser2XOff_;
        pt.y =
            temp_x * std::sin(laser2Alpha_ * M_PI / 180) + temp_y * std::cos(laser2Alpha_ * M_PI / 180) + laser2YOff_;
        pt.z = laser2ZOff_;
        if ((i < (laser2AngleMin_ * M_PI / 180)) || (i > (laser2AngleMax_ * M_PI / 180)))
        {
          if (inverse2_)
          {
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        else
        {
          if (!inverse2_)
          {
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        count++;
      }
    }

    auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *pc2_msg_);
    pc2_msg_->header.frame_id = cloudFrameId_;
    pc2_msg_->header.stamp = now();
    pc2_msg_->is_dense = false;
    point_cloud_pub_->publish(*pc2_msg_);
  }

  float ScanMerger::GET_R(float x, float y)
  {
    return sqrt(x * x + y * y);
  }
  float ScanMerger::GET_THETA(float x, float y)
  {
    float temp_res;
    if ((x != 0))
    {
      temp_res = atan(y / x);
    }
    else
    {
      if (y >= 0)
      {
        temp_res = M_PI / 2;
      }
      else
      {
        temp_res = -M_PI / 2;
      }
    }
    if (temp_res > 0)
    {
      if (y < 0)
      {
        temp_res -= M_PI;
      }
    }
    else if (temp_res < 0)
    {
      if (x < 0)
      {
        temp_res += M_PI;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "x: '%f', y: '%f', a: '%f'", x, y, temp_res);

    return temp_res;
  }
  float ScanMerger::interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle)
  {
    return (magnitude_1 + current_angle * ((magnitude_2 - magnitude_1) / (angle_2 - angle_1)));
  }
  void ScanMerger::initialize_params()
  {
    this->declare_parameter("pointCloudTopic", "base/custom_cloud");
    this->declare_parameter("pointCloutFrameId", "laser");

    this->declare_parameter("scanTopic1", "lidar_front_right/scan");
    this->declare_parameter("laser1XOff", -0.45);
    this->declare_parameter("laser1YOff", 0.24);
    this->declare_parameter("laser1ZOff", 0.0);
    this->declare_parameter("laser1Alpha", 45.0);
    this->declare_parameter("laser1AngleMin", -181.0);
    this->declare_parameter("laser1AngleMax", 181.0);
    this->declare_parameter("laser1R", 255);
    this->declare_parameter("laser1G", 0);
    this->declare_parameter("laser1B", 0);
    this->declare_parameter("show1", true);
    this->declare_parameter("flip1", false);
    this->declare_parameter("inverse1", false);

    this->declare_parameter("scanTopic2", "lidar_rear_left/scan");
    this->declare_parameter("laser2XOff", 0.315);
    this->declare_parameter("laser2YOff", -0.24);
    this->declare_parameter("laser2ZOff", 0.0);
    this->declare_parameter("laser2Alpha", 225.0);
    this->declare_parameter("laser2AngleMin", -181.0);
    this->declare_parameter("laser2AngleMax", 181.0);
    this->declare_parameter("laser2R", 0);
    this->declare_parameter("laser2G", 0);
    this->declare_parameter("laser2B", 255);
    this->declare_parameter("show2", true);
    this->declare_parameter("flip2", false);
    this->declare_parameter("inverse2", false);
  }
  void ScanMerger::refresh_params()
  {
    this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "pointCloud");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloudFrameId_, "laser");
    this->get_parameter_or<std::string>("scanTopic1", topic1_, "lidar_front_right/scan");
    this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.0);
    this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser1ZOff", laser1ZOff_, 0.0);
    this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 0.0);
    this->get_parameter_or<float>("laser1AngleMin", laser1AngleMin_, -181.0);
    this->get_parameter_or<float>("laser1AngleMax", laser1AngleMax_, 181.0);
    this->get_parameter_or<uint8_t>("laser1R", laser1R_, 0);
    this->get_parameter_or<uint8_t>("laser1G", laser1G_, 0);
    this->get_parameter_or<uint8_t>("laser1B", laser1B_, 0);
    this->get_parameter_or<bool>("show1", show1_, true);
    this->get_parameter_or<bool>("flip1", flip1_, false);
    this->get_parameter_or<bool>("inverse1", inverse1_, false);
    this->get_parameter_or<std::string>("scanTopic2", topic2_, "lidar_rear_left/scan");
    this->get_parameter_or<float>("laser2XOff", laser2XOff_, 0.0);
    this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser2ZOff", laser2ZOff_, 0.0);
    this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
    this->get_parameter_or<float>("laser2AngleMin", laser2AngleMin_, -181.0);
    this->get_parameter_or<float>("laser2AngleMax", laser2AngleMax_, 181.0);
    this->get_parameter_or<uint8_t>("laser2R", laser2R_, 0);
    this->get_parameter_or<uint8_t>("laser2G", laser2G_, 0);
    this->get_parameter_or<uint8_t>("laser2B", laser2B_, 0);
    this->get_parameter_or<bool>("show2", show2_, false);
    this->get_parameter_or<bool>("flip2", flip2_, false);
    this->get_parameter_or<bool>("inverse2", inverse2_, false);
  }
