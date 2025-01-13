#include <chrono>
#include <sstream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include "robot_self_filter/self_see_filter.h"
#include "robot_self_filter/point_ouster.h"
#include "robot_self_filter/point_hesai.h"
#include "robot_self_filter/point_pandar.h"
#include "robot_self_filter/point_robosense.h"
#include <yaml-cpp/yaml.h> // Ensure you have the yaml-cpp library installed and linked

namespace robot_self_filter
{

enum class SensorType : int
{
  XYZSensor       = 0,
  XYZRGBSensor    = 1,
  OusterSensor    = 2,
  HesaiSensor     = 3,
  RobosenseSensor = 4,
  PandarSensor    = 5,
};

class SelfFilterNode : public rclcpp::Node
{
public:
  SelfFilterNode()
  : Node("self_filter"), subscribing_(false)
  {
    // Attempt to declare use_sim_time only if not declared already
    try {
      this->declare_parameter<bool>("use_sim_time", true);
      this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    } catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException &)
    {
      // Parameter already declared, do nothing
    }

    this->declare_parameter<std::string>("sensor_frame", "Lidar");
    this->set_parameter(rclcpp::Parameter("sensor_frame", "Lidar"));
    this->declare_parameter<bool>("use_rgb", false);
    this->declare_parameter<int>("max_queue_size", 10);
    this->declare_parameter<int>("lidar_sensor_type", 0);
    this->declare_parameter<std::string>("robot_description", "");

    // ... existing parameter declarations ...

    // sensor_frame_     = this->get_parameter("sensor_frame").as_string();
    use_rgb_          = this->get_parameter("use_rgb").as_bool();
    max_queue_size_   = this->get_parameter("max_queue_size").as_int();
    int temp_sensor_type = this->get_parameter("lidar_sensor_type").as_int();
    sensor_type_      = static_cast<SensorType>(temp_sensor_type);
    
    // Add parameter logging
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  sensor_frame: %s", sensor_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  use_rgb: %s", use_rgb_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  max_queue_size: %d", max_queue_size_);
    RCLCPP_INFO(this->get_logger(), "  lidar_sensor_type: %d", temp_sensor_type);

    // ... rest of the constructor


    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
      )
    );
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pointCloudPublisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 1);

    auto period = std::chrono::milliseconds(1000);
    subscription_check_timer_ =
      this->create_wall_timer(period, std::bind(&SelfFilterNode::checkSubscriptions, this));
  }

  void initSelfFilter()
  {
    std::string robot_description_xml = this->get_parameter("robot_description").as_string();

    switch (sensor_type_)
    {
    case SensorType::XYZSensor:
      self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZ>>(this->shared_from_this());
      break;
    case SensorType::XYZRGBSensor:
      self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZRGB>>(this->shared_from_this());
      break;
    case SensorType::OusterSensor:
      self_filter_ = std::make_shared<filters::SelfFilter<PointOuster>>(this->shared_from_this());
      break;
    case SensorType::HesaiSensor:
      self_filter_ = std::make_shared<filters::SelfFilter<PointHesai>>(this->shared_from_this());
      break;
    case SensorType::RobosenseSensor:
      self_filter_ = std::make_shared<filters::SelfFilter<PointRobosense>>(this->shared_from_this());
      break;
    case SensorType::PandarSensor:
      self_filter_ = std::make_shared<filters::SelfFilter<PointPandar>>(this->shared_from_this());
      break;
    default:
      self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZ>>(this->shared_from_this());
      break;
    }

    self_filter_->getLinkNames(frames_);
  }

private:
  void checkSubscriptions()
  {
    size_t subscription_count = pointCloudPublisher_->get_subscription_count();
    if (subscription_count > 0 && !subscribing_)
    {
      this->subscribe();
      subscribing_ = true;
    }
    else if (subscription_count == 0 && subscribing_)
    {
      this->unsubscribe();
      subscribing_ = false;
    }
  }

  void subscribe()
  {
    if (frames_.empty())
    {
      no_filter_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in",
        rclcpp::SensorDataQoS(),
        std::bind(&SelfFilterNode::noFilterCallback, this, std::placeholders::_1));
    }
    else
    {
      auto sensor_data_qos_profile =
        rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile();
      sub_.subscribe(this, "cloud_in", sensor_data_qos_profile);

      message_filter_ =
        std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
          sub_,
          *tf_buffer_,
          sensor_frame_,
          static_cast<uint32_t>(max_queue_size_),
          this->get_node_logging_interface(),
          this->get_node_clock_interface(),
          std::chrono::duration<int64_t, std::nano>(1000000000)
        );

      if (!frames_.empty())
      {
        message_filter_->setTargetFrames(frames_);
        RCLCPP_INFO(this->get_logger(), "Setting %zu target frames", frames_.size());
        message_filter_->setTolerance(rclcpp::Duration::from_seconds(1.0));
      }

      message_filter_->registerCallback(
        std::bind(&SelfFilterNode::cloudCallback, this, std::placeholders::_1));
    }
  }

  void unsubscribe()
  {
    if (frames_.empty())
    {
      no_filter_sub_.reset();
    }
    else
    {
      message_filter_.reset();
    }
  }

  void noFilterCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud)
  {
    pointCloudPublisher_->publish(*cloud);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud)
  {
    RCLCPP_INFO(this->get_logger(), "Received cloud message with timestamp %f", 
                rclcpp::Time(cloud->header.stamp).seconds());
    RCLCPP_INFO(this->get_logger(), "Checking TF cache");
    checkTfCache(tf_buffer_, frames_.front());
    
    sensor_msgs::msg::PointCloud2 out2;
    int input_size  = 0;
    int output_size = 0;
    self_filter_->fillPointCloud2(cloud, sensor_frame_, out2, input_size, output_size);
    pointCloudPublisher_->publish(out2);
  }

  void checkTfCache(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string& frame);

private:
  std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;
  std::shared_ptr<filters::SelfFilterInterface> self_filter_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> message_filter_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr no_filter_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
  rclcpp::TimerBase::SharedPtr subscription_check_timer_;

  std::string sensor_frame_;
  bool        use_rgb_;
  bool        subscribing_;
  SensorType  sensor_type_;
  int         max_queue_size_;
  std::vector<std::string> frames_;
};

void SelfFilterNode::checkTfCache(
  const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
  const std::string& frame)
{
  std::string yaml_str = tf_buffer->allFramesAsYAML();
  YAML::Node yaml_node = YAML::Load(yaml_str);

  for (const auto& frame : yaml_node)
  {
    std::string frame_id = frame.first.as<std::string>();
    double most_recent_transform = frame.second["most_recent_transform"].as<double>();

    RCLCPP_INFO(
      this->get_logger(),
      "Frame: %s | Most Recent Transform Time: %.2f",
      frame_id.c_str(),
      most_recent_transform
    );
  }
}
}  // namespace robot_self_filter

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_self_filter::SelfFilterNode>();
  node->initSelfFilter();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
