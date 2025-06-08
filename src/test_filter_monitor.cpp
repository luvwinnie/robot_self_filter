#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class FilterMonitorNode : public rclcpp::Node
{
public:
  FilterMonitorNode() : Node("filter_monitor")
  {
    input_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/pointcloud", 10,
      std::bind(&FilterMonitorNode::inputCallback, this, std::placeholders::_1));
      
    output_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/pointcloud_filtered", 10,
      std::bind(&FilterMonitorNode::outputCallback, this, std::placeholders::_1));
      
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&FilterMonitorNode::printStats, this));
      
    RCLCPP_INFO(this->get_logger(), "Filter monitor started");
  }

private:
  void inputCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_input_size_ = msg->width * msg->height;
    input_count_++;
  }
  
  void outputCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_output_size_ = msg->width * msg->height;
    output_count_++;
  }
  
  void printStats()
  {
    if (input_count_ > 0 && output_count_ > 0)
    {
      double filter_ratio = 100.0 * latest_output_size_ / latest_input_size_;
      RCLCPP_INFO(this->get_logger(), 
                  "Input: %d points | Output: %d points | Kept: %.1f%% | Status: %s",
                  latest_input_size_, latest_output_size_, filter_ratio,
                  (latest_output_size_ == 0) ? "ALL FILTERED" : 
                  (latest_output_size_ == latest_input_size_) ? "NO FILTERING" : "WORKING");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for data... Input msgs: %d, Output msgs: %d", 
                  input_count_, output_count_);
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr output_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  int latest_input_size_ = 0;
  int latest_output_size_ = 0;
  int input_count_ = 0;
  int output_count_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FilterMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 