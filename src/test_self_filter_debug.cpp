#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <robot_self_filter/self_see_filter.h>

class SelfFilterDebugNode : public rclcpp::Node
{
public:
  SelfFilterDebugNode() : Node("self_filter_debug")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Self Filter Debug Node");
    
    // Create the self filter
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZ>>(node_ptr);
    filter_->configure();
    
    // Check how many collision bodies were loaded
    std::vector<std::string> link_names;
    filter_->getLinkNames(link_names);
    
    RCLCPP_INFO(this->get_logger(), "Loaded %zu collision bodies:", link_names.size());
    for (const auto& name : link_names) {
      RCLCPP_INFO(this->get_logger(), "  - %s", name.c_str());
    }
    
    if (link_names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "NO COLLISION BODIES LOADED! This will cause all points to be filtered!");
      return;
    }
    
    // Subscribe to point cloud
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&SelfFilterDebugNode::pointCloudCallback, this, std::placeholders::_1));
      
    // Publisher for debug info
    debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/filtered_cloud", 10);
    
    RCLCPP_INFO(this->get_logger(), "Debug node ready, subscribed to /livox/lidar");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", 
                msg->width * msg->height);
    
    // Test the filter
    sensor_msgs::msg::PointCloud2 filtered_cloud;
    int input_size, output_size;
    
    bool success = filter_->fillPointCloud2(msg, "livox_frame", filtered_cloud, input_size, output_size);
    
    RCLCPP_INFO(this->get_logger(), "Filter result: %s", success ? "SUCCESS" : "FAILED");
    RCLCPP_INFO(this->get_logger(), "Input points: %d, Output points: %d", input_size, output_size);
    RCLCPP_INFO(this->get_logger(), "Filtering ratio: %.2f%% kept", 
                100.0 * output_size / input_size);
    
    if (output_size == 0) {
      RCLCPP_ERROR(this->get_logger(), "ALL POINTS WERE FILTERED OUT!");
      RCLCPP_ERROR(this->get_logger(), "This suggests:");
      RCLCPP_ERROR(this->get_logger(), "  1. All points are marked as INSIDE the robot");
      RCLCPP_ERROR(this->get_logger(), "  2. Transform issues");
      RCLCPP_ERROR(this->get_logger(), "  3. Collision shapes are too large");
      RCLCPP_ERROR(this->get_logger(), "  4. Wrong invert parameter");
    } else if (output_size == input_size) {
      RCLCPP_WARN(this->get_logger(), "NO POINTS WERE FILTERED!");
      RCLCPP_WARN(this->get_logger(), "This suggests:");
      RCLCPP_WARN(this->get_logger(), "  1. No collision bodies loaded");
      RCLCPP_WARN(this->get_logger(), "  2. Transform failures");
      RCLCPP_WARN(this->get_logger(), "  3. Collision shapes are too small");
    } else {
      RCLCPP_INFO(this->get_logger(), "Filtering working correctly!");
    }
    
    // Publish for visualization
    debug_pub_->publish(filtered_cloud);
    
    // Only run once for debugging
    static bool first_run = true;
    if (first_run) {
      first_run = false;
      
      // Test a few specific points
      auto self_mask = filter_->getSelfMaskPtr();
      if (self_mask) {
        // Test point at robot base
        tf2::Vector3 test_pt1(0.0, 0.0, 0.2);  // 20cm above base
        int mask1 = self_mask->getMaskContainment(test_pt1);
        RCLCPP_INFO(this->get_logger(), "Test point (0,0,0.2): %s", 
                    mask1 == 0 ? "INSIDE" : (mask1 == 1 ? "OUTSIDE" : "SHADOW"));
        
        // Test point far away
        tf2::Vector3 test_pt2(5.0, 5.0, 5.0);  // 5m away
        int mask2 = self_mask->getMaskContainment(test_pt2);
        RCLCPP_INFO(this->get_logger(), "Test point (5,5,5): %s", 
                    mask2 == 0 ? "INSIDE" : (mask2 == 1 ? "OUTSIDE" : "SHADOW"));
      }
    }
  }
  
  std::shared_ptr<filters::SelfFilter<pcl::PointXYZ>> filter_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfFilterDebugNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 