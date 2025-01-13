#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filters/filter_base.hpp>
#include <robot_self_filter/self_mask.h>

namespace filters
{

/// Simple interface for classes that remove the robot's own geometry from sensor data
class SelfFilterInterface
{
public:
  virtual ~SelfFilterInterface() = default;

  virtual void getLinkNames(std::vector<std::string> &frames) = 0;

  /**
   * @brief Remove parts of the robot from PointCloud2 data
   * @param[in]  cloud2       Input cloud (sensor_msgs::PointCloud2)
   * @param[in]  sensor_frame Frame of the sensor producing this cloud
   * @param[out] out2         Output cloud with robot points removed
   * @param[out] input_size   Original cloud size
   * @param[out] output_size  Filtered cloud size
   */
  virtual bool fillPointCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud2,
                               const std::string &sensor_frame,
                               sensor_msgs::msg::PointCloud2 &out2,
                               int &input_size,
                               int &output_size) = 0;
};

/**
 * @brief A filter to remove parts of the robot seen in a pointcloud
 *
 * @tparam PointT  The point type (e.g., pcl::PointXYZ, pcl::PointXYZRGB, etc.)
 */
template <typename PointT>
class SelfFilter : public FilterBase<pcl::PointCloud<PointT>>, public SelfFilterInterface
{
public:
  using PointCloud = pcl::PointCloud<PointT>;

  /**
   * @brief Constructor. The node must already have declared (or be capable of declaring) parameters used here
   * @param node  The rclcpp::Node on which to retrieve parameters and do logging
   */
  explicit SelfFilter(const rclcpp::Node::SharedPtr &node)
    : node_(node)
    , tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock()))
    , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    // Declare and read typical parameters
    node_->declare_parameter<double>("min_sensor_dist",
                                     0.01,
                                     rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<double>("self_see_default_padding",
                                     0.01,
                                     rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<double>("self_see_default_scale",
                                     1.0,
                                     rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<bool>("keep_organized",
                                   false,
                                   rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<bool>("zero_for_removed_points",
                                   false,
                                   rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<bool>("invert",
                                   false,
                                   rcl_interfaces::msg::ParameterDescriptor());

    node_->get_parameter("min_sensor_dist", min_sensor_dist_);
    node_->get_parameter("self_see_default_padding", default_padding_);
    node_->get_parameter("self_see_default_scale", default_scale_);
    node_->get_parameter("keep_organized", keep_organized_);
    node_->get_parameter("zero_for_removed_points", zero_for_removed_points_);
    node_->get_parameter("invert", invert_);

    // Example: read array of link names from a parameter
    // Then for each link, read scale/padding if they exist.
    node_->declare_parameter<std::vector<std::string>>("self_see_links.names",
                                                       std::vector<std::string>(),
                                                       rcl_interfaces::msg::ParameterDescriptor());
    std::vector<std::string> link_names;
    node_->get_parameter("self_see_links.names", link_names);

    std::vector<robot_self_filter::LinkInfo> links;
    if (link_names.empty())
    {
      RCLCPP_WARN(node_->get_logger(),
                  "No links specified for self filtering (self_see_links.names is empty).");
    }
    else
    {
      for (auto &lname : link_names)
      {
        robot_self_filter::LinkInfo li;
        li.name = lname;

        // Parameter naming pattern (example):
        // self_see_links.<link_name>.padding
        // self_see_links.<link_name>.scale
        std::string padding_key = std::string("self_see_links.") + lname + ".padding";
        std::string scale_key   = std::string("self_see_links.") + lname + ".scale";

        // Declare these with defaults
        node_->declare_parameter<double>(padding_key,
                                         default_padding_,
                                         rcl_interfaces::msg::ParameterDescriptor());
        node_->declare_parameter<double>(scale_key,
                                         default_scale_,
                                         rcl_interfaces::msg::ParameterDescriptor());

        double link_padding = default_padding_;
        double link_scale   = default_scale_;
        node_->get_parameter(padding_key, link_padding);
        node_->get_parameter(scale_key, link_scale);

        li.padding = link_padding;
        li.scale   = link_scale;
        links.push_back(li);
      }
    }

    // NOTE: The constructor of SelfMask requires 3 parameters:
    //       (rclcpp::Node::SharedPtr, tf2_ros::Buffer&, const std::vector<LinkInfo>&).
    //       So we must pass node_, *tf_buffer_, and links:
    sm_.reset(new robot_self_filter::SelfMask<PointT>(node_, *tf_buffer_, links));

    if (!sensor_frame_.empty())
    {
      RCLCPP_INFO(node_->get_logger(),
                  "SelfFilter: removing shadow points for sensor in frame '%s'. min_sensor_dist = %f",
                  sensor_frame_.c_str(), min_sensor_dist_);
    }
  }

  /**
   * @brief Destructor
   */
  ~SelfFilter() override = default;

  /**
   * @brief Configure, if needed. In the original ROS1 code, we read the "invert" parameter here.
   *        We do the same in the constructor for ROS2. This remains for API compatibility.
   */
  bool configure() override
  {
    if (invert_)
    {
      RCLCPP_INFO(node_->get_logger(), "SelfFilter is in 'invert' mode.");
    }
    return true;
  }

  /**
   * @brief Set sensor frame (optional usage)
   */
  void setSensorFrame(const std::string &frame)
  {
    sensor_frame_ = frame;
  }

  /**
   * @brief Overload to update the filter with a custom sensor frame
   */
  bool updateWithSensorFrame(const PointCloud &data_in,
                             PointCloud &data_out,
                             const std::string &sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out);
  }

  /**
   * @brief Overload to update the filter with a custom sensor frame and also produce "diff" cloud
   */
  bool updateWithSensorFrame(const PointCloud &data_in,
                             PointCloud &data_out,
                             PointCloud &data_diff,
                             const std::string &sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out, data_diff);
  }

  /**
   * @brief Update the filter
   */
  bool update(const PointCloud &data_in, PointCloud &data_out) override
  {
    std::vector<int> keep(data_in.points.size(), 0);

    if (sensor_frame_.empty())
    {
      // Just do containment test
      sm_->maskContainment(data_in, keep);
    }
    else
    {
      // Intersection test
      sm_->maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    }

    fillResult(data_in, keep, data_out);
    return true;
  }

  /**
   * @brief Update the filter, also producing a cloud of "removed" points
   */
  bool update(const PointCloud &data_in, PointCloud &data_out, PointCloud &data_diff)
  {
    std::vector<int> keep(data_in.points.size(), 0);

    if (sensor_frame_.empty())
    {
      sm_->maskContainment(data_in, keep);
    }
    else
    {
      sm_->maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    }

    fillResult(data_in, keep, data_out);
    fillDiff(data_in, keep, data_diff);
    return true;
  }

  /**
   * @brief Bulk update for multiple clouds
   */
  bool update(const std::vector<PointCloud> &data_in,
              std::vector<PointCloud> &data_out)
  {
    bool result = true;
    data_out.resize(data_in.size());

    for (size_t i = 0; i < data_in.size(); ++i)
    {
      if (!update(data_in[i], data_out[i]))
        result = false;
    }
    return result;
  }

  /**
   * @brief Overload with sensor frame
   */
  bool updateWithSensorFrame(const std::vector<PointCloud> &data_in,
                             std::vector<PointCloud> &data_out,
                             const std::string &sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out);
  }

  /**
   * @brief Implementation of the SelfFilterInterface method for sensor_msgs::PointCloud2
   */
  bool fillPointCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud2,
                       const std::string &sensor_frame,
                       sensor_msgs::msg::PointCloud2 &out2,
                       int &input_size,
                       int &output_size) override
  {
    // Convert from ROS2 message to PCL
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud2, *cloud);

    // Filter
    pcl::PointCloud<PointT> out;
    updateWithSensorFrame(*cloud, out, sensor_frame);

    // Convert back to ROS2 message
    pcl::toROSMsg(out, out2);
    out2.header = cloud2->header;  // preserve stamp, frame, etc.

    input_size  = static_cast<int>(cloud->points.size());
    output_size = static_cast<int>(out.points.size());
    return true;
  }

  /**
   * @brief Get underlying SelfMask
   */
  robot_self_filter::SelfMask<PointT> *getSelfMask()
  {
    return sm_.get();
  }

  /**
   * @brief Return link names in the SelfMask
   */
  void getLinkNames(std::vector<std::string> &frames) override
  {
    if (sm_)
      sm_->getLinkNames(frames);
  }

protected:
  /**
   * @brief Mark removed points (either as zero or NaN) and keep robot-free points
   */
  void fillResult(const PointCloud &data_in, const std::vector<int> &keep, PointCloud &data_out)
  {
    const size_t np = data_in.points.size();
    data_out.header = data_in.header;
    data_out.points.clear();
    data_out.points.reserve(np);

    // The replacement "blank" point
    PointT blank_pt;
    if (zero_for_removed_points_)
    {
      blank_pt.x = 0.0f;
      blank_pt.y = 0.0f;
      blank_pt.z = 0.0f;
    }
    else
    {
      blank_pt.x = std::numeric_limits<float>::quiet_NaN();
      blank_pt.y = std::numeric_limits<float>::quiet_NaN();
      blank_pt.z = std::numeric_limits<float>::quiet_NaN();
    }

    // Copy "outside" points, fill with blank (or NaN) for "inside" if keep_organized_ is set
    for (size_t i = 0; i < np; ++i)
    {
      bool outside = (keep[i] == robot_self_filter::OUTSIDE);

      // Normal (non-inverted) mode: outside means keep
      if (outside && !invert_)
      {
        data_out.points.push_back(data_in.points[i]);
      }
      // Inverted mode: inside means keep
      else if (!outside && invert_)
      {
        data_out.points.push_back(data_in.points[i]);
      }
      else if (keep_organized_)
      {
        // Keep place for organized cloud
        data_out.points.push_back(blank_pt);
      }
      // if keep_organized_ is false, we skip the inside points
    }

    if (keep_organized_)
    {
      data_out.width  = data_in.width;
      data_out.height = data_in.height;
    }
    else
    {
      data_out.width  = static_cast<uint32_t>(data_out.points.size());
      data_out.height = 1;
    }
  }

  /**
   * @brief Fill a "diff" cloud with points that were removed
   */
  void fillDiff(const PointCloud &data_in, const std::vector<int> &keep, PointCloud &data_out)
  {
    const size_t np = data_in.points.size();
    data_out.header = data_in.header;
    data_out.points.clear();
    data_out.points.reserve(np);

    for (size_t i = 0; i < np; ++i)
    {
      bool outside = (keep[i] == robot_self_filter::OUTSIDE);

      // If the main output is "outside" points, then the diff output is the "inside" points, etc.
      if ((outside && invert_) || (!outside && !invert_))
      {
        data_out.points.push_back(data_in.points[i]);
      }
    }

    data_out.width  = static_cast<uint32_t>(data_out.points.size());
    data_out.height = 1;
  }

protected:
  rclcpp::Node::SharedPtr node_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Robot Self Mask
  std::shared_ptr<robot_self_filter::SelfMask<PointT>> sm_;

  // Stored parameters
  bool invert_                 = false;
  bool keep_organized_         = false;
  bool zero_for_removed_points_ = false;
  double min_sensor_dist_      = 0.01;
  double default_padding_      = 0.01;
  double default_scale_        = 1.0;
  std::string sensor_frame_;
};

}  // namespace filters

#endif  // FILTERS_SELF_SEE_H_
