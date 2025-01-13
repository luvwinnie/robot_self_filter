#ifndef ROBOT_SELF_FILTER_SELF_MASK_
#define ROBOT_SELF_FILTER_SELF_MASK_

#include <rclcpp/rclcpp.hpp>

// TF2 and transforms
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// PCL and ROS 2 conversions
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// sensor_msgs/msg/PointCloud2 is still used to interpret the header
#include <sensor_msgs/msg/point_cloud2.hpp>

// URDF / resource retriever
#include <urdf/model.h>
#include <resource_retriever/retriever.hpp>

// Standard / Boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <string>
#include <vector>
#include <algorithm> // for std::sort

// Bodies and shapes
#include <robot_self_filter/bodies.h>  // Contains the shapes & bodies logic

using namespace urdf;

namespace robot_self_filter
{

/** \brief The possible values of a mask computed for a point */
enum
{
  INSIDE = 0,
  OUTSIDE = 1,
  SHADOW = 2,
};

struct LinkInfo
{
  std::string name;
  double padding;
  double scale;
};

/** \brief Convert a URDF pose to tf2::Transform */
static inline tf2::Transform urdfPose2TFTransform(const urdf::Pose &pose)
{
  // Build quaternion
  tf2::Quaternion q(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
  // Build translation
  tf2::Vector3 t(pose.position.x, pose.position.y, pose.position.z);
  return tf2::Transform(q, t);
}

/** \brief Convert URDF geometry to a shapes::Shape */
static shapes::Shape* constructShape(const urdf::Geometry *geom)
{
  // Equivalent to ROS_ASSERT(geom) in ROS 2:
  assert(geom && "Geometry pointer must not be null!");

  shapes::Shape *result = nullptr;
  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
    {
      auto sphere = dynamic_cast<const urdf::Sphere*>(geom);
      result = new shapes::Sphere(sphere->radius);
      break;
    }
    case urdf::Geometry::BOX:
    {
      auto box = dynamic_cast<const urdf::Box*>(geom);
      result = new shapes::Box(box->dim.x, box->dim.y, box->dim.z);
      break;
    }
    case urdf::Geometry::CYLINDER:
    {
      auto cyl = dynamic_cast<const urdf::Cylinder*>(geom);
      result = new shapes::Cylinder(cyl->radius, cyl->length);
      break;
    }
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        resource_retriever::Retriever retriever;
        resource_retriever::MemoryResource res;
        bool ok = true;

        try
        {
          res = retriever.get(mesh->filename);
        }
        catch (resource_retriever::Exception &e)
        {
          // This macro replaced ROS_ERROR
          RCLCPP_ERROR(rclcpp::get_logger("self_mask"), "%s", e.what());
          ok = false;
        }

        if (ok)
        {
          if (res.size == 0)
          {
            RCLCPP_WARN(rclcpp::get_logger("self_mask"),
                        "Retrieved empty mesh for resource '%s'",
                        mesh->filename.c_str());
          }
          else
          {
            boost::filesystem::path model_path(mesh->filename);
            std::string ext = model_path.extension().string();
            if (ext == ".dae" || ext == ".DAE")
            {
              result = shapes::createMeshFromBinaryDAE(mesh->filename.c_str());
            }
            else
            {
              result = shapes::createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size);
            }
            if (!result)
              RCLCPP_ERROR(rclcpp::get_logger("self_mask"),
                           "Failed to load mesh '%s'",
                           mesh->filename.c_str());
          }
        }
      }
      else
        RCLCPP_WARN(rclcpp::get_logger("self_mask"), "Empty mesh filename");
      break;
    }
    default:
      RCLCPP_ERROR(rclcpp::get_logger("self_mask"), "Unknown geometry type: %d", (int)geom->type);
      break;
  }
  return result;
}

/** \brief SelfMask computes which points in a pointcloud are inside the robot’s links */
template <typename PointT>
class SelfMask
{
protected:

  struct SeeLink
  {
    SeeLink()
    {
      body = nullptr;
      unscaledBody = nullptr;
    }
    std::string      name;
    bodies::Body    *body;
    bodies::Body    *unscaledBody;
    tf2::Transform   constTransf;
    double           volume;
  };

  struct SortBodies
  {
    bool operator()(const SeeLink &b1, const SeeLink &b2)
    {
      return b1.volume > b2.volume;
    }
  };

public:
  using PointCloud = pcl::PointCloud<PointT>;

  /** \brief Construct the filter */
  SelfMask(rclcpp::Node::SharedPtr node,
           tf2_ros::Buffer &tf_buffer,
           const std::vector<LinkInfo> &links)
    : node_(node)
    , tf_buffer_(tf_buffer)
  {
    configure(links);
  }

  /** \brief Destructor to clean up */
  ~SelfMask()
  {
    freeMemory();
  }

  /** \brief Compute the containment mask (INSIDE or OUTSIDE) for a given pointcloud */
  void maskContainment(const PointCloud &data_in, std::vector<int> &mask)
  {
    mask.resize(data_in.points.size());
    if (bodies_.empty())
    {
      std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    }
    else
    {
      // Convert PCL header to ROS 2 header
      std_msgs::msg::Header header;
      pcl_conversions::fromPCL(data_in.header, header);

      assumeFrame(header);
      maskAuxContainment(data_in, mask);
    }
  }

  /** \brief Compute the intersection mask (INSIDE, OUTSIDE, SHADOW) for the given pointcloud */
  void maskIntersection(const PointCloud &data_in,
                        const std::string &sensor_frame,
                        const double min_sensor_dist,
                        std::vector<int> &mask,
                        const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr)
  {
    mask.resize(data_in.points.size());
    if (bodies_.empty())
    {
      std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    }
    else
    {
      std_msgs::msg::Header header;
      pcl_conversions::fromPCL(data_in.header, header);

      assumeFrame(header, sensor_frame, min_sensor_dist);
      if (sensor_frame.empty())
      {
        maskAuxContainment(data_in, mask);
      }
      else
      {
        maskAuxIntersection(data_in, mask, intersectionCallback);
      }
    }
  }

  /** \brief Overload to compute the intersection mask if sensor position is known directly */
  void maskIntersection(const PointCloud &data_in,
                        const tf2::Vector3 &sensor_pos,
                        const double min_sensor_dist,
                        std::vector<int> &mask,
                        const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr)
  {
    mask.resize(data_in.points.size());
    if (bodies_.empty())
    {
      std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    }
    else
    {
      std_msgs::msg::Header header;
      pcl_conversions::fromPCL(data_in.header, header);

      assumeFrame(header, sensor_pos, min_sensor_dist);
      maskAuxIntersection(data_in, mask, intersectionCallback);
    }
  }

  /** \brief Assume subsequent calls will be in the frame passed to this function */
  void assumeFrame(const std_msgs::msg::Header &header)
  {
    const rclcpp::Time transform_time(header.stamp.sec, header.stamp.nanosec, RCL_SYSTEM_TIME);
    const unsigned int bs = bodies_.size();

    // Place the links in the assumed frame
    for (unsigned int i = 0; i < bs; ++i)
    {
      geometry_msgs::msg::TransformStamped transform_stamped;
      // Attempt to see if we can transform
      if (!tf_buffer_.canTransform(header.frame_id,
                                   bodies_[i].name,
                                   transform_time,
                                   rclcpp::Duration(std::chrono::milliseconds(100))))
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "WaitForTransform timed out from %s to %s after 100ms",
                     bodies_[i].name.c_str(), header.frame_id.c_str());
      }

      try
      {
        // Convert the geometry_msgs transform to tf2::Transform
        transform_stamped = tf_buffer_.lookupTransform(header.frame_id,
                                                       bodies_[i].name,
                                                       transform_time,
                                                       rclcpp::Duration(std::chrono::milliseconds(100)));

        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);

        tf2::Vector3 t(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);

        tf2::Transform tf2_transform(q, t);

        // set pose for scaled/unscaled bodies
        bodies_[i].body->setPose(tf2_transform * bodies_[i].constTransf);
        bodies_[i].unscaledBody->setPose(tf2_transform * bodies_[i].constTransf);
      }
      catch(tf2::TransformException &ex)
      {
        // If we fail, set identity
        tf2::Transform identity;
        identity.setIdentity();
        bodies_[i].body->setPose(identity);
        bodies_[i].unscaledBody->setPose(identity);

        RCLCPP_ERROR(node_->get_logger(),
                     "Unable to lookup transform from %s to %s. Exception: %s",
                     bodies_[i].name.c_str(), header.frame_id.c_str(), ex.what());
      }
    }

    computeBoundingSpheres();
  }

  /** \brief Overload to specify the sensor position directly */
  void assumeFrame(const std_msgs::msg::Header &header,
                   const tf2::Vector3 &sensor_pos,
                   const double min_sensor_dist)
  {
    assumeFrame(header);
    sensor_pos_ = sensor_pos;
    min_sensor_dist_ = min_sensor_dist;
  }

  /** \brief Overload to specify sensor’s frame for transforms */
  void assumeFrame(const std_msgs::msg::Header &header,
                   const std::string &sensor_frame,
                   const double min_sensor_dist)
  {
    assumeFrame(header);

    rclcpp::Time transform_time(header.stamp.sec, header.stamp.nanosec, RCL_SYSTEM_TIME);

    if (!sensor_frame.empty())
    {
      // Attempt a transform from the sensor_frame to cloud frame
      if (!tf_buffer_.canTransform(header.frame_id,
                                   sensor_frame,
                                   transform_time,
                                   rclcpp::Duration(std::chrono::milliseconds(100))))
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "WaitForTransform timed out from %s to %s after 100ms",
                     sensor_frame.c_str(), header.frame_id.c_str());
        sensor_pos_ = tf2::Vector3(0, 0, 0);
      }

      try
      {
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_.lookupTransform(header.frame_id,
                                       sensor_frame,
                                       transform_time,
                                       rclcpp::Duration(std::chrono::milliseconds(100)));

        tf2::Vector3 t(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);

        sensor_pos_ = t;
      }
      catch(tf2::TransformException &ex)
      {
        sensor_pos_ = tf2::Vector3(0, 0, 0);
        RCLCPP_ERROR(node_->get_logger(),
                     "Unable to lookup transform from %s to %s. Exception: %s",
                     sensor_frame.c_str(), header.frame_id.c_str(), ex.what());
      }
    }
    else
    {
      // If sensor_frame is empty, we do not transform sensor_pos_ (0,0,0)
      sensor_pos_ = tf2::Vector3(0, 0, 0);
    }

    min_sensor_dist_ = min_sensor_dist;
  }

  /** \brief Get mask for a single point (INSIDE or OUTSIDE); must call assumeFrame() first */
  int getMaskContainment(const tf2::Vector3 &pt) const
  {
    const unsigned int bs = bodies_.size();
    int out = OUTSIDE;
    for (unsigned int j = 0; out == OUTSIDE && j < bs; ++j)
    {
      if (bodies_[j].body->containsPoint(pt))
        out = INSIDE;
    }
    return out;
  }

  /** \brief Overload: x,y,z version */
  int getMaskContainment(double x, double y, double z) const
  {
    return getMaskContainment(tf2::Vector3(x, y, z));
  }

  /** \brief Get intersection mask (INSIDE, OUTSIDE, SHADOW) for a single point; must call assumeFrame() first */
  int getMaskIntersection(double x, double y, double z,
                          const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr) const
  {
    return getMaskIntersection(tf2::Vector3(x, y, z), intersectionCallback);
  }

  /** \brief Overload for intersection mask */
  int getMaskIntersection(const tf2::Vector3 &pt,
                          const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr) const
  {
    const unsigned int bs = bodies_.size();
    int out = OUTSIDE;

    // First check if the point is inside any unscaled body
    for (unsigned int j = 0; out == OUTSIDE && j < bs; ++j)
    {
      if (bodies_[j].unscaledBody->containsPoint(pt))
        out = INSIDE;
    }

    if (out == OUTSIDE)
    {
      // Check if the point is "shadow" w.r.t. sensor
      tf2::Vector3 dir = sensor_pos_ - pt;
      double lng = dir.length();
      if (lng < min_sensor_dist_)
      {
        out = INSIDE;
      }
      else
      {
        dir /= lng;  // normalize

        // Check intersection with each body
        std::vector<tf2::Vector3> intersections;
        for (unsigned int j = 0; out == OUTSIDE && j < bs; ++j)
        {
          if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
          {
            // Dot product check
            tf2::Vector3 diff = sensor_pos_ - intersections[0];
            if (dir.dot(diff) >= 0.0)
            {
              if (intersectionCallback)
                intersectionCallback(intersections[0]);
              out = SHADOW;
            }
          }
        }
        // If not shadow, see if point is inside scaled body
        for (unsigned int j = 0; out == OUTSIDE && j < bs; ++j)
        {
          if (bodies_[j].body->containsPoint(pt))
            out = INSIDE;
        }
      }
    }
    return out;
  }

  /** \brief Get the link names used for self-filtering */
  void getLinkNames(std::vector<std::string> &frames) const
  {
    for (const auto &b : bodies_)
      frames.push_back(b.name);
  }

protected:

  /** \brief Free memory for bodies */
  void freeMemory()
  {
    for (auto &b : bodies_)
    {
      if (b.body)         delete b.body;
      if (b.unscaledBody) delete b.unscaledBody;
    }
    bodies_.clear();
  }

  /** \brief Configure the filter: load URDF, parse collision geometries, create shapes */
  bool configure(const std::vector<LinkInfo> &links)
  {
    freeMemory();
    sensor_pos_ = tf2::Vector3(0, 0, 0);

    // Retrieve the robot_description parameter
    std::string content;
    if (!node_->get_parameter("robot_description", content) || content.empty())
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Robot model not found! Did you remap 'robot_description'?");
      return false;
    }

    // Parse URDF
    auto urdfModel = std::make_shared<urdf::Model>();
    if (!urdfModel->initString(content))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Unable to parse URDF description!");
      return false;
    }

    std::stringstream missing;

    // Build bodies for requested links
    for (auto &linfo : links)
    {
      const urdf::Link *link = urdfModel->getLink(linfo.name).get();
      if (!link)
      {
        missing << " " << linfo.name;
        continue;
      }

      if (!(link->collision && link->collision->geometry))
      {
        RCLCPP_WARN(node_->get_logger(),
                    "No collision geometry specified for link '%s'",
                    linfo.name.c_str());
        continue;
      }

      // If a link has multiple collision elements in URDF
      std::vector<CollisionSharedPtr> collisions = link->collision_array;

      // If collision_array is empty, fallback to single collision
      if (collisions.empty() && link->collision)
      {
        collisions.push_back(link->collision);
      }

      // Construct a body for each collision geometry
      for (unsigned int j = 0; j < collisions.size(); ++j)
      {
        shapes::Shape *shape = constructShape(collisions[j]->geometry.get());
        if (!shape)
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "Unable to construct collision shape #%u for link '%s'",
                       (j + 1), linfo.name.c_str());
          continue;
        }

        SeeLink sl;
        sl.body = bodies::createBodyFromShape(shape);
        if (sl.body)
        {
          sl.name = linfo.name;
          // collision offset
          sl.constTransf = urdfPose2TFTransform(collisions[j]->origin);
          sl.body->setScale(linfo.scale);
          sl.body->setPadding(linfo.padding);

          RCLCPP_INFO_STREAM(node_->get_logger(),
                             "Self see link name " << linfo.name
                             << " collision #" << (j+1)
                             << " with padding " << linfo.padding);

          sl.volume = sl.body->computeVolume();
          sl.unscaledBody = bodies::createBodyFromShape(shape);
          bodies_.push_back(sl);
        }
        else
        {
          RCLCPP_WARN(node_->get_logger(),
                      "Unable to create body for link '%s' collision geometry #%u",
                      linfo.name.c_str(), (j + 1));
        }
        delete shape;
      }
    }

    if (!missing.str().empty())
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Some links were included for self mask but they do not exist in the URDF:%s",
                  missing.str().c_str());
    }

    if (bodies_.empty())
    {
      RCLCPP_WARN(node_->get_logger(),
                  "No robot links will be checked for self mask");
    }

    // Sort bodies by volume descending
    std::sort(bodies_.begin(), bodies_.end(), SortBodies());

    bspheres_.resize(bodies_.size());
    bspheresRadius2_.resize(bodies_.size());

    for (auto &b : bodies_)
    {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Self mask includes link %s with volume %f",
                   b.name.c_str(), b.volume);
    }

    return true;
  }

  /** \brief Compute bounding spheres for each link’s collision body */
  void computeBoundingSpheres()
  {
    const unsigned int bs = bodies_.size();
    for (unsigned int i = 0; i < bs; ++i)
    {
      bodies_[i].body->computeBoundingSphere(bspheres_[i]);
      bspheresRadius2_[i] = bspheres_[i].radius * bspheres_[i].radius;
    }
  }

  /** \brief Helper for simple inside/outside mask */
  void maskAuxContainment(const PointCloud &data_in, std::vector<int> &mask)
  {
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.points.size();

    // Merge all bounding spheres
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    double radiusSquared = bound.radius * bound.radius;

    for (size_t i = 0; i < np; ++i)
    {
      tf2::Vector3 pt(data_in.points[i].x,
                      data_in.points[i].y,
                      data_in.points[i].z);
      int out = OUTSIDE;
      // First check if inside the overall bounding sphere
      double dist2 = (pt - bound.center).length2();
      if (dist2 < radiusSquared)
      {
        // Then check actual link bodies
        for (unsigned int j = 0; out == OUTSIDE && j < bs; ++j)
        {
          if (bodies_[j].body->containsPoint(pt))
            out = INSIDE;
        }
      }
      mask[i] = out;
    }
  }

  /** \brief Helper for inside/outside/shadow mask */
  void maskAuxIntersection(const PointCloud &data_in,
                           std::vector<int> &mask,
                           const std::function<void(const tf2::Vector3&)> &callback)
  {
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.points.size();

    // Merge all bounding spheres
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    double radiusSquared = bound.radius * bound.radius;

    for (size_t i = 0; i < np; ++i)
    {
      tf2::Vector3 pt(data_in.points[i].x,
                      data_in.points[i].y,
                      data_in.points[i].z);
      int out = OUTSIDE;

      // Check unscaled body first
      double dist2 = (pt - bound.center).length2();
      if (dist2 < radiusSquared)
      {
        for (unsigned int j = 0; out == OUTSIDE && j < bs; ++j)
        {
          if (bodies_[j].unscaledBody->containsPoint(pt))
            out = INSIDE;
        }
      }

      // If not inside unscaled body
      if (out == OUTSIDE)
      {
        // Shadow check
        tf2::Vector3 dir = sensor_pos_ - pt;
        double lng = dir.length();
        if (lng < min_sensor_dist_)
        {
          out = INSIDE;
        }
        else
        {
          dir /= lng; // normalize
          std::vector<tf2::Vector3> intersections;
          for (unsigned int j = 0; out == OUTSIDE && j < bs; ++j)
          {
            if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
            {
              // Dot product to check if intersection is between pt and sensor
              tf2::Vector3 diff = sensor_pos_ - intersections[0];
              if (dir.dot(diff) >= 0.0)
              {
                if (callback)
                  callback(intersections[0]);
                out = SHADOW;
              }
            }
          }
          // If still OUTSIDE, check if inside scaled bodies
          if (out == OUTSIDE && dist2 < radiusSquared)
          {
            for (unsigned int j = 0; out == OUTSIDE && j < bs; ++j)
            {
              if (bodies_[j].body->containsPoint(pt))
                out = INSIDE;
            }
          }
        }
      }
      mask[i] = out;
    }
  }

  // ROS 2 node and TF buffer
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer &tf_buffer_;

  tf2::Vector3 sensor_pos_;
  double       min_sensor_dist_{0.0};

  // Bodies and bounding spheres
  std::vector<SeeLink>                bodies_;
  std::vector<double>                 bspheresRadius2_;
  std::vector<bodies::BoundingSphere> bspheres_;
};

}  // namespace robot_self_filter

#endif  // ROBOT_SELF_FILTER_SELF_MASK_
