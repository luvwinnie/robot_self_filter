// ============================ self_mask.h (Fixed for multi-dimensional scaling) ============================
#ifndef ROBOT_SELF_FILTER_SELF_MASK_
#define ROBOT_SELF_FILTER_SELF_MASK_

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <urdf/model.h>
#include <resource_retriever/retriever.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <robot_self_filter/bodies.h>

namespace robot_self_filter
{

enum
{
  INSIDE  = 0,
  OUTSIDE = 1,
  SHADOW  = 2,
};

// Extended LinkInfo to allow multi-dimensional scale/padding
struct LinkInfo
{
  std::string name;

  // Fallback single scale/padding
  double scale   = 1.0;
  double padding = 0.01;

  // For boxes
  std::vector<double> box_scale;    // [sx, sy, sz]
  std::vector<double> box_padding;  // [px, py, pz]

  // For cylinders
  std::vector<double> cylinder_scale;   // [rad_scale, vert_scale]
  std::vector<double> cylinder_padding; // [rad_pad,   vert_pad]

  // For mesh shapes - to support URDF mesh scaling
  std::vector<double> mesh_scale;    // [sx, sy, sz]
  std::vector<double> mesh_padding;  // [px, py, pz]
};

static inline tf2::Transform urdfPose2TFTransform(const urdf::Pose &pose)
{
  tf2::Quaternion q(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
  tf2::Vector3 t(pose.position.x, pose.position.y, pose.position.z);
  return tf2::Transform(q, t);
}

// Return both the shape and URDF mesh scaling information
struct ShapeWithScale
{
  shapes::Shape* shape = nullptr;
  tf2::Vector3 urdf_scale{1.0, 1.0, 1.0};  // URDF mesh scaling
};

static ShapeWithScale constructShapeWithScale(const urdf::Geometry *geom)
{
  ShapeWithScale result;
  if (!geom) return result;

  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
    {
      const auto *sphere = dynamic_cast<const urdf::Sphere*>(geom);
      result.shape = new shapes::Sphere(sphere->radius);
      break;
    }
    case urdf::Geometry::BOX:
    {
      const auto *box = dynamic_cast<const urdf::Box*>(geom);
      result.shape = new shapes::Box(box->dim.x, box->dim.y, box->dim.z);
      break;
    }
    case urdf::Geometry::CYLINDER:
    {
      const auto *cyl = dynamic_cast<const urdf::Cylinder*>(geom);
      result.shape = new shapes::Cylinder(cyl->radius, cyl->length);
      break;
    }
    case urdf::Geometry::MESH:
    {
      const auto *mesh = dynamic_cast<const urdf::Mesh*>(geom);
      if (mesh && !mesh->filename.empty())
      {
        // Extract URDF mesh scaling (like MoveIt does)
        result.urdf_scale.setValue(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        
        resource_retriever::Retriever retriever;
        resource_retriever::MemoryResource res;
        try
        {
          res = retriever.get(mesh->filename);
        }
        catch (...)
        {
          return result;
        }
        if (res.size > 0)
        {
          boost::filesystem::path model_path(mesh->filename);
          std::string ext = model_path.extension().string();
          if (ext == ".dae" || ext == ".DAE")
            result.shape = shapes::createMeshFromBinaryDAE(mesh->filename.c_str());
          else
            result.shape = shapes::createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size);
        }
      }
      break;
    }
    default:
      break;
  }
  return result;
}

// Legacy function for compatibility
static shapes::Shape* constructShape(const urdf::Geometry *geom)
{
  return constructShapeWithScale(geom).shape;
}

template <typename PointT>
class SelfMask
{
protected:
  struct SeeLink
  {
    SeeLink() : body(nullptr), unscaledBody(nullptr), volume(0.0) {}
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
      // Larger volumes first
      return b1.volume > b2.volume;
    }
  };

public:
  using PointCloud = pcl::PointCloud<PointT>;

  SelfMask(rclcpp::Node::SharedPtr node,
           tf2_ros::Buffer &tf_buffer,
           const std::vector<LinkInfo> &links)
    : node_(node)
    , tf_buffer_(tf_buffer)
  {
    configure(links);
  }

  ~SelfMask()
  {
    freeMemory();
  }

  void maskContainment(const PointCloud &data_in, std::vector<int> &mask)
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
      assumeFrame(header);
      maskAuxContainment(data_in, mask);
    }
  }

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
      return;
    }

    std_msgs::msg::Header header;
    pcl_conversions::fromPCL(data_in.header, header);

    assumeFrame(header, sensor_frame, min_sensor_dist);

    if (sensor_frame.empty())
      maskAuxContainment(data_in, mask);
    else
      maskAuxIntersection(data_in, mask, intersectionCallback);
  }

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
      return;
    }

    std_msgs::msg::Header header;
    pcl_conversions::fromPCL(data_in.header, header);

    assumeFrame(header, sensor_pos, min_sensor_dist);
    maskAuxIntersection(data_in, mask, intersectionCallback);
  }

  void assumeFrame(const std_msgs::msg::Header &header)
  {
    rclcpp::Time transform_time(header.stamp.sec, header.stamp.nanosec, RCL_SYSTEM_TIME);
    for (auto &sl : bodies_)
    {
      try
      {
        auto transform_stamped = tf_buffer_.lookupTransform(
          header.frame_id, sl.name,
          transform_time, rclcpp::Duration(std::chrono::milliseconds(100)));
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
        sl.body->setPose(tf2_transform * sl.constTransf);
        sl.unscaledBody->setPose(tf2_transform * sl.constTransf);
      }
      catch(...)
      {
        // keep old pose
      }
    }
    computeBoundingSpheres();
  }

  void assumeFrame(const std_msgs::msg::Header &header,
                   const tf2::Vector3 &sensor_pos,
                   const double min_sensor_dist)
  {
    assumeFrame(header);
    sensor_pos_       = sensor_pos;
    min_sensor_dist_  = min_sensor_dist;
  }

  void assumeFrame(const std_msgs::msg::Header &header,
                   const std::string &sensor_frame,
                   const double min_sensor_dist)
  {
    assumeFrame(header);
    rclcpp::Time transform_time(header.stamp.sec, header.stamp.nanosec, RCL_SYSTEM_TIME);

    if (!sensor_frame.empty())
    {
      try
      {
        auto transform_stamped = tf_buffer_.lookupTransform(
          header.frame_id, sensor_frame,
          transform_time, rclcpp::Duration(std::chrono::milliseconds(100)));
        tf2::Vector3 t(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);
        sensor_pos_ = t;
      }
      catch(...)
      {
        sensor_pos_ = tf2::Vector3(0,0,0);
      }
    }
    else
    {
      sensor_pos_ = tf2::Vector3(0,0,0);
    }
    min_sensor_dist_ = min_sensor_dist;
  }

  int getMaskContainment(const tf2::Vector3 &pt) const
  {
    for (auto &sl : bodies_)
    {
      if (sl.body->containsPoint(pt))
        return INSIDE;
    }
    return OUTSIDE;
  }

  int getMaskIntersection(const tf2::Vector3 &pt,
                          const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr) const
  {
    for (auto &sl : bodies_)
      if (sl.unscaledBody->containsPoint(pt))
        return INSIDE;

    tf2::Vector3 dir = sensor_pos_ - pt;
    double lng = dir.length();
    if (lng < min_sensor_dist_) return INSIDE;

    dir /= lng;
    for (auto &sl : bodies_)
    {
      std::vector<tf2::Vector3> hits;
      if (sl.body->intersectsRay(pt, dir, &hits, 1))
      {
        tf2::Vector3 diff = sensor_pos_ - hits[0];
        if (dir.dot(diff) >= 0.0)
        {
          if (intersectionCallback) intersectionCallback(hits[0]);
          return SHADOW;
        }
      }
    }

    for (auto &sl : bodies_)
    {
      if (sl.body->containsPoint(pt))
        return INSIDE;
    }
    return OUTSIDE;
  }

  void getLinkNames(std::vector<std::string> &frames) const
  {
    for (auto &sl : bodies_)
      frames.push_back(sl.name);
  }

  const std::vector<SeeLink>& getBodies() const
  {
    return bodies_;
  }

protected:
  void freeMemory()
  {
    for (auto &sl : bodies_)
    {
      if (sl.body)         delete sl.body;
      if (sl.unscaledBody) delete sl.unscaledBody;
    }
    bodies_.clear();
  }

  bool configure(const std::vector<LinkInfo> &links)
  {
    freeMemory();
    sensor_pos_ = tf2::Vector3(0, 0, 0);

    std::string content;
    if (!node_->get_parameter("robot_description", content) || content.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Robot model not found!");
      return false;
    }
    auto urdfModel = std::make_shared<urdf::Model>();
    if (!urdfModel->initString(content))
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to parse URDF!");
      return false;
    }

    for (auto &linfo : links)
    {
      const urdf::Link *link = urdfModel->getLink(linfo.name).get();
      if (!link || !(link->collision && link->collision->geometry))
        continue;

      // Collect collision geometry
      std::vector<urdf::CollisionSharedPtr> collisions = link->collision_array;
      if (collisions.empty() && link->collision)
        collisions.push_back(link->collision);

      for (auto &coll : collisions)
      {
        ShapeWithScale shape_info = constructShapeWithScale(coll->geometry.get());
        if (!shape_info.shape) continue;

        SeeLink sl;
        sl.name       = linfo.name;
        sl.constTransf = urdfPose2TFTransform(coll->origin);
        sl.body       = bodies::createBodyFromShape(shape_info.shape);

        if (sl.body)
        {
          // handle shape type
          switch (sl.body->getType())
          {
            case shapes::SPHERE:
            {
              auto sph = dynamic_cast<bodies::Sphere*>(sl.body);
              // single scale/padding only
              sph->setScale(linfo.scale);
              sph->setPadding(linfo.padding);
              break;
            }
            case shapes::BOX:
            {
              auto bx = dynamic_cast<bodies::Box*>(sl.body);
              if (linfo.box_scale.size() == 3 && linfo.box_padding.size() == 3)
              {
                bx->setScale(linfo.box_scale[0],
                             linfo.box_scale[1],
                             linfo.box_scale[2]);
                bx->setPadding(linfo.box_padding[0],
                               linfo.box_padding[1],
                               linfo.box_padding[2]);
              }
              else
              {
                // fallback
                bx->setScale(linfo.scale, linfo.scale, linfo.scale);
                bx->setPadding(linfo.padding, linfo.padding, linfo.padding);
              }
              break;
            }
            case shapes::CYLINDER:
            {
              auto cyl = dynamic_cast<bodies::Cylinder*>(sl.body);
              if (linfo.cylinder_scale.size() == 2 && linfo.cylinder_padding.size() == 2)
              {
                cyl->setScale(linfo.cylinder_scale[0],
                              linfo.cylinder_scale[1]);
                cyl->setPadding(linfo.cylinder_padding[0],
                                linfo.cylinder_padding[1]);
              }
              else
              {
                // fallback
                cyl->setScale(linfo.scale, linfo.scale);
                cyl->setPadding(linfo.padding, linfo.padding);
              }
              break;
            }
            case shapes::MESH:
            {
              auto mesh_body = dynamic_cast<bodies::ConvexMesh*>(sl.body);
              if (mesh_body)
              {
                // Apply URDF mesh scaling combined with user configuration scaling
                double final_scale_x, final_scale_y, final_scale_z;
                double final_pad_x, final_pad_y, final_pad_z;
                
                if (linfo.mesh_scale.size() == 3 && linfo.mesh_padding.size() == 3)
                {
                  // Use user-specified mesh scaling/padding
                  final_scale_x = shape_info.urdf_scale.x() * linfo.mesh_scale[0];
                  final_scale_y = shape_info.urdf_scale.y() * linfo.mesh_scale[1];
                  final_scale_z = shape_info.urdf_scale.z() * linfo.mesh_scale[2];
                  final_pad_x = linfo.mesh_padding[0];
                  final_pad_y = linfo.mesh_padding[1];
                  final_pad_z = linfo.mesh_padding[2];
                }
                else
                {
                  // Fallback: use URDF scaling + uniform user scaling/padding
                  final_scale_x = shape_info.urdf_scale.x() * linfo.scale;
                  final_scale_y = shape_info.urdf_scale.y() * linfo.scale;
                  final_scale_z = shape_info.urdf_scale.z() * linfo.scale;
                  final_pad_x = final_pad_y = final_pad_z = linfo.padding;
                }
                
                // SAFETY CHECK: Skip mesh shapes with very small scaling that cause issues
                if (final_scale_x < 0.01 || final_scale_y < 0.01 || final_scale_z < 0.01)
                {
                  RCLCPP_WARN(node_->get_logger(), 
                             "Skipping mesh collision shape for link '%s' due to problematic scaling [%.3f, %.3f, %.3f]. "
                             "Consider using primitive collision shapes instead.",
                             linfo.name.c_str(), final_scale_x, final_scale_y, final_scale_z);
                  delete sl.body;
                  sl.body = nullptr;
                  delete sl.unscaledBody;
                  sl.unscaledBody = nullptr;
                  break;  // Skip this collision shape
                }
                
                mesh_body->setScale(final_scale_x, final_scale_y, final_scale_z);
                mesh_body->setPadding(final_pad_x, final_pad_y, final_pad_z);
              }
              break;
            }
            default:
              break;
          }

          // Skip if collision shape was marked as problematic
          if (!sl.body)
          {
            delete shape_info.shape;
            continue;
          }

          // compute volume
          sl.volume        = sl.body->computeVolume();
          
          // Create unscaled body with proper URDF scaling applied
          sl.unscaledBody  = bodies::createBodyFromShape(shape_info.shape);
          if (sl.unscaledBody && sl.unscaledBody->getType() == shapes::MESH)
          {
            auto unscaled_mesh = dynamic_cast<bodies::ConvexMesh*>(sl.unscaledBody);
            if (unscaled_mesh)
            {
              // Apply URDF mesh scaling to unscaled body (but no user padding/scaling)
              unscaled_mesh->setScale(shape_info.urdf_scale.x(), 
                                    shape_info.urdf_scale.y(), 
                                    shape_info.urdf_scale.z());
              unscaled_mesh->setPadding(0.0, 0.0, 0.0);  // No padding for unscaled
            }
          }

          bodies_.push_back(sl);
        }
        delete shape_info.shape;
      }
    }

    // Sort descending by volume
    std::sort(bodies_.begin(), bodies_.end(), SortBodies());

    bspheres_.resize(bodies_.size());
    bspheresRadius2_.resize(bodies_.size());
    return true;
  }

  void computeBoundingSpheres()
  {
    for (size_t i = 0; i < bodies_.size(); ++i)
    {
      bodies_[i].body->computeBoundingSphere(bspheres_[i]);
      bspheresRadius2_[i] = bspheres_[i].radius * bspheres_[i].radius;
    }
  }

  void maskAuxContainment(const PointCloud &data_in, std::vector<int> &mask)
  {
    // Merge bounding spheres for a quick cull
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    double radiusSq = bound.radius * bound.radius;

    for (size_t i = 0; i < data_in.points.size(); ++i)
    {
      tf2::Vector3 pt(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
      int out = OUTSIDE;
      double dist2 = (pt - bound.center).length2();
      if (dist2 < radiusSq)
      {
        for (auto &sl : bodies_)
        {
          if (sl.body->containsPoint(pt))
          {
            out = INSIDE;
            break;
          }
        }
      }
      mask[i] = out;
    }
  }

  void maskAuxIntersection(const PointCloud &data_in,
                           std::vector<int> &mask,
                           const std::function<void(const tf2::Vector3&)> &callback)
  {
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    double radiusSq = bound.radius * bound.radius;

    for (size_t i = 0; i < data_in.points.size(); ++i)
    {
      tf2::Vector3 pt(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
      int out = OUTSIDE;
      double dist2 = (pt - bound.center).length2();

      // Quick check if inside the unscaled shape
      if (dist2 < radiusSq)
      {
        for (auto &sl : bodies_)
        {
          if (sl.unscaledBody->containsPoint(pt))
          {
            out = INSIDE;
            break;
          }
        }
      }

      if (out == OUTSIDE)
      {
        tf2::Vector3 dir = sensor_pos_ - pt;
        double lng = dir.length();
        if (lng < min_sensor_dist_)
        {
          out = INSIDE;
        }
        else
        {
          dir /= lng;
          // Ray intersect
          for (auto &sl : bodies_)
          {
            std::vector<tf2::Vector3> hits;
            if (sl.body->intersectsRay(pt, dir, &hits, 1))
            {
              tf2::Vector3 diff = sensor_pos_ - hits[0];
              if (dir.dot(diff) >= 0.0)
              {
                if (callback) callback(hits[0]);
                out = SHADOW;
                break;
              }
            }
          }
          if (out == OUTSIDE && dist2 < radiusSq)
          {
            for (auto &sl : bodies_)
            {
              if (sl.body->containsPoint(pt))
              {
                out = INSIDE;
                break;
              }
            }
          }
        }
      }
      mask[i] = out;
    }
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer        &tf_buffer_;

  tf2::Vector3             sensor_pos_{0, 0, 0};
  double                   min_sensor_dist_{0.01};
  std::vector<SeeLink>     bodies_;
  std::vector<bodies::BoundingSphere> bspheres_;
  std::vector<double>      bspheresRadius2_;
};

}  // namespace robot_self_filter

#endif  // ROBOT_SELF_FILTER_SELF_MASK_
