/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#ifndef GEOMETRIC_SHAPES_POINT_INCLUSION_
#define GEOMETRIC_SHAPES_POINT_INCLUSION_

#include "robot_self_filter/shapes.h"

// Replace tf includes with tf2 equivalents
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Core>              
// #include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
// #include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <vector>

/**
   This set of classes allows quickly detecting whether a given point
   is inside an object or not. Only basic (simple) types of objects
   are supported: spheres, cylinders, boxes. This capability is useful
   when removing points from inside the robot (when the robot sees its
   arms, for example).
*/

namespace robot_self_filter
{
namespace bodies
{
  /** \brief Definition of a sphere that bounds another object */
  struct BoundingSphere
  {
    tf2::Vector3 center;
    double       radius;
  };

  /** \brief A body is a shape + its pose. Point inclusion, ray
      intersection can be tested, volumes and bounding spheres can
      be computed.*/
  class Body
  {
  public:
    Body()
    {
      m_scale = 1.0;
      m_padding = 0.0;
      m_pose.setIdentity();
      m_type = shapes::UNKNOWN_SHAPE;
    }

    virtual ~Body() = default;

    /** \brief Get the type of shape this body represents */
    shapes::ShapeType getType(void) const
    {
      return m_type;
    }

    /** \brief If the dimension of the body should be scaled, this
        method sets the scale. Default is 1.0 */
    void setScale(double scale)
    {
      m_scale = scale;
      updateInternalData();
    }

    /** \brief Retrieve the current scale */
    double getScale(void) const
    {
      return m_scale;
    }

    /** \brief If constant padding should be added to the body, this
        method sets the padding. Default is 0.0 */
    void setPadding(double padd)
    {
      m_padding = padd;
      updateInternalData();
    }

    /** \brief Retrieve the current padding */
    double getPadding(void) const
    {
      return m_padding;
    }

    /** \brief Set the pose of the body. Default is identity */
    void setPose(const tf2::Transform &pose)
    {
      m_pose = pose;
      updateInternalData();
    }

    /** \brief Retrieve the pose of the body */
    const tf2::Transform& getPose(void) const
    {
      return m_pose;
    }

    /** \brief Set the dimensions of the body (from corresponding shape) */
    void setDimensions(const shapes::Shape *shape)
    {
      useDimensions(shape);
      updateInternalData();
    }

    /** \brief Check if a point is inside the body */
    bool containsPoint(double x, double y, double z) const
    {
      return containsPoint(tf2::Vector3(x, y, z));
    }

    /** \brief Check if a ray intersects the body, and find the
        set of intersections, in order, along the ray. A maximum
        number of intersections can be specified as well. If that
        number is 0, all intersections are returned */
    virtual bool intersectsRay(const tf2::Vector3& origin,
                               const tf2::Vector3& dir,
                               std::vector<tf2::Vector3>* intersections = nullptr,
                               unsigned int count = 0) const = 0;

    /** \brief Check if a point is inside the body */
    virtual bool containsPoint(const tf2::Vector3 &p, bool verbose = false) const = 0;

    /** \brief Compute the volume of the body. This method includes
        changes induced by scaling and padding */
    virtual double computeVolume(void) const = 0;

    /** \brief Compute the bounding radius for the body, in its current
        pose. Scaling and padding are accounted for. */
    virtual void computeBoundingSphere(BoundingSphere &sphere) const = 0;

  protected:
    virtual void updateInternalData(void) = 0;
    virtual void useDimensions(const shapes::Shape *shape) = 0;

    shapes::ShapeType m_type;
    tf2::Transform    m_pose;
    double            m_scale;
    double            m_padding;
  };

  /** \brief Definition of a sphere */
  class Sphere : public Body
  {
  public:
    Sphere() : Body()
    {
      m_type = shapes::SPHERE;
    }

    Sphere(const shapes::Shape *shape) : Body()
    {
      m_type = shapes::SPHERE;
      setDimensions(shape);
    }

    ~Sphere() override = default;

    bool containsPoint(const tf2::Vector3 &p, bool verbose=false) const override;
    double computeVolume(void) const override;
    void computeBoundingSphere(BoundingSphere &sphere) const override;
    bool intersectsRay(const tf2::Vector3& origin,
                       const tf2::Vector3 &dir,
                       std::vector<tf2::Vector3> *intersections = nullptr,
                       unsigned int count = 0) const override;

  protected:
    void useDimensions(const shapes::Shape *shape) override;
    void updateInternalData(void) override;

    tf2::Vector3 m_center;
    double       m_radius;
    double       m_radiusU;
    double       m_radius2;
  };

  /** \brief Definition of a cylinder */
  class Cylinder : public Body
  {
  public:
    Cylinder() : Body()
    {
      m_type = shapes::CYLINDER;
    }

    Cylinder(const shapes::Shape *shape) : Body()
    {
      m_type = shapes::CYLINDER;
      setDimensions(shape);
    }

    ~Cylinder() override = default;

    bool containsPoint(const tf2::Vector3 &p, bool verbose=false) const override;
    double computeVolume(void) const override;
    void computeBoundingSphere(BoundingSphere &sphere) const override;
    bool intersectsRay(const tf2::Vector3& origin,
                       const tf2::Vector3 &dir,
                       std::vector<tf2::Vector3> *intersections = nullptr,
                       unsigned int count = 0) const override;

  protected:
    void useDimensions(const shapes::Shape *shape) override;
    void updateInternalData(void) override;

    tf2::Vector3 m_center;
    tf2::Vector3 m_normalH;
    tf2::Vector3 m_normalB1;
    tf2::Vector3 m_normalB2;

    double       m_length;
    double       m_length2;
    double       m_radius;
    double       m_radiusU;
    double       m_radiusB;
    double       m_radiusBSqr;
    double       m_radius2;
    double       m_d1;
    double       m_d2;
  };

  /** \brief Definition of a box */
  class Box : public Body
  {
  public:
    Box() : Body()
    {
      m_type = shapes::BOX;
    }

    Box(const shapes::Shape *shape) : Body()
    {
      m_type = shapes::BOX;
      setDimensions(shape);
    }

    ~Box() override = default;

    bool containsPoint(const tf2::Vector3 &p, bool verbose=false) const override;
    double computeVolume(void) const override;
    void computeBoundingSphere(BoundingSphere &sphere) const override;
    bool intersectsRay(const tf2::Vector3& origin,
                       const tf2::Vector3 &dir,
                       std::vector<tf2::Vector3> *intersections = nullptr,
                       unsigned int count = 0) const override;

  protected:
    void useDimensions(const shapes::Shape *shape) override; // (x, y, z) = (length, width, height)
    void updateInternalData(void) override;

    tf2::Vector3 m_center;
    tf2::Vector3 m_normalL;
    tf2::Vector3 m_normalW;
    tf2::Vector3 m_normalH;

    tf2::Vector3 m_corner1;
    tf2::Vector3 m_corner2;

    double       m_length;
    double       m_width;
    double       m_height;
    double       m_length2;
    double       m_width2;
    double       m_height2;
    double       m_radiusB;
    double       m_radius2;
  };

  /*
  // Example of a mesh body (commented out in the original file)
  class Mesh : public Body
  {
  public:
      Mesh() : Body()
      {
          m_type = shapes::MESH;
          m_btMeshShape = NULL;
          m_btMesh = NULL;
      }

      Mesh(const shapes::Shape *shape) : Body()
      {
          m_type = shapes::MESH;
          m_btMeshShape = NULL;
          m_btMesh = NULL;
          setDimensions(shape);
      }

      virtual ~Mesh()
      {
          if (m_btMeshShape)
              delete m_btMeshShape;
          if (m_btMesh)
              delete m_btMesh;
      }

      // ...
  };
  */

  /** \brief Definition of a convex mesh. Convex hull is computed for a given shape::Mesh */
  class ConvexMesh : public Body
  {
  public:
    ConvexMesh() : Body()
    {
      m_type = shapes::MESH;
    }

    ConvexMesh(const shapes::Shape *shape) : Body()
    {
      m_type = shapes::MESH;
      setDimensions(shape);
    }

    ~ConvexMesh() override = default;

    bool containsPoint(const tf2::Vector3 &p, bool verbose=false) const override;
    double computeVolume(void) const override;
    void computeBoundingSphere(BoundingSphere &sphere) const override;
    bool intersectsRay(const tf2::Vector3& origin,
                       const tf2::Vector3 &dir,
                       std::vector<tf2::Vector3> *intersections = nullptr,
                       unsigned int count = 0) const override;

  protected:
    void useDimensions(const shapes::Shape *shape) override;
    void updateInternalData(void) override;

    // If you previously used tf::tfVector4, you may need to switch to either bullet's btVector4
    // or define a custom 4D vector structure. Below is an example if you have a tf2::Vector4:
    std::vector<Eigen::Vector4d> m_planes;  // Each plane stored as [nx, ny, nz, d]

    std::vector<tf2::Vector3>    m_vertices;
    std::vector<tf2::Vector3>    m_scaledVertices;
    std::vector<unsigned int>    m_triangles;
    tf2::Transform               m_iPose;

    tf2::Vector3                 m_center;
    tf2::Vector3                 m_meshCenter;
    double                       m_radiusB;
    double                       m_radiusBSqr;
    double                       m_meshRadiusB;

    tf2::Vector3                 m_boxOffset;
    Box                          m_boundingBox;

    // Example utility method adapted if you had tf::tfVector4:
    unsigned int countVerticesBehindPlane(const Eigen::Vector4d &planeNormal) const;
	bool isPointInsidePlanes(const tf2::Vector3 &point) const;
  };

  /** \brief Create a body from a given shape */
  Body* createBodyFromShape(const shapes::Shape *shape);

  /** \brief Compute a bounding sphere to enclose a set of bounding spheres */
  void mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres,
                            BoundingSphere &mergedSphere);

} // namespace bodies
} // namespace robot_self_filter

#endif  // GEOMETRIC_SHAPES_POINT_INCLUSION_
