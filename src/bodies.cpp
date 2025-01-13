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

#include "robot_self_filter/bodies.h"
#include <LinearMath/btConvexHull.h>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>

namespace robot_self_filter
{
namespace bodies
{

// Create a Body* from a shapes::Shape
Body* createBodyFromShape(const shapes::Shape *shape)
{
  Body *body = nullptr;

  if (shape)
  {
    switch (shape->type)
    {
      case shapes::BOX:
        body = new Box(shape);
        break;
      case shapes::SPHERE:
        body = new Sphere(shape);
        break;
      case shapes::CYLINDER:
        body = new Cylinder(shape);
        break;
      case shapes::MESH:
        body = new ConvexMesh(shape);
        break;
      default:
        std::cerr << "[createBodyFromShape] Unknown shape type: " << shape->type << std::endl;
        break;
    }
  }

  return body;
}

// Merge bounding spheres into a single bounding sphere
void mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere)
{
  if (spheres.empty())
  {
    mergedSphere.center.setValue(0.0, 0.0, 0.0);
    mergedSphere.radius = 0.0;
  }
  else
  {
    mergedSphere = spheres[0];
    for (size_t i = 1; i < spheres.size(); ++i)
    {
      if (spheres[i].radius <= 0.0)
        continue;
      double d = spheres[i].center.distance(mergedSphere.center);

      // If mergedSphere is fully inside spheres[i]
      if (d + mergedSphere.radius <= spheres[i].radius)
      {
        mergedSphere.center = spheres[i].center;
        mergedSphere.radius = spheres[i].radius;
      }
      else if (d + spheres[i].radius > mergedSphere.radius)
      {
        // Spheres partially overlap or are separate
        tf2::Vector3 delta = mergedSphere.center - spheres[i].center;
        double new_radius = (delta.length() + spheres[i].radius + mergedSphere.radius) / 2.0;
        mergedSphere.center = delta.normalized() * (new_radius - spheres[i].radius) + spheres[i].center;
        mergedSphere.radius = new_radius;
      }
    }
  }
}

static const double ZERO = 1e-9;

/** \brief Compute the square of the distance between a ray and a point 
    Note: this requires 'dir' to be normalized */
static inline double distanceSQR(const tf2::Vector3& p, const tf2::Vector3& origin, const tf2::Vector3& dir)
{
  tf2::Vector3 a = p - origin;
  double d = dir.dot(a);
  return a.length2() - d * d;
}

namespace detail
{
  // Temporary structure for intersection points (used for ordering them by parameter "time" along the ray)
  struct intersc
  {
    intersc(const tf2::Vector3 &_pt, double _tm) : pt(_pt), time(_tm) {}
    tf2::Vector3 pt;
    double        time;
  };

  // Define an ordering on intersection points
  struct interscOrder
  {
    bool operator()(const intersc &a, const intersc &b) const
    {
      return a.time < b.time;
    }
  };
}  // namespace detail

/* ------------------------------------
 *             Sphere
 * ------------------------------------ */
bool Sphere::containsPoint(const tf2::Vector3 &p, bool /*verbose*/) const 
{
  return (m_center - p).length2() < m_radius2;
}

void Sphere::useDimensions(const shapes::Shape *shape)
{
  m_radius = static_cast<const shapes::Sphere*>(shape)->radius;
}

void Sphere::updateInternalData(void)
{
  m_radiusU = m_radius * m_scale + m_padding;
  m_radius2 = m_radiusU * m_radiusU;
  m_center = m_pose.getOrigin();
}

double Sphere::computeVolume(void) const
{
  // Volume of a sphere = (4/3) * pi * r^3
  return 4.0 * M_PI * m_radiusU * m_radiusU * m_radiusU / 3.0;
}

void Sphere::computeBoundingSphere(BoundingSphere &sphere) const
{
  sphere.center = m_center;
  sphere.radius = m_radiusU;
}

bool Sphere::intersectsRay(const tf2::Vector3& origin,
                           const tf2::Vector3& dir,
                           std::vector<tf2::Vector3> *intersections,
                           unsigned int count) const
{
  // Quick rejection if the ray is too far from the sphere's center
  if (distanceSQR(m_center, origin, dir) > m_radius2)
    return false;

  bool result = false;
  tf2::Vector3 cp = origin - m_center;
  double dpcpv = cp.dot(dir);

  // Vector from center to projected point on ray
  tf2::Vector3 w = cp - dpcpv * dir;
  tf2::Vector3 Q = m_center + w;
  double x = m_radius2 - w.length2();

  if (std::fabs(x) < ZERO)
  {
    // One intersection (tangent)
    w = Q - origin;
    double dpQv = w.dot(dir);
    if (dpQv > ZERO)
    {
      if (intersections)
        intersections->push_back(Q);
      result = true;
    }
  }
  else if (x > 0.0)
  {
    // Two intersections
    x = std::sqrt(x);
    w = dir * x;
    tf2::Vector3 A = Q - w;  // first intersection
    tf2::Vector3 B = Q + w;  // second intersection
    tf2::Vector3 wA = A - origin;
    tf2::Vector3 wB = B - origin;
    double dpAv = wA.dot(dir);
    double dpBv = wB.dot(dir);

    if (dpAv > ZERO)
    {
      result = true;
      if (intersections)
      {
        intersections->push_back(A);
        if (count == 1)
          return result;
      }
    }
    if (dpBv > ZERO)
    {
      result = true;
      if (intersections)
        intersections->push_back(B);
    }
  }
  return result;
}

/* ------------------------------------
 *             Cylinder
 * ------------------------------------ */
bool Cylinder::containsPoint(const tf2::Vector3 &p, bool /*verbose*/) const 
{
  tf2::Vector3 v = p - m_center;
  double pH = v.dot(m_normalH);

  if (std::fabs(pH) > m_length2)
    return false;

  double pB1 = v.dot(m_normalB1);
  double remaining = m_radius2 - pB1 * pB1;
  if (remaining < 0.0)
    return false;
  else
  {
    double pB2 = v.dot(m_normalB2);
    return (pB2 * pB2 < remaining);
  }
}

void Cylinder::useDimensions(const shapes::Shape *shape)
{
  // shape->length, shape->radius
  m_length = static_cast<const shapes::Cylinder*>(shape)->length;
  m_radius = static_cast<const shapes::Cylinder*>(shape)->radius;
}

void Cylinder::updateInternalData(void)
{
  m_radiusU = m_radius * m_scale + m_padding;
  m_radius2 = m_radiusU * m_radiusU;
  m_length2 = m_scale * m_length / 2.0 + m_padding;
  m_center = m_pose.getOrigin();
  m_radiusBSqr = m_length2 * m_length2 + m_radius2;
  m_radiusB = std::sqrt(m_radiusBSqr);

  // Extract orientation
  tf2::Matrix3x3 basis(m_pose.getBasis());
  m_normalB1 = basis.getColumn(0);
  m_normalB2 = basis.getColumn(1);
  m_normalH  = basis.getColumn(2);

  // For intersection with planes
  double tmp = -m_normalH.dot(m_center);
  m_d1 = tmp + m_length2;
  m_d2 = tmp - m_length2;
}

double Cylinder::computeVolume(void) const
{
  // Volume = π * r^2 * h
  return M_PI * m_radius2 * (m_length2 * 2.0);
}

void Cylinder::computeBoundingSphere(BoundingSphere &sphere) const
{
  sphere.center = m_center;
  sphere.radius = m_radiusB;
}

bool Cylinder::intersectsRay(const tf2::Vector3& origin,
                             const tf2::Vector3& dir,
                             std::vector<tf2::Vector3> *intersections,
                             unsigned int count) const
{
  if (distanceSQR(m_center, origin, dir) > m_radiusBSqr)
    return false;

  std::vector<detail::intersc> ipts;
  // Intersect with bases
  double tmp = m_normalH.dot(dir);
  if (std::fabs(tmp) > ZERO)
  {
    double tmp2 = -m_normalH.dot(origin);
    double t1 = (tmp2 - m_d1) / tmp;
    if (t1 > 0.0)
    {
      tf2::Vector3 p1 = origin + dir * t1;
      tf2::Vector3 v1 = p1 - m_center;
      // Project out the height component
      v1 = v1 - m_normalH.dot(v1) * m_normalH;
      if (v1.length2() < (m_radius2 + ZERO))
      {
        if (!intersections)
          return true;
        ipts.emplace_back(p1, t1);
      }
    }

    double t2 = (tmp2 - m_d2) / tmp;
    if (t2 > 0.0)
    {
      tf2::Vector3 p2 = origin + dir * t2;
      tf2::Vector3 v2 = p2 - m_center;
      v2 = v2 - m_normalH.dot(v2) * m_normalH;
      if (v2.length2() < (m_radius2 + ZERO))
      {
        if (!intersections)
          return true;
        ipts.emplace_back(p2, t2);
      }
    }
  }

  // If we haven't yet got two intersections, check the infinite side
  if (ipts.size() < 2)
  {
    // Cylinder is defined by normalH
    tf2::Vector3 VD = m_normalH.cross(dir);
    tf2::Vector3 ROD = m_normalH.cross(origin - m_center);
    double a = VD.length2();
    double b = 2.0 * ROD.dot(VD);
    double c = ROD.length2() - m_radius2;
    double d = b * b - 4.0 * a * c;
    if ((d > 0.0) && (std::fabs(a) > ZERO))
    {
      d = std::sqrt(d);
      double e = 2.0 * a;
      double t1 = (-b + d) / e;
      double t2 = (-b - d) / e;

      if (t1 > 0.0)
      {
        tf2::Vector3 p1 = origin + dir * t1;
        tf2::Vector3 v1 = m_center - p1;
        if (std::fabs(m_normalH.dot(v1)) < (m_length2 + ZERO))
        {
          if (!intersections)
            return true;
          ipts.emplace_back(p1, t1);
        }
      }
      if (t2 > 0.0)
      {
        tf2::Vector3 p2 = origin + dir * t2;
        tf2::Vector3 v2 = m_center - p2;
        if (std::fabs(m_normalH.dot(v2)) < (m_length2 + ZERO))
        {
          if (!intersections)
            return true;
          ipts.emplace_back(p2, t2);
        }
      }
    }
  }

  if (ipts.empty())
    return false;

  std::sort(ipts.begin(), ipts.end(), detail::interscOrder());
  unsigned int n = (count > 0) ? std::min<unsigned int>(count, ipts.size()) : ipts.size();
  for (unsigned int i = 0; i < n; ++i)
    intersections->push_back(ipts[i].pt);

  return true;
}

/* ------------------------------------
 *             Box
 * ------------------------------------ */
bool Box::containsPoint(const tf2::Vector3 &p, bool /*verbose*/) const
{
  tf2::Vector3 v = p - m_center;

  double pL = v.dot(m_normalL);
  if (std::fabs(pL) > m_length2)
    return false;

  double pW = v.dot(m_normalW);
  if (std::fabs(pW) > m_width2)
    return false;

  double pH = v.dot(m_normalH);
  if (std::fabs(pH) > m_height2)
    return false;

  return true;
}

void Box::useDimensions(const shapes::Shape *shape)
{
  // shape->size[] = (length, width, height)
  const double *size = static_cast<const shapes::Box*>(shape)->size;
  m_length = size[0];
  m_width  = size[1];
  m_height = size[2];
}

void Box::updateInternalData(void)
{
  double s2 = m_scale / 2.0;
  m_length2 = m_length * s2 + m_padding;
  m_width2  = m_width  * s2 + m_padding;
  m_height2 = m_height * s2 + m_padding;

  m_center = m_pose.getOrigin();

  m_radius2 = m_length2 * m_length2 + m_width2 * m_width2 + m_height2 * m_height2;
  m_radiusB = std::sqrt(m_radius2);

  tf2::Matrix3x3 basis(m_pose.getBasis());
  m_normalL = basis.getColumn(0);
  m_normalW = basis.getColumn(1);
  m_normalH = basis.getColumn(2);

  // For AABB corners in pose frame
  tf2::Vector3 tmp = m_normalL * m_length2 + m_normalW * m_width2 + m_normalH * m_height2;
  m_corner1 = m_center - tmp;
  m_corner2 = m_center + tmp;
}

double Box::computeVolume(void) const
{
  // Box volume = length * width * height
  // But we stored half-extents in m_length2, m_width2, m_height2
  return 8.0 * m_length2 * m_width2 * m_height2;
}

void Box::computeBoundingSphere(BoundingSphere &sphere) const
{
  sphere.center = m_center;
  sphere.radius = m_radiusB;
}

bool Box::intersectsRay(const tf2::Vector3& origin,
                        const tf2::Vector3& dir,
                        std::vector<tf2::Vector3> *intersections,
                        unsigned int count) const
{
  if (distanceSQR(m_center, origin, dir) > m_radius2)
    return false;

  double t_near = -std::numeric_limits<double>::infinity();
  double t_far  =  std::numeric_limits<double>::infinity();

  // For each of the 3 basis directions
  for (int i = 0; i < 3; i++)
  {
    const tf2::Vector3 &vN = (i == 0 ? m_normalL : (i == 1 ? m_normalW : m_normalH));
    double dp = vN.dot(dir);

    if (std::fabs(dp) > ZERO)
    {
      double t1 = vN.dot(m_corner1 - origin) / dp;
      double t2 = vN.dot(m_corner2 - origin) / dp;
      if (t1 > t2)
        std::swap(t1, t2);

      if (t1 > t_near)
        t_near = t1;
      if (t2 < t_far)
        t_far = t2;
      if (t_near > t_far)
        return false;
      if (t_far < 0.0)
        return false;
    }
    else
    {
      // Ray is parallel to this face; check if origin is out of bounds
      if (i == 0)
      {
        // X direction is "L"
        double minY = std::min(m_corner1.y(), m_corner2.y());
        double maxY = std::max(m_corner1.y(), m_corner2.y());
        double minZ = std::min(m_corner1.z(), m_corner2.z());
        double maxZ = std::max(m_corner1.z(), m_corner2.z());
        if ((minY > origin.y() || maxY < origin.y()) &&
            (minZ > origin.z() || maxZ < origin.z()))
        {
          return false;
        }
      }
      else if (i == 1)
      {
        // Y direction is "W"
        double minX = std::min(m_corner1.x(), m_corner2.x());
        double maxX = std::max(m_corner1.x(), m_corner2.x());
        double minZ = std::min(m_corner1.z(), m_corner2.z());
        double maxZ = std::max(m_corner1.z(), m_corner2.z());
        if ((minX > origin.x() || maxX < origin.x()) &&
            (minZ > origin.z() || maxZ < origin.z()))
        {
          return false;
        }
      }
      else
      {
        // Z direction is "H"
        double minX = std::min(m_corner1.x(), m_corner2.x());
        double maxX = std::max(m_corner1.x(), m_corner2.x());
        double minY = std::min(m_corner1.y(), m_corner2.y());
        double maxY = std::max(m_corner1.y(), m_corner2.y());
        if ((minX > origin.x() || maxX < origin.x()) &&
            (minY > origin.y() || maxY < origin.y()))
        {
          return false;
        }
      }
    }
  }

  if (intersections)
  {
    if ((t_far - t_near) > ZERO)
    {
      intersections->push_back(origin + dir * t_near);
      if (count > 1)
        intersections->push_back(origin + dir * t_far);
    }
    else
    {
      // Tangential
      intersections->push_back(origin + dir * t_far);
    }
  }
  return true;
}

/* ------------------------------------
 *          ConvexMesh
 * ------------------------------------ */
bool ConvexMesh::containsPoint(const tf2::Vector3 &p, bool /*verbose*/) const
{
  // Quick bounding box check first
  if (m_boundingBox.containsPoint(p))
  {
    // Transform point into mesh coordinate frame
    tf2::Vector3 ip = m_iPose * p;
    // Scale about the mesh center
    ip = m_meshCenter + (ip - m_meshCenter) * m_scale;

    return isPointInsidePlanes(ip);
  }
  return false;
}

void ConvexMesh::useDimensions(const shapes::Shape *shape)
{
  const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);

  double maxX = -std::numeric_limits<double>::infinity();
  double maxY = -std::numeric_limits<double>::infinity();
  double maxZ = -std::numeric_limits<double>::infinity();
  double minX =  std::numeric_limits<double>::infinity();
  double minY =  std::numeric_limits<double>::infinity();
  double minZ =  std::numeric_limits<double>::infinity();

  // Find bounding box of the incoming mesh
  for (unsigned int i = 0; i < mesh->vertexCount; ++i)
  {
    double vx = mesh->vertices[3 * i];
    double vy = mesh->vertices[3 * i + 1];
    double vz = mesh->vertices[3 * i + 2];

    if (vx > maxX) maxX = vx;
    if (vy > maxY) maxY = vy;
    if (vz > maxZ) maxZ = vz;

    if (vx < minX) minX = vx;
    if (vy < minY) minY = vy;
    if (vz < minZ) minZ = vz;
  }

  // Construct a box around the mesh
  shapes::Box *box_shape = new shapes::Box((maxX - minX), (maxY - minY), (maxZ - minZ));
  m_boundingBox.setDimensions(box_shape);
  delete box_shape;

  // Offset = center of that AABB in mesh frame
  m_boxOffset.setValue((minX + maxX) / 2.0,
                       (minY + maxY) / 2.0,
                       (minZ + maxZ) / 2.0);

  m_planes.clear();
  m_triangles.clear();
  m_vertices.clear();
  m_meshRadiusB = 0.0;
  m_meshCenter.setValue(0, 0, 0);

  // Prepare bullet hull computation
  btVector3 *vertices = new btVector3[mesh->vertexCount];
  for (unsigned int i = 0; i < mesh->vertexCount; ++i)
  {
    vertices[i].setValue(mesh->vertices[3 * i],
                         mesh->vertices[3 * i + 1],
                         mesh->vertices[3 * i + 2]);
  }

  HullDesc hd(QF_TRIANGLES, mesh->vertexCount, vertices);
  HullResult hr;
  HullLibrary hl;
  if (hl.CreateConvexHull(hd, hr) == QE_OK)
  {
    // Collect hull vertices
    m_vertices.reserve(hr.m_OutputVertices.size());
    tf2::Vector3 sum(0, 0, 0);

    for (size_t j = 0; j < hr.m_OutputVertices.size(); ++j)
    {
      tf2::Vector3 v(hr.m_OutputVertices[j][0],
                     hr.m_OutputVertices[j][1],
                     hr.m_OutputVertices[j][2]);
      m_vertices.push_back(v);
      sum += v;
    }

    // Center of the hull in local coordinates
    m_meshCenter = sum / (double)(hr.m_OutputVertices.size());

    // Compute max radius from center
    for (auto & v : m_vertices)
    {
      double dist = v.distance2(m_meshCenter);
      if (dist > m_meshRadiusB)
        m_meshRadiusB = dist;
    }
    m_meshRadiusB = std::sqrt(m_meshRadiusB);

    // Collect hull triangles and plane equations
    m_triangles.reserve(hr.m_Indices.size());
    for (unsigned int j = 0; j < hr.mNumFaces; ++j)
    {
      // Indices come in triples
      unsigned int i0 = hr.m_Indices[j * 3];
      unsigned int i1 = hr.m_Indices[j * 3 + 1];
      unsigned int i2 = hr.m_Indices[j * 3 + 2];

      tf2::Vector3 p1(hr.m_OutputVertices[i0][0],
                      hr.m_OutputVertices[i0][1],
                      hr.m_OutputVertices[i0][2]);
      tf2::Vector3 p2(hr.m_OutputVertices[i1][0],
                      hr.m_OutputVertices[i1][1],
                      hr.m_OutputVertices[i1][2]);
      tf2::Vector3 p3(hr.m_OutputVertices[i2][0],
                      hr.m_OutputVertices[i2][1],
                      hr.m_OutputVertices[i2][2]);

      tf2::Vector3 edge1 = p2 - p1;
      tf2::Vector3 edge2 = p3 - p1;

      edge1.normalize();
      edge2.normalize();
      tf2::Vector3 planeNormal = edge1.cross(edge2);

      if (planeNormal.length2() > 1e-6)
      {
        planeNormal.normalize();
        // We'll store as (nx, ny, nz, d) with the final 'd' = - planeNormal.dot(p1)
        Eigen::Vector4d planeEquation;
        planeEquation << planeNormal.x(),
                         planeNormal.y(),
                         planeNormal.z(),
                        -planeNormal.dot(p1);

        // Attempt to pick the plane orientation that has fewer vertices behind it
        unsigned int behindPlane = countVerticesBehindPlane(planeEquation);
        if (behindPlane > 0)
        {
          // Try flipping normal
          Eigen::Vector4d planeEquation2 = -planeEquation; // flips [nx,ny,nz,d] to [-nx,-ny,-nz,-d]
          unsigned int behindPlane2 = countVerticesBehindPlane(planeEquation2);
          if (behindPlane2 < behindPlane)
          {
            planeEquation = planeEquation2;
            behindPlane   = behindPlane2;
          }
        }
        m_planes.push_back(planeEquation);
        m_triangles.push_back(i0);
        m_triangles.push_back(i1);
        m_triangles.push_back(i2);
      }
    }
  }
  else
  {
    std::cerr << "[ConvexMesh::useDimensions] Unable to compute convex hull!" << std::endl;
  }

  hl.ReleaseResult(hr);
  delete[] vertices;
}

void ConvexMesh::updateInternalData(void)
{
  // Adjust the bounding box pose
  tf2::Transform pose = m_pose;
  pose.setOrigin(m_pose * m_boxOffset);
  m_boundingBox.setPose(pose);
  m_boundingBox.setPadding(m_padding);
  m_boundingBox.setScale(m_scale);

  // For quick center-based checks
  m_iPose = m_pose.inverse();
  m_center = m_pose * m_meshCenter;
  m_radiusB = m_meshRadiusB * m_scale + m_padding;
  m_radiusBSqr = m_radiusB * m_radiusB;

  // Scale the local vertices about m_meshCenter
  m_scaledVertices.resize(m_vertices.size());
  for (size_t i = 0; i < m_vertices.size(); ++i)
  {
    tf2::Vector3 v = m_vertices[i] - m_meshCenter;
    double l = v.length();
    // Scale + padding. If l is zero, avoid dividing by zero
    double factor = (l > ZERO) ? (m_scale + m_padding / l) : m_scale;
    m_scaledVertices[i] = m_meshCenter + v * factor;
  }
}

void ConvexMesh::computeBoundingSphere(BoundingSphere &sphere) const
{
  sphere.center = m_center;
  sphere.radius = m_radiusB;
}

bool ConvexMesh::isPointInsidePlanes(const tf2::Vector3& point) const
{
  // Check plane equations stored in m_planes
  for (const auto &plane : m_planes)
  {
    // plane(0..3) => [nx, ny, nz, d]
    // Dist = n·p + d, then subtract padding and small epsilon
    double dist = plane(0) * point.x() +
                  plane(1) * point.y() +
                  plane(2) * point.z() +
                  plane(3) -
                  m_padding - 1e-6;

    if (dist > 0.0)
      return false;
  }
  return true;
}

unsigned int ConvexMesh::countVerticesBehindPlane(const Eigen::Vector4d &planeNormal) const
{
  unsigned int result = 0;
  for (const auto &v : m_vertices)
  {
    double dist = planeNormal(0) * v.x() +
                  planeNormal(1) * v.y() +
                  planeNormal(2) * v.z() +
                  planeNormal(3) - 1e-6;
    if (dist > 0.0)
      result++;
  }
  return result;
}

double ConvexMesh::computeVolume(void) const
{
  // Signed volume approach (sum over tetrahedrons)
  double volume = 0.0;
  for (size_t i = 0; i < m_triangles.size() / 3; ++i)
  {
    const tf2::Vector3 &v1 = m_vertices[m_triangles[3*i + 0]];
    const tf2::Vector3 &v2 = m_vertices[m_triangles[3*i + 1]];
    const tf2::Vector3 &v3 = m_vertices[m_triangles[3*i + 2]];

    volume += (v1.x() * v2.y() * v3.z() +
               v2.x() * v3.y() * v1.z() +
               v3.x() * v1.y() * v2.z() -
               v1.x() * v3.y() * v2.z() -
               v2.x() * v1.y() * v3.z() -
               v3.x() * v2.y() * v1.z());
  }
  return std::fabs(volume) / 6.0;
}

bool ConvexMesh::intersectsRay(const tf2::Vector3& origin,
                               const tf2::Vector3& dir,
                               std::vector<tf2::Vector3> *intersections,
                               unsigned int count) const
{
  if (distanceSQR(m_center, origin, dir) > m_radiusBSqr)
    return false;
  if (!m_boundingBox.intersectsRay(origin, dir))
    return false;

  tf2::Vector3 orig = m_iPose * origin;
  tf2::Vector3 dr   = tf2::Matrix3x3(m_iPose.getBasis()) * dir;

  std::vector<detail::intersc> ipts;
  bool result = false;

  unsigned int nt = m_triangles.size() / 3;
  for (unsigned int i = 0; i < nt; ++i)
  {
    const Eigen::Vector4d &plane = m_planes[i]; // matched 1:1 with each face
    double denom = plane(0) * dr.x() + plane(1) * dr.y() + plane(2) * dr.z();

    if (std::fabs(denom) > 1e-9)
    {
      double numer = -(plane(0)*orig.x() +
                       plane(1)*orig.y() +
                       plane(2)*orig.z() +
                       plane(3));
      double t = numer / denom;
      if (t > 0.0)
      {
        int v1 = m_triangles[3*i + 0];
        int v2 = m_triangles[3*i + 1];
        int v3 = m_triangles[3*i + 2];
		
        const tf2::Vector3 &a = m_scaledVertices[v1];
        const tf2::Vector3 &b = m_scaledVertices[v2];
        const tf2::Vector3 &c = m_scaledVertices[v3];

        // Intersection point in mesh frame
        tf2::Vector3 P = orig + dr * t;

        // Barycentric or inside-triangle test
        tf2::Vector3 cb = c - b;
        tf2::Vector3 ab = a - b;
        tf2::Vector3 pb = P - b;

        tf2::Vector3 c1 = cb.cross(pb);
        tf2::Vector3 c2 = cb.cross(ab);
        if (c1.dot(c2) < 0.0)
          continue;

        tf2::Vector3 ca = c - a;
        tf2::Vector3 pa = P - a;
        tf2::Vector3 ba = -ab;

        c1 = ca.cross(pa);
        c2 = ca.cross(ba);
        if (c1.dot(c2) < 0.0)
          continue;

        c1 = ba.cross(pa);
        c2 = ba.cross(ca);
        if (c1.dot(c2) < 0.0)
          continue;

        // Passed all tests
        result = true;
        if (intersections)
        {
          // Transform intersection back to world frame
          ipts.emplace_back(origin + dir * t, t);
        }
        else
        {
          // If we don't need the actual points, we can stop
          break;
        }
      }
    }
  }

  if (intersections)
  {
    std::sort(ipts.begin(), ipts.end(), detail::interscOrder());
    unsigned int n = (count > 0) ? std::min<unsigned int>(count, ipts.size()) : ipts.size();
    for (unsigned int i = 0; i < n; ++i)
      intersections->push_back(ipts[i].pt);
  }

  return result;
}

}  // namespace bodies
}  // namespace robot_self_filter
