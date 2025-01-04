#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <Eigen/Dense>
#include <OpenMesh/Core/Geometry/EigenVectorT.hh>

#include <OpenMesh/Core/Utils/PropertyManager.hh>

struct MyTraits : public OpenMesh::DefaultTraits {
  typedef Eigen::Vector3d Point;
  typedef Eigen::Vector3d Normal;
  typedef Eigen::Vector2d TexCoord2D;
};

class Mesh : public OpenMesh::TriMesh_ArrayKernelT<MyTraits> {
 public:
  // Property manager
  template <typename T>
  using VProp = OpenMesh::VProp<T>;
  template <typename T>
  using FProp = OpenMesh::FProp<T>;
  template <typename T>
  using HProp = OpenMesh::HProp<T>;
  template <typename T>
  using EProp = OpenMesh::EProp<T>;
};