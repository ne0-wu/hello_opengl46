#pragma once

#include "Mesh.hh"

#include "MyGL/Mesh.hh"

inline glm::vec3 to_glm(const Eigen::Vector3d& v) {
  return {v[0], v[1], v[2]};
}

inline glm::vec2 to_glm(const Eigen::Vector2d& v) {
  return {v[0], v[1]};
}

class MeshToGL {
 public:
  static std::vector<MyGL::Vertex> vertices(const Mesh& mesh) {
    std::vector<MyGL::Vertex> vertices;
    vertices.reserve(mesh.n_vertices());

    const auto zero3d = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
    const auto zero2d = Eigen::Vector2d(0.0f, 0.0f);

    for (const auto& v : mesh.vertices()) {
      const Eigen::Vector3d& point = mesh.point(v);
      const Eigen::Vector3d& normal =
          mesh.has_vertex_normals() ? mesh.normal(v) : zero3d;
      const Eigen::Vector2d& tex_coord =
          mesh.has_vertex_texcoords2D() ? mesh.texcoord2D(v) : zero2d;
      vertices.push_back({to_glm(point), to_glm(normal), to_glm(tex_coord)});
    }
    return vertices;
  }

  static std::vector<unsigned int> indices(const Mesh& mesh) {
    std::vector<unsigned int> indices;
    indices.reserve(mesh.n_faces() * 3);

    for (const auto& f : mesh.faces())
      for (const auto& v : mesh.fv_range(f))
        indices.push_back(v.idx());
    return indices;
  }
};