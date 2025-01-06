#pragma once

#include "MyGL/Mesh.hh"
#include "MyGL/PointCloud.hh"
#include "MyGL/ShaderManager.hh"

class PickVertex {
 public:
  PickVertex();

  // Returns the index of the vertex that was picked,
  // or -1 if no vertex was picked.
  int pick(const glm::ivec2& pos,
           const MyGL::Mesh& mesh,
           const std::tuple<glm::mat4, glm::mat4, glm::mat4>& mvp);

  int get_picked_vertex() const { return vertex_id_; }

  void highlight_hovered_vertex(
      const MyGL::Mesh& mesh,
      const std::tuple<glm::mat4, glm::mat4, glm::mat4>& mvp);

  void set_pick_point_size(float size) { pick_point_size_ = size; }
  void set_highlight_color(const glm::vec4& color) { highlight_color_ = color; }

 private:
  int vertex_id_{-1};

  float pick_point_size_{15.0f};
  glm::vec4 highlight_color_{1.0f, 0.0f, 0.0f, 1.0f};

  MyGL::PointCloud highlighted_vertex_;

  static constexpr GLubyte PIXEL_COMPONENTS = 4;
};
