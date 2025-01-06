#include "PickVertex.hh"

#include <iostream>

#include "MyGL/Utils.hh"

PickVertex::PickVertex() {
  auto& shader_manager = MyGL::ShaderManager::get_instance();
  shader_manager.add_shader(
      "pick_vertex",
      {{GL_VERTEX_SHADER,
        MyGL::read_file_to_string("data/shaders/pick_vertex.vert")},
       {GL_FRAGMENT_SHADER,
        MyGL::read_file_to_string("data/shaders/pick_vertex.frag")}});
  shader_manager.add_shader(
      "round_point",
      {{GL_VERTEX_SHADER, MyGL::read_file_to_string("data/shaders/basic.vert")},
       {GL_FRAGMENT_SHADER,
        MyGL::read_file_to_string("data/shaders/round_point.frag")}});
  shader_manager.add_shader(
      "basic",
      {{GL_VERTEX_SHADER, MyGL::read_file_to_string("data/shaders/basic.vert")},
       {GL_FRAGMENT_SHADER,
        MyGL::read_file_to_string("data/shaders/basic.frag")}});
}

int PickVertex::pick(const glm::ivec2& pos,
                     const MyGL::Mesh& mesh,
                     const std::tuple<glm::mat4, glm::mat4, glm::mat4>& mvp) {
  // If the mouse is outside the viewport, return -1
  auto [width, height] = MyGL::get_viewport_size();
  if (pos.x < 0 || pos.x >= width || pos.y < 0 || pos.y >= height) {
    vertex_id_ = -1;
    return -1;
  }

  GLboolean multisample_enabled;
  glGetBooleanv(GL_MULTISAMPLE, &multisample_enabled);
  glDisable(GL_MULTISAMPLE);

  glClearColor(0.f, 0.f, 0.f, 0.f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Shader Manager
  auto& shader_manager = MyGL::ShaderManager::get_instance();

  // Update Z-buffer
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  shader_manager.use_shader("basic");
  mesh.draw();

  // Draw vertices
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glPointSize(pick_point_size_);
  shader_manager.use_shader("pick_vertex");
  mesh.draw(MyGL::Mesh::DrawMode::POINTS);

  // Read pixel
  GLubyte pixel[PIXEL_COMPONENTS];
  glReadPixels(pos.x, pos.y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);

  int index =
      static_cast<int>(pixel[0] + (pixel[1] << 8) + (pixel[2] << 16)) - 1;
  vertex_id_ = index;

  if (multisample_enabled)
    glEnable(GL_MULTISAMPLE);

  return index;
}

void PickVertex::highlight_hovered_vertex(
    const MyGL::Mesh& mesh,
    const std::tuple<glm::mat4, glm::mat4, glm::mat4>& mvp) {
  if (vertex_id_ == -1)
    return;

  highlighted_vertex_.update({mesh.get_vertex_position(vertex_id_)});

  auto& shader_manager = MyGL::ShaderManager::get_instance();
  shader_manager.use_shader("round_point");
  shader_manager.set_uniform("round_point", "color", highlight_color_);
  highlighted_vertex_.draw();
}
