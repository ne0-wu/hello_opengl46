#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "Dijkstra.hh"
#include "Mesh.hh"
#include "MeshToGL.hh"

#include "MyGL/LogConsole.hh"
#include "MyGL/PickVertex.hh"
#include "MyGL/Shader.hh"
#include "MyGL/ShaderManager.hh"
#include "MyGL/Utils.hh"
#include "MyGL/Window.hh"

struct Flags {
  enum class InteractionMode {
    DEFAULT,
    SELECT_VERTEX
  } draw_mode = InteractionMode::DEFAULT;

  bool draw_wireframe = true;
  bool show_log_console = false;
} flags;

const char* InteractionModeItems[] = {"Default", "Select Vertex"};

class StatusBar {
 public:
  StatusBar(const std::string& text) : text(text) {}

  void set_text(const std::string& text) { this->text = text; }

  void draw() {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(viewport->Pos.x, viewport->Pos.y + viewport->Size.y - 30));
    ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, 30));
    ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDecoration;
    if (ImGui::Begin("##StatusBar", nullptr, window_flags)) {
      float textHeight = ImGui::GetTextLineHeight();
      ImGui::SetCursorPosY((30 - textHeight) * 0.5f);
      ImGui::Text("%s", text.c_str());
    }
    ImGui::End();
  }

 private:
  std::string text;
} status_bar("no message");

MyGL::LogConsole logger;

// ==================================================

class GeodesicPath {
 public:
  GeodesicPath(const Mesh& mesh,
               Mesh::VertexHandle source,
               Mesh::VertexHandle target)
      : mesh(mesh), source(source), target(target) {
    auto dijkstra = Dijkstra::compute(mesh, source, target);
    if (!dijkstra.has_path(target))
      throw std::runtime_error("No path found");

    path = dijkstra.get_path(target);
  }

  std::vector<Mesh::VertexHandle>& get_path() { return path; }

 private:
  const Mesh& mesh;
  Mesh::VertexHandle source, target;

  std::vector<Mesh::VertexHandle> path;
};

// ==================================================

// ImGUI IO for camera control
void update_camera(MyGL::Camera& camera, float delta_time) {
  auto io = ImGui::GetIO();
  if (io.WantCaptureKeyboard)
    return;
  if (ImGui::IsKeyDown(ImGuiKey_W))
    camera.on_keyboard(MyGL::Camera::KeyboardMoveDirection::FORWARD,
                       delta_time);
  if (ImGui::IsKeyDown(ImGuiKey_S))
    camera.on_keyboard(MyGL::Camera::KeyboardMoveDirection::BACKWARD,
                       delta_time);
  if (ImGui::IsKeyDown(ImGuiKey_A))
    camera.on_keyboard(MyGL::Camera::KeyboardMoveDirection::LEFT, delta_time);
  if (ImGui::IsKeyDown(ImGuiKey_D))
    camera.on_keyboard(MyGL::Camera::KeyboardMoveDirection::RIGHT, delta_time);
  if (ImGui::IsKeyDown(ImGuiKey_K))
    camera.on_keyboard(MyGL::Camera::KeyboardMoveDirection::UP, delta_time);
  if (ImGui::IsKeyDown(ImGuiKey_J))
    camera.on_keyboard(MyGL::Camera::KeyboardMoveDirection::DOWN, delta_time);

  if (io.WantCaptureMouse)
    return;

  auto [x, y] = ImGui::GetMousePos();
}

// ==================================================

int main() {
  // Initialize window (and OpenGL context)
  MyGL::Window window;

  auto& shader_manager = MyGL::ShaderManager::get_instance();
  shader_manager.add_shader(
      "phong",
      {{GL_VERTEX_SHADER, MyGL::read_file_to_string("data/shaders/phong.vert")},
       {GL_FRAGMENT_SHADER,
        MyGL::read_file_to_string("data/shaders/phong.frag")}});

  shader_manager.add_shader(
      "basic",
      {{GL_VERTEX_SHADER, MyGL::read_file_to_string("data/shaders/basic.vert")},
       {GL_FRAGMENT_SHADER,
        MyGL::read_file_to_string("data/shaders/basic.frag")}});
  shader_manager.add_shader(
      "round_point",
      {{GL_VERTEX_SHADER, MyGL::read_file_to_string("data/shaders/basic.vert")},
       {GL_FRAGMENT_SHADER,
        MyGL::read_file_to_string("data/shaders/round_point.frag")}});

  MyGL::PickVertex pick_vertex;

  // Load mesh from file
  Mesh mesh;
  if (!OpenMesh::IO::read_mesh(mesh, "data/models/stanford-bunny.obj"))
    throw std::runtime_error("Failed to read mesh from file");

  // Move mesh to [-1, 1]^3
  glm::mat4 model;  // for convenience, we represent translation of models in
                    // the model matrix
  Eigen::Vector3d min =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d max =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::min());

  for (const auto& v : mesh.vertices()) {
    const auto& point = mesh.point(v);
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }

  Eigen::Vector3d center = (min + max) / 2.0;
  const auto scale = 2.0 / (max - min).maxCoeff();
  model = glm::scale(glm::mat4(1.0f), glm::vec3(scale)) *
          glm::translate(glm::mat4(1.0f),
                         glm::vec3(-center[0], -center[1], -center[2]));

  // Compute normals (for Phong shading)
  mesh.request_face_normals();
  mesh.request_vertex_normals();
  mesh.update_normals();

  // Convert mesh to MyGL::Mesh
  MeshToGL mesh2gl;
  MyGL::Mesh gl_mesh(mesh2gl.vertices(mesh), mesh2gl.indices(mesh));

  // Set up camera
  // the camera looks at the origin and is positioned at (0, 0, -2) in the
  // beginning
  MyGL::OrbitCamera camera({0.0f, 0.0f, 0.0f});
  camera.set_position({0.0f, 0.0f, -2.0f});

  Mesh::VertexHandle source_vertex, target_vertex;

  enum class DijkstraPathState {
    IDLE,
    SELECT_SOURCE,
    SELECT_TARGET,
    COMPUTE_PATH,
    UPLOAD_PATH,
    DONE
  } dijkstra_path_state = DijkstraPathState::IDLE;

  // Path
  std::vector<Mesh::VertexHandle> path;
  MyGL::PointCloud path_points;

  // Render loop
  // ==================================================
  float last_frame_time = static_cast<float>(glfwGetTime());

  auto render_loop = [&]() -> std::optional<int> {
    float current_frame_time = static_cast<float>(glfwGetTime());
    float delta_time = current_frame_time - last_frame_time;
    last_frame_time = current_frame_time;

    update_camera(camera, delta_time);

    // write the model matrix of camera in the view matrix
    glm::mat4 view = camera.get_view_matrix() * camera.get_model_matrix();
    auto [width, height] = window.get_framebuffer_size();
    glm::mat4 projection =
        camera.get_projection_matrix(static_cast<float>(width) / height);

    shader_manager.for_each_shader([&](MyGL::ShaderProgram& shader) {
      shader.set_MVP({model, view, projection});
    });

    // Select vertex
    // ==================================================
    if (window.is_mouse_inside() && !ImGui::GetIO().WantCaptureMouse) {
      ImVec2 mouse_pos = ImGui::GetMousePos();
      auto [width, height] = MyGL::get_viewport_size();
      pick_vertex.pick({mouse_pos.x, height - mouse_pos.y}, gl_mesh,
                       {model, view, projection});
    }

    auto hovered_vertex = Mesh::VertexHandle(pick_vertex.get_picked_vertex());
    if (hovered_vertex.is_valid())
      status_bar.set_text("Hovered vertex: " +
                          std::to_string(hovered_vertex.idx()));
    else
      status_bar.set_text("No vertex hovered");

    if (ImGui::IsMouseClicked(0) && hovered_vertex.is_valid()) {
      switch (dijkstra_path_state) {
        case DijkstraPathState::SELECT_SOURCE:
          source_vertex = hovered_vertex;
          dijkstra_path_state = DijkstraPathState::SELECT_TARGET;
          break;
        case DijkstraPathState::SELECT_TARGET:
          target_vertex = hovered_vertex;
          dijkstra_path_state = DijkstraPathState::COMPUTE_PATH;
          break;
      }
    }

    // Compute geodesic path
    // ==================================================
    if (dijkstra_path_state == DijkstraPathState::COMPUTE_PATH) {
      try {
        GeodesicPath geodesic_path(mesh, source_vertex, target_vertex);
        path = geodesic_path.get_path();
      } catch (const std::exception& e) {
        logger.log("Failed to compute shortest path: %s", e.what());
      }
      dijkstra_path_state = DijkstraPathState::UPLOAD_PATH;
    }

    // Upload path to GPU
    // ==================================================
    if (dijkstra_path_state == DijkstraPathState::UPLOAD_PATH) {
      std::vector<glm::vec3> path_vertices;
      for (const auto& v : path)
        path_vertices.push_back(to_glm(mesh.point(v)));

      path_points.update(path_vertices);

      dijkstra_path_state = DijkstraPathState::DONE;
    }

    ImGui::Begin("Menu");

    ImGui::Checkbox("Draw wireframe", &flags.draw_wireframe);
    ImGui::Checkbox("Show log console", &flags.show_log_console);

    if (dijkstra_path_state == DijkstraPathState::IDLE)
      if (ImGui::Button("Find shortest path"))
        dijkstra_path_state = DijkstraPathState::SELECT_SOURCE;

    ImGui::End();

    status_bar.draw();

    if (flags.show_log_console)
      logger.draw();

    glEnable(GL_MULTISAMPLE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // draw the mesh
    if (flags.draw_wireframe) {
      shader_manager.set_uniform("basic", "color",
                                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
      shader_manager.use_shader("basic");
      gl_mesh.draw(MyGL::Mesh::DrawMode::WIREFRAME);
    }

    shader_manager.set_uniform("phong", "light_pos",
                               glm::vec3(2.2f, 1.0f, 2.0f));
    shader_manager.set_uniform("phong", "light_color",
                               glm::vec3(1.0f, 1.0f, 1.0f));
    shader_manager.set_uniform("phong", "view_pos", camera.get_position());
    shader_manager.set_uniform("phong", "color",
                               glm::vec4(1.0f, 0.5f, 0.2f, 1.0f));

    shader_manager.use_shader("phong");
    gl_mesh.draw();

    pick_vertex.highlight_hovered_vertex(gl_mesh, {model, view, projection});

    if (dijkstra_path_state == DijkstraPathState::DONE) {
      shader_manager.set_uniform("round_point", "color",
                                 glm::vec4(0.2f, 0.8f, 0.3f, 1.0f));
      shader_manager.use_shader("round_point");
      path_points.draw();
    }

    return std::nullopt;
  };

  window.main_loop(render_loop);
}