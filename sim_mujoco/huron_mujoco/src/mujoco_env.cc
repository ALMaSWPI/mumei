#include <iostream>
#include "huron_mujoco/mujoco_env.h"
#include "huron_mujoco/motor.h"
#include "huron_mujoco/encoder.h"
#include "huron_mujoco/floating_base.h"

namespace huron {
namespace mujoco {

static GLFWwindow* window;
static mjvCamera cam;                      // abstract camera
static mjvOption opt;                      // visualization options
static mjvScene scn;                       // abstract scene
static mjrContext con;                     // custom GPU context

// moue interaction
static bool button_left = false;
static bool button_middle = false;
static bool button_right =  false;
static double lastx = 0;
static double lasty = 0;

static mjModel* m;                  // MuJoCo model
static mjData* d;                   // MuJoCo data

static std::function<void(void)> user_loop_func;  // user-defined


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) ==
    GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) ==
    GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) ==
    GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void dummy_loop() {}

MujocoEnvironment::MujocoEnvironment(std::function<void(void)> loop_func,
                                     std::function<void(void)> exit_func)
    : Environment(std::bind(dummy_loop), exit_func) {
  user_loop_func = loop_func;
}

/**
 * Must be called after MujocoEnvironment().
 */
void mjcontroller(const mjModel* m, mjData* d) {
  user_loop_func();
}


void MujocoEnvironment::Initialize(int argc, char* argv[]) {
  // check command-line arguments
  if (argc != 2) {
    std::printf(" USAGE:  basic modelfile\n");
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(argv[1]) > 4 &&
      !std::strcmp(argv[1] + std::strlen(argv[1]) - 4, ".mjb")) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);
  mjcb_control = mjcontroller;
}



void MujocoEnvironment::Configure(void* config) {
}

void MujocoEnvironment::Finalize() {
}

void MujocoEnvironment::LoopPrologue() {
  mjtNum simstart = d->time;
  while (d->time - simstart < 1.0/60.0) {
    mj_step(m, d);
  }

  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // update scene and render
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoEnvironment::LoopEpilogue() {
}

void MujocoEnvironment::Exit() {
  exit_func_();
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

std::shared_ptr<huron::Motor> MujocoEnvironment::CreateMotor(
  const std::string& name,
  double gear_ratio) {
  auto motor = mujoco::Motor::make_shared(
    name,
    mj_name2id(m, mjOBJ_ACTUATOR, name.c_str()),
    weak_from_this(),
    gear_ratio);
  return motor;
}

std::shared_ptr<huron::Motor> MujocoEnvironment::CreateMotor(
  int id,
  double gear_ratio) {
  auto motor = mujoco::Motor::make_shared(
    std::string(mj_id2name(m, mjOBJ_ACTUATOR, id)),
    id,
    weak_from_this(),
    gear_ratio);
  return motor;
}

std::shared_ptr<huron::Encoder>
MujocoEnvironment::CreateEncoder(
  const std::string& name,
  double gear_ratio) {
  auto enc = mujoco::Encoder::make_shared(
      name,
      mj_name2id(m, mjOBJ_JOINT, name.c_str()),
      weak_from_this(),
      gear_ratio);
  return enc;
}

std::shared_ptr<huron::StateProvider> MujocoEnvironment::CreateFloatingBase(
  const std::string& name) {
  int id = name.empty() ? 0 : mj_name2id(m, mjOBJ_JOINT, name.c_str());
  return FloatingBase::make_shared(name, id, weak_from_this());
}

Eigen::Vector2d
MujocoEnvironment::GetEncoderValues(int id) const {
  return Eigen::Vector2d(d->qpos[m->jnt_qposadr[id]],
                         d->qvel[m->jnt_dofadr[id]]);
}

Eigen::Vector<double, 13>
MujocoEnvironment::GetFloatingBaseStates(int id) const {
  Eigen::Vector<double, 13> ret;
  int i = 0;
  for (; i < 6; ++i) {
    ret(i) = d->qpos[id + i];
    ret(i + 7) = d->qvel[id + i];
  }
  ret(i) = d->qpos[id + i];
  return ret;
}

void MujocoEnvironment::SetMotorValue(int id, double u) {
  d->ctrl[id] = u;
}

void MujocoEnvironment::PrintStates() const {
  std::cout << "qpos: ";
  for (int i = 0; i < m->nq; ++i) {
    std::cout << d->qpos[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "qvel: ";
  for (int i = 0; i < m->nv; ++i) {
    std::cout << d->qvel[i] << " ";
  }
  std::cout << std::endl;
}

}  // namespace mujoco
}  // namespace huron
