#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "start.h"
using namespace std;
// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
int startup_key_id = -1;
bool reset_requested = false;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

void reset_to_startup_key() {
  if (startup_key_id >= 0) {
    mj_resetDataKeyframe(m, d, startup_key_id);
  } else {
    mj_resetData(m, d);
  }
  mj_forward(m, d);
}

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act == GLFW_PRESS && (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_BACKSPACE)) {
    reset_requested = true;
  }
}


void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  glfwGetCursorPos(window, &lastx, &lasty);
}


void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

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


void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void Init(){
  int id = mj_name2id(m,mjOBJ_SENSOR,"fr_thigh_joint_p");
  if(id == -1){
    cout<<"没有 找到这个sensor"<<endl;
  }
  int data_pos = m->sensor_adr[id];
  vector<float> sd (m->sensor_dim[id]);
  for(int i=0;i<sd.size();i++){
    cout<<"i: "<< i <<" "<< d->sensordata[data_pos+i]<<endl;
  }
}
// main function
int main(int argc, const char** argv) {

  char error[1000] = "Could not load binary model";
  if(use_go1_model == 1){
    m = mj_loadXML("/home/loaf/WT_MPC/model/urdf/go1.xml", 0, error, 1000);
  }
  else{
    m = mj_loadXML("/home/loaf/WT_MPC/model/White_Tigger.xml", 0, error, 1000);
  }
  
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);
  startup_key_id = mj_name2id(m, mjOBJ_KEY, "sit_down_pose");
  if (startup_key_id == -1) {
    startup_key_id = mj_name2id(m, mjOBJ_KEY, "init_pose");
  }
  reset_to_startup_key();

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  using clock_type = std::chrono::steady_clock;
  start sim_start(m, d);
  const double render_dt = 1.0 / 90.0;
  const double realtime_factor = 1.0;

  double next_ctrl_time = d->time;
  double sim_time0 = d->time;
  auto wall_time0 = clock_type::now();
  glfwSwapInterval(0);
  while (!glfwWindowShouldClose(window)) {
    if (reset_requested) {
      reset_requested = false;
      reset_to_startup_key();
      sim_start.reset();
      next_ctrl_time = d->time;
      sim_time0 = d->time;
      wall_time0 = clock_type::now();
    }

    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0/90.0) {
      if (d->time + 1e-12 >= next_ctrl_time) {
      sim_start.run();                  // 控制器 500Hz, ctrl->dt = 0.002
      next_ctrl_time += sim_start.ctrl->dt;
      }
      mj_step(m, d);
    }
    
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mj_forward(m, d);
    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
    auto target_wall_time =
      wall_time0 + std::chrono::duration<double>(
          (d->time - sim_time0) / realtime_factor);

    std::this_thread::sleep_until(target_wall_time);
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return EXIT_SUCCESS;
}
