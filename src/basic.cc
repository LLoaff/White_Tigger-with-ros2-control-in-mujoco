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
double sim_speed = 1.0;
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
  if (act == GLFW_PRESS) {
    if (key == GLFW_KEY_EQUAL || key == GLFW_KEY_KP_ADD) {
        sim_speed *= 2.0;

        if (sim_speed > 8.0) {
            sim_speed = 8.0;
        }

        std::printf("仿真速度：%.2fx\n", sim_speed);
    }

    if (key == GLFW_KEY_MINUS || key == GLFW_KEY_KP_SUBTRACT) {
        sim_speed *= 0.5;

        if (sim_speed < 0.125) {
            sim_speed = 0.125;
        }

        std::printf("仿真速度：%.3fx\n", sim_speed);
    }
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
  #ifdef USE_GO1_MODEL
    m = mj_loadXML("/home/loaf/WT_MPC/model/urdf/go1.xml", 0, error, 1000);
  #else
    m = mj_loadXML("/home/loaf/WT_MPC/model/White_Tigger_simple_longleg.xml", 0, error, 1000);
  #endif
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

  // using clock_type = std::chrono::steady_clock;
  // start sim_start(m, d);
  // const double render_dt = 1.0 / 90.0;
  // const double realtime_factor = 1.0;

  // double next_ctrl_time = d->time;
  // double sim_time0 = d->time;
  // auto wall_time0 = clock_type::now();
  // glfwSwapInterval(0);
  using clock_type = std::chrono::steady_clock;
  start sim_start(m, d);

  double next_ctrl_time = d->time;

  // 尚未执行的仿真时间，单位是秒
  double sim_accumulator = 0.0;

  // 上一次进入主循环时的现实时间
  auto last_wall_time = clock_type::now();

  while (!glfwWindowShouldClose(window)) {
    // if (reset_requested) {
    //   reset_requested = false;
    //   reset_to_startup_key();
    //   sim_start.reset();
    //   next_ctrl_time = d->time;
    //   sim_time0 = d->time;
    //   wall_time0 = clock_type::now();
    // }
    if (reset_requested) {
        reset_requested = false;

        reset_to_startup_key();
        sim_start.reset();

        next_ctrl_time = d->time;

        // 丢弃重置前积攒的仿真时间
        sim_accumulator = 0.0;
        last_wall_time = clock_type::now();
    }
    // mjtNum simstart = d->time;
    // while (d->time - simstart < sim_speed/90.0) {
    //   if (d->time + 1e-12 >= next_ctrl_time) {
    //     sim_start.run();                  // 控制器 500Hz, ctrl->dt = 0.002
    //     next_ctrl_time += sim_start.ctrl->dt;
    //   }
    //   mj_step(m, d);
    // }
    // 获取当前现实时间
    auto current_wall_time = clock_type::now();

    // 计算从上一帧到现在，现实世界经过了多少秒
    double wall_dt =
        std::chrono::duration<double>(
            current_wall_time - last_wall_time
        ).count();

    last_wall_time = current_wall_time;

    // 防止拖动窗口、调试断点或程序卡顿后一次性补算太久
    if (wall_dt > 0.1) {
        wall_dt = 0.1;
    }

    // 根据速度倍率，计算应该推进多少仿真时间
    sim_accumulator += wall_dt * sim_speed;

    // 每次循环只推进一个固定物理步：0.002 秒
    while (sim_accumulator + 1e-12 >= m->opt.timestep) {
        // 控制器必须放在 mj_step() 前面
        if (d->time + 1e-12 >= next_ctrl_time) {
            sim_start.run();
            next_ctrl_time += sim_start.ctrl->dt;
        }

        // 使用刚刚计算出的控制量，推进一次物理仿真
        mj_step(m, d);

        // 消耗一个物理步的仿真时间
        sim_accumulator -= m->opt.timestep;
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
    // auto target_wall_time =
    //   wall_time0 + std::chrono::duration<double>(
    //       (d->time - sim_time0) / realtime_factor);

    // std::this_thread::sleep_until(target_wall_time);
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
