#include <stdio.h>
#include <math.h>
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

#define KP 130.0
#define KD 3.0
#define TARGET_ANGLE M_PI

/* Globals */
mjModel* m = NULL;
mjData* d = NULL;

/* Visualization */
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window;

/* Wrap angle to [-pi, pi] */
double wrap_angle(double a)
{
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

/* PD Controller */
void controller(const mjModel* m, mjData* d)
{
    double theta = wrap_angle(d->qpos[0]);
    double theta_dot = d->qvel[0];

    double error = wrap_angle(TARGET_ANGLE - theta);

    double torque = KP * error - KD * theta_dot;

    d->ctrl[0] = torque;
}

int main(int argc, const char** argv)
{
    if (argc < 2) {
        printf("Usage: %s pendulum.xml\n", argv[0]);
        return 1;
    }

    char error[1000];
    m = mj_loadXML(argv[1], NULL, error, 1000);
    if (!m) {
        printf("%s\n", error);
        return 1;
    }

    d = mj_makeData(m);
    mjcb_control = controller;

    /* GLFW */
    glfwInit();
    window = glfwCreateWindow(1200, 900, "Inverted Pendulum (PD Control)", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    /* MuJoCo visualization */
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.distance = 6.0;
    cam.azimuth = 90;
    cam.elevation = -10;

    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 2.0;   // LOOK AT THE HINGE HEIGHT

    /* Start near upright but unstable */
    d->qpos[0] = 1.0;

    while (!glfwWindowShouldClose(window)) {

        mj_step(m, d);

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}

