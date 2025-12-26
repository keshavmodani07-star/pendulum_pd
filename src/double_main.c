#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// -----------------------------
// Global state
// -----------------------------
mjModel* m = NULL;
mjData*  d = NULL;
GLFWwindow* window = NULL;

int site_ee = -1;
double t_sim = 0.0;

// Visualization
mjvCamera  cam;
mjvOption  opt;
mjvScene   scn;
mjrContext con;

// -----------------------------
// Utilities
// -----------------------------
void die(const char* msg)
{
    printf("ERROR: %s\n", msg);
    exit(1);
}

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// -----------------------------
// Main
// -----------------------------
int main(void)
{
    // Load model
    char error[1000];
    m = mj_loadXML("../assets/double_pendulum.xml", NULL, error, 1000);
    if (!m) die(error);

    d = mj_makeData(m);

    site_ee = mj_name2id(m, mjOBJ_SITE, "endeff");
    if (site_ee < 0) die("End-effector site not found");

    // GLFW
    if (!glfwInit())
        die("GLFW init failed");

    window = glfwCreateWindow(1200, 900, "Jacobian IK - Small Circle", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, keyboard);

    // MuJoCo visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 1000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Camera setup
    cam.distance  = 5.0;
    cam.azimuth   = 90.0;
    cam.elevation = -15.0;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 1.25;

    // -----------------------------
    // Main loop
    // -----------------------------
    while (!glfwWindowShouldClose(window))
    {
        // ---------------------------------
        // Small circular target (REACHABLE)
        // ---------------------------------
        t_sim += 0.01;

        double cx = 0.45;   // center X
        double cz = 1.05;   // center Z
        double r  = 0.12;   // SMALL radius

        double target[3];
        target[0] = cx + r * cos(t_sim);
        target[1] = 0.0;
        target[2] = cz + r * sin(t_sim);

        // ---------------------------------
        // Current end-effector position
        // ---------------------------------
        const double* ee = d->site_xpos + 3 * site_ee;

        double err_x = target[0] - ee[0];
        double err_z = target[2] - ee[2];

        // ---------------------------------
        // Jacobian
        // ---------------------------------
        double Jpos[3 * m->nv];
        double Jrot[3 * m->nv];
        mj_jacSite(m, d, Jpos, Jrot, site_ee);

        // ---------------------------------
        // Planar Jacobian Transpose IK
        // ---------------------------------
        double alpha = 0.3;

        for (int j = 0; j < m->nv; j++)
        {
            double dq =
                Jpos[0 * m->nv + j] * err_x +
                Jpos[2 * m->nv + j] * err_z;

            d->qpos[j] += alpha * dq;
        }

        // Kinematics only
        mj_forward(m, d);

        // ---------------------------------
        // Render
        // ---------------------------------
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwTerminate();

    return 0;
}
