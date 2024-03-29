#include "gl.h"
#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include "vertexrecorder.h"
#include "starter3_util.h"
#include "camera.h"
#include "timestepper.h"
#include "boid.h"

using namespace std;

namespace
{

// Declarations of functions whose implementations occur later.
void initSystem();
void stepSystem();
void drawSystem();
void freeSystem();
void resetTime();

void initRendering();
void drawAxis();

// Some constants
const Vector3f LIGHT_POS(3.0f, 3.0f, 5.0f);
// originally: const Vector3f LIGHT_COLOR(120.0f, 120.0f, 120.0f);
Vector3f LIGHT_COLOR(120.0f, 120.0f, 120.0f);
Vector3f FLOOR_COLOR(0.656f, 0.398f, 0.195f);
// originally: const Vector3f FLOOR_COLOR(1.0f, 0.0f, 0.0f);
// std::vector<string> settings { "sunset", "midday" };

string SCENE_SETTING = "sunset";

bool drawObstacle = true;
bool seekCursorMode = false;

// time keeping
// current "tick" (e.g. clock number of processor)
uint64_t start_tick;
// number of seconds since start of program
double elapsed_s;
// number of seconds simulated
double simulated_s;

// Globals here.
TimeStepper* timeStepper;
float h;
char integrator;
int cursorX;
int cursorY;

Camera camera;
bool gMousePressed = false;
GLuint program_color;
GLuint program_light;

Boid* boid;

// Function implementations

static void toggleScene() {
    if (SCENE_SETTING == "sunset") {
        SCENE_SETTING = "midday";
        LIGHT_COLOR = Vector3f(0.656f * 500, 0.398f * 500, 0.195f * 500);
        FLOOR_COLOR = Vector3f(0.203f, 0.5f, 0.976f);
    } else {
        SCENE_SETTING = "sunset";
        LIGHT_COLOR = Vector3f(120.0f, 120.0f, 120.0f);
        FLOOR_COLOR = Vector3f(0.656f, 0.398f, 0.195f);
    }
}

void resetObstacleMode() {
    drawObstacle = false;
    boid->setDrawObstacle(false);
}

void resetSeekCursorMode() {
    seekCursorMode = false;
    boid->setCursorMode(false);
}

static void keyCallback(GLFWwindow* window, int key,
    int scancode, int action, int mods)
{
    if (action == GLFW_RELEASE) { // only handle PRESS and REPEAT
        return;
    }

    // Special keys (arrows, CTRL, ...) are documented
    // here: http://www.glfw.org/docs/latest/group__keys.html
    switch (key) {
    case GLFW_KEY_ESCAPE: // Escape key
        exit(0);
        break;
    case ' ':
    {
        Matrix4f eye = Matrix4f::identity();
        camera.SetRotation(eye);
        camera.SetCenter(Vector3f(0, 0, 0));
        break;
    }
    case 'R':
    {
        cout << "Resetting simulation\n";
        freeSystem();
        initSystem();
        resetTime();
        break;
    }
    case 'B': 
    {
        toggleScene();
        break;
    }
    case 'C':
    {
        seekCursorMode = !seekCursorMode;
        boid->setCursorMode(seekCursorMode);
        if (seekCursorMode) {
            // reset other modes
            resetObstacleMode();
        }
        break;
    }
    case 'O':
    {
        drawObstacle = !drawObstacle;
        boid->setDrawObstacle(drawObstacle);
        if (drawObstacle) {
            // reset other modes
            resetSeekCursorMode();
        }
        break;
    }
    default:
        cout << "Unhandled key press " << key << "." << endl;
    }
}

static void mouseCallback(GLFWwindow* window, int button, int action, int mods)
{
    double xd, yd;
    glfwGetCursorPos(window, &xd, &yd);
    int x = (int)xd;
    int y = (int)yd;

    int lstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    int rstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    int mstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
    if (lstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::LEFT, x, y);
    }
    else if (rstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::RIGHT, x, y);
    }
    else if (mstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::MIDDLE, x, y);
    }
    else {
        gMousePressed = true;
        camera.MouseRelease(x, y);
        gMousePressed = false;
    }
}

static void motionCallback(GLFWwindow* window, double x, double y)
{
    if (!gMousePressed) {
        return;
    }
    camera.MouseDrag((int)x, (int)y);
}

void setViewport(GLFWwindow* window)
{
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    camera.SetDimensions(w, h);
    camera.SetViewport(0, 0, w, h);
    camera.ApplyViewport();
}

static Vector3f getCursorPosition() {
        return Vector3f(cursorX, cursorY, 0);
}

void drawAxis()
{
    glUseProgram(program_color);
    Matrix4f M = Matrix4f::translation(camera.GetCenter()).inverse();
    camera.SetUniforms(program_color, M);

    const Vector3f DKRED(1.0f, 0.5f, 0.5f);
    const Vector3f DKGREEN(0.5f, 1.0f, 0.5f);
    const Vector3f DKBLUE(0.5f, 0.5f, 1.0f);
    const Vector3f GREY(0.5f, 0.5f, 0.5f);

    const Vector3f ORGN(0, 0, 0);
    const Vector3f AXISX(5, 0, 0);
    const Vector3f AXISY(0, 5, 0);
    const Vector3f AXISZ(0, 0, 5);

    VertexRecorder recorder;
    recorder.record_poscolor(ORGN, DKRED);
    recorder.record_poscolor(AXISX, DKRED);
    recorder.record_poscolor(ORGN, DKGREEN);
    recorder.record_poscolor(AXISY, DKGREEN);
    recorder.record_poscolor(ORGN, DKBLUE);
    recorder.record_poscolor(AXISZ, DKBLUE);

    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISX, GREY);
    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISY, GREY);
    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISZ, GREY);

    glLineWidth(3);
    recorder.draw(GL_LINES);
}

// initialize your particle systems
void initSystem()
{
    switch (integrator) {
    case 'e': timeStepper = new ForwardEuler(); break;
    case 't': timeStepper = new Trapezoidal(); break;
    case 'r': timeStepper = new RK4(); break;
    default: printf("Unrecognized integrator\n"); exit(-1);
    }

    boid = new Boid();
}

void freeSystem() {
    delete timeStepper; timeStepper = nullptr;
    delete boid; boid = nullptr;
}

void resetTime() {
    elapsed_s = 0;
    simulated_s = 0;
    start_tick = glfwGetTimerValue();
}

// TODO: To add external forces like wind or turbulances,
//       update the external forces before each time step
void stepSystem()
{
    // step until simulated_s has caught up with elapsed_s.
    while (simulated_s < elapsed_s) {
        timeStepper->takeStep(boid, h);
        simulated_s += h;
    }
}

// Draw the current particle positions
void drawSystem()
{
    // GLProgram wraps up all object that
    // particle systems need for drawing themselves
    GLProgram gl(program_light, program_color, &camera);
    gl.updateLight(LIGHT_POS, LIGHT_COLOR.xyz()); // once per frame

    boid->draw(gl);

    // set uniforms for floor
    gl.updateMaterial(FLOOR_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(0, -5.0f, 0));
    // draw floor
    drawFloor(100.0f);

    // draw obstacles
    if (drawObstacle) {
        const Vector3f OBSTACLE_COLOR(0.69f, 0.46f, 0.46f);
        gl.updateMaterial(OBSTACLE_COLOR);
        // obst 1
        gl.updateModelMatrix(Matrix4f::translation(Vector3f(-3.0, 2.5, 0)));
        drawSphere(0.5f, 50, 50);
        // obst 2
        gl.updateModelMatrix(Matrix4f::translation(Vector3f(-3.0, -1.5, 0)));
        drawSphere(0.7f, 50, 50);
        // obst 3
        gl.updateModelMatrix(Matrix4f::translation(Vector3f(3.0, 1.5, 0)));
        drawSphere(0.4f, 50, 50);
        // obst 4
        gl.updateModelMatrix(Matrix4f::translation(Vector3f(1.5, -1.5, 0)));
        drawSphere(0.8f, 50, 50);
        // obst 5
        gl.updateModelMatrix(Matrix4f::translation(Vector3f(5.0, 3.0, 0)));
        drawSphere(0.8f, 50, 50);
    }
}

//-------------------------------------------------------------------

void initRendering()
{
    // Clear to black
    glClearColor(0, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}
}

// Main routine.
// Set up OpenGL, define the callbacks and start the main loop
int main(int argc, char** argv)
{

    if (argc != 3) {
        printf("Usage: %s <e|t|r> <timestep>\n", argv[0]);
        printf("       e: Integrator: Forward Euler\n");
        printf("       t: Integrator: Trapezoid\n");
        printf("       r: Integrator: RK 4\n");
        printf("\n");
        printf("Try  : %s t 0.001\n", argv[0]);
        printf("       for trapezoid (1ms steps)\n");
        printf("Or   : %s r 0.01\n", argv[0]);
        printf("       for RK4 (10ms steps)\n");
        return -1;
    }

    integrator = argv[1][0];
    h = (float)atof(argv[2]);
    printf("Using Integrator %c with time step %.4f\n", integrator, h);


    GLFWwindow* window = createOpenGLWindow(1024, 1024, "Boids");

    // setup the event handlers
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseCallback);
    glfwSetCursorPosCallback(window, motionCallback);

    initRendering();

    // The program object controls the programmable parts
    // of OpenGL. All OpenGL programs define a vertex shader
    // and a fragment shader.
    program_color = compileProgram(c_vertexshader, c_fragmentshader_color);
    if (!program_color) {
        printf("Cannot compile program\n");
        return -1;
    }
    program_light = compileProgram(c_vertexshader, c_fragmentshader_light);
    if (!program_light) {
        printf("Cannot compile program\n");
        return -1;
    }

    camera.SetDimensions(600, 600);
    camera.SetPerspective(50);
    camera.SetDistance(10);

    // Setup particle system
    initSystem();

    // Main Loop
    uint64_t freq = glfwGetTimerFrequency();
    resetTime();
    while (!glfwWindowShouldClose(window)) {
        // Clear the rendering window
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        setViewport(window);

        if (gMousePressed) {
            drawAxis();
        }

        uint64_t now = glfwGetTimerValue();
        elapsed_s = (double)(now - start_tick) / freq;
        stepSystem();

        // Draw the simulation
        drawSystem();

        // Make back buffer visible
        glfwSwapBuffers(window);

        // Check if any input happened during the last frame
        glfwPollEvents();
        double xd, yd;
        glfwGetCursorPos(window, &xd, &yd);
        cursorX = (int)xd;
        cursorY = (int)yd;
        boid->setCursorPosition(cursorX, cursorY);
    }

    // All OpenGL resource that are created with
    // glGen* or glCreate* must be freed.
    glDeleteProgram(program_color);
    glDeleteProgram(program_light);


    return 0;	// This line is never reached.
}
