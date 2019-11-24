#include "simplesystem.h"

#include "camera.h"
#include "vertexrecorder.h"


SimpleSystem::SimpleSystem()
{
    // 3.2 initialize the simple system
    std::vector<Vector3f> initialState;
    // initialState.push_back(Vector3f(0.01,0.01,0.02));
    // initialState.push_back(Vector3f(0.01,0.2,0.3));
    initialState.push_back(Vector3f(0.1,2,0.03));
    setState(initialState);
}

std::vector<Vector3f> SimpleSystem::evalF(std::vector<Vector3f> state)
{
    // for a given state, evaluate f(X,t)
    std::vector<Vector3f> derivatives;
    for (int i = 0; i < state.size(); ++i) {
        Vector3f &vstate = state[i];
        float x = vstate[0];
        float y = vstate[1];
        float z = vstate[2];
        derivatives.push_back(Vector3f(-y,x,0));
    }
    return derivatives;
}

// render the system (ie draw the particles)
void SimpleSystem::draw(GLProgram& gl)
{

    // TODO 3.2: draw the particle. 
    //           we provide code that draws a static sphere.
    //           you should replace it with your own
    //           drawing code.
    //           In this assignment, you must manage two
    //           kinds of uniforms before you draw
    //            1. Update material uniforms (color)
    //            2. Update transform uniforms
    //           GLProgram is a helper object that has
    //           methods to set the uniform state.

    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    std::vector<Vector3f> currentState = getState();
    for (int i = 0; i < currentState.size(); ++i) {
        Vector3f position = currentState[i];
        gl.updateModelMatrix(Matrix4f::translation(position));
        drawSphere(0.075f, 10, 10);
    } 
    // Vector3f pos(1, 0, 0); //YOUR PARTICLE POSITION
    // gl.updateModelMatrix(Matrix4f::translation(pos));
    // drawSphere(0.075f, 10, 10);
}
