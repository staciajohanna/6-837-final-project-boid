#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

const float PARTICLE_MASS = 1;
const float DRAG = 0.5;
const float RESTING_LENGTH = 0.15;
const float STIFFNESS = 30;
const Vector3f g = Vector3f(0, -9.18, 0);

PendulumSystem::PendulumSystem()
{
    // 4.2 Add particles for simple pendulum
    std::vector<Vector3f> initialState;
    initialState.push_back(Vector3f(-0.5, 0.7, 0));
    initialState.push_back(Vector3f(0.0,0.0,0.0));
    initialState.push_back(Vector3f(-0.5, 0.4, 0));
    initialState.push_back(Vector3f(0.0,0.0,0.0));
    initialState.push_back(Vector3f(-0.5, 0.3, 0));
    initialState.push_back(Vector3f(0.0,0.0,0.0));
    initialState.push_back(Vector3f(-0.5, 0.01, 0));
    initialState.push_back(Vector3f(0.0,0.0,0.0));
    setState(initialState);
    // 4.3 Extend to multiple particles

    // To add a bit of randomness, use e.g.
    // float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.
}


std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    Vector3f HINGE = Vector3f(-0.5, 1.0, 0);
    // 4.1: implement evalF
    //  - gravity
    //  - viscous drag
    //  - springs
    for (int i = 0; i < state.size()/2; ++i) {
        Vector3f &pos = state[i * 2];
        Vector3f &vel = state[i * 2 + 1];
        // calculate gravity 
        Vector3f gravity = PARTICLE_MASS * g;
        // calculate viscous drag
        Vector3f viscous_drag = -DRAG * vel;
        // calculate springs 
        int beforeIndex;
        Vector3f beforePos;
        if (i == 0){beforePos = HINGE;} 
        else {
            beforeIndex = (i - 1) * 2;
            beforePos = state[beforeIndex];
        } 
        Vector3f d = beforePos - pos;
        Vector3f springForce = STIFFNESS * (d.abs() - RESTING_LENGTH)* d.normalized();
        if (i < (state.size() - 2)/2) {
            int afterIndex = (i + 1) * 2;
            Vector3f afterPos = state[afterIndex];
            d =  pos - afterPos;
            springForce += -STIFFNESS * (d.abs() - RESTING_LENGTH)* d.normalized(); 
        }
        Vector3f netForce = gravity + viscous_drag + springForce;
        f.push_back(vel);
        f.push_back(netForce);
    }
    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // 4.2, 4.3

    gl.updateModelMatrix(Matrix4f::translation(Vector3f(-0.5, 1.0, 0)));
    drawSphere(0.075f, 10, 10);
    std::vector<Vector3f> currentState = getState();
    for (int i = 0; i < currentState.size()/2; ++i) {
        Vector3f position = currentState[i * 2];
        gl.updateModelMatrix(Matrix4f::translation(position));
        drawSphere(0.075f, 10, 10);
    } 
}
