#include "boid.h"

#include "camera.h"
#include "vertexrecorder.h"

Boid::Boid()
{
    // initialize the boid system
    std::vector<Vector3f> initialState;
    for (int i=0;i<7;i++) {
        initialState.push_back(Vector3f(0.1,rand_uniform(0.0, 1.0),rand_uniform(0.0, 1.0)));
    }
    setState(initialState);
}

std::vector<Vector3f> Boid::evalF(std::vector<Vector3f> state)
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
void Boid::draw(GLProgram& gl)
{
    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    std::vector<Vector3f> currentState = getState();
    for (int i = 0; i < currentState.size(); ++i) {
        Vector3f position = currentState[i];
        gl.updateModelMatrix(Matrix4f::translation(position));
        drawTriangle(0.3f);
    } 
}
