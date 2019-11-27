#include "boid.h"

#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

const float NEIGHBOR_RADIUS = 1.0;

Boid::Boid()
{
    // initialize the boid system
    std::vector<Vector3f> initialState;
    for (int i=0;i<7;i++) {
        initialState.push_back(Vector3f(0.1,rand_uniform(0.0, 3.0),0.0));
        initialState.push_back(Vector3f(0.0,0.0,0.0));
    }
    setState(initialState);
}

bool isNeighbor(Vector3f current, Vector3f neighbor) 
{
    if ((current - neighbor).abs() <= NEIGHBOR_RADIUS) {
        return true;
    } return false;
}

Vector3f getSeparationForce(std::vector<Vector3f> &state, int birdIndex) 
{
    Vector3f &curPos = state[birdIndex * 2];
    Vector3f &curVel = state[birdIndex * 2 + 1];
    Vector3f res = Vector3f(0.0);
    int numOfNeighbor = 0;
    for (int i=0;i<state.size()/2;i++) {
        if (i == birdIndex) continue;
        Vector3f &pos = state[i * 2];
        Vector3f &vel = state[i * 2 + 1];
        //if (!isNeighbor(curPos, pos)) continue;
        float dist = sqrt((curPos - pos).absSquared());
        Vector3f dir = curPos - pos;
        res += dir.normalized()/(dist*dist);
        numOfNeighbor++;
    }
    if (numOfNeighbor > 0) res /= (float)numOfNeighbor;
    return res;
}

Vector3f getAlignmentForce(std::vector<Vector3f> &state, int birdIndex) 
{
    Vector3f &curPos = state[birdIndex * 2];
    Vector3f &curVel = state[birdIndex * 2 + 1];
    Vector3f vAvg = Vector3f(0.0);
    int numOfNeighbor = 0;
    for (int i=0;i<state.size()/2;i++) {
        if (i == birdIndex) continue;
        Vector3f &pos = state[i * 2];
        Vector3f &vel = state[i * 2 + 1];
        //if (!isNeighbor(curPos, pos)) continue;
        vAvg += vel;
        numOfNeighbor++;
    }
    if (numOfNeighbor > 0) vAvg /= (float)numOfNeighbor;
    return vAvg - curVel;
}

std::vector<Vector3f> Boid::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    for (int i = 0; i < state.size()/2; ++i) {
        Vector3f &pos = state[i * 2];
        Vector3f &vel = state[i * 2 + 1];
        Vector3f separationForce = getSeparationForce(state, i);
        Vector3f alignmentForce = getAlignmentForce(state, i);
        Vector3f cohesionForce = vel + alignmentForce;
        Vector3f netForce = separationForce + alignmentForce + cohesionForce;
        f.push_back(vel);
        f.push_back(netForce);
    }
    return f;
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
        drawBird(0.15f);
    } 
}
