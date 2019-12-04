#include "boid.h"
#include "gl.h"
#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <vector>

#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

const float NEIGHBOR_RADIUS = 5.0;
const float WEIGHT_SEPARATION = 1.5;
const float WEIGHT_ALIGNMENT = 1.0;
const float WEIGHT_COHESION = 1.0;
const float WEIGHT_SEEK = 4.0;
const float WEIGHT_COLLISION_AVOIDANCE = 1.0;
Vector3f cursorPosition;

Boid::Boid()
{
    // initialize the boid system
    std::vector<Vector3f> initialState;
    for (int i=0;i<7;i++) {
        initialState.push_back(Vector3f(rand_uniform(0.0, 2.0),rand_uniform(-5.0, -4.0),0.0));
        initialState.push_back(Vector3f(0.0,1.0,0.0));
    }
    for (int i=0;i<7;i++) {
        initialState.push_back(Vector3f(rand_uniform(-5.0, -4.0),rand_uniform(0.0, 2.0),0.0));
        initialState.push_back(Vector3f(1.0,0.0,0.0));
    }
    setState(initialState);
}

bool isNeighbor(Vector3f current, Vector3f neighbor) 
{
    
    if ((current - neighbor).abs() < NEIGHBOR_RADIUS - 0.00001) {
        return true;
    } 
    return false;
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
        if (!isNeighbor(curPos, pos)) continue;
        float dist = (curPos - pos).abs();
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
        if (!isNeighbor(curPos, pos)) continue;
        vAvg += vel;
        numOfNeighbor++;
    }
    if (numOfNeighbor > 0) vAvg /= (float)numOfNeighbor;
    return vAvg - curVel;
}

Vector3f getSeekForce(std::vector<Vector3f> &state, int birdIndex) 
{
        
        Vector3f &curPos = state[birdIndex * 2];
        Vector3f &curVel = state[birdIndex * 2 + 1];
        // double xd, yd;
        // glfwGetCursorPos(window, &xd, &yd);
        // cursorX = (int)xd;
        // cursorY = (int)yd;
        float targetX = (cursorPosition[0] - 512.0f) * (4.0f/1024.0f);
        float targetY = -1 * (cursorPosition[1] - 402.0f) * (4.0f/804.0f);
        // float targetX = -10.0 - (1024 / (cursorPosition[0] + 1) * 2);
        // float targetY = 10 - (804 / (cursorPosition[1] + 1) * 2);
        // Vector3f target = Vector3f(targetX, targetY, 0);
        Vector3f target = Vector3f(targetX, targetY, 0);
        std::cout << cursorPosition[0] << "cursorX IS PRINTING \n";
        std::cout << cursorPosition[1] << "cursorY IS PRINTING \n";
        std::cout << targetX << "targetX IS PRINTING \n";
        std::cout << targetY << "targetY IS PRINTING \n";
        Vector3f desiredVelocity = (target - curPos).normalized();
        Vector3f steeringForce = desiredVelocity - curVel;
        return steeringForce;
}

Vector3f getCollisionAvoidanceForce(std::vector<Vector3f> &state, int birdIndex) 
{

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
        Vector3f seekForce = getSeekForce(state, i);
        Vector3f collisionAvoidanceForce = Vector3f(0,0,0);
        Vector3f netForce = WEIGHT_SEPARATION * separationForce + 
                            WEIGHT_ALIGNMENT * alignmentForce + 
                            WEIGHT_COHESION * cohesionForce +
                            WEIGHT_SEEK * seekForce +
                            WEIGHT_COLLISION_AVOIDANCE * collisionAvoidanceForce;
        f.push_back(vel);
        f.push_back(netForce);
    }
    return f;
}

// render the system (ie draw the particles)
void Boid::draw(GLProgram& gl)
{
    const Vector3f PARTICLE_COLOR(0.1f, 0.1f, 0.1f);
    gl.updateMaterial(PARTICLE_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(Vector3f(-0.5, 1.0, 0)));
    std::vector<Vector3f> currentState = getState();
    for (int i = 0; i < currentState.size()/2; ++i) {
        Vector3f position = currentState[i * 2];
        Vector3f velocity = currentState[i * 2 + 1];
        Vector3f a = Vector3f(1, 1, 0);
        float rad = acos((Vector3f::dot(a, velocity))/(a.abs() * velocity.abs()));
        gl.updateModelMatrix(Matrix4f::translation(position)*Matrix4f::rotation(Vector3f(0,0, -1), rad));
        drawBird(0.15f);
    } 
}

void Boid::setCursorPosition(int cursorX, int cursorY) {
    cursorPosition = Vector3f(cursorX, cursorY, 0);
}