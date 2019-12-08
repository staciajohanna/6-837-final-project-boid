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
#include <float.h>
using namespace std;

const float NEIGHBOR_RADIUS = 5.0;
const float WEIGHT_SEPARATION = 4.0;
const float WEIGHT_ALIGNMENT = 1.0;
const float WEIGHT_COHESION = 0.5;
const float WEIGHT_SEEK = 1.7;
const float WEIGHT_COLLISION_AVOIDANCE = 3.0;
const float SEE_FRONT = 2.0;
bool drawObstacle = true;
Vector3f cursorPosition;

const std::vector<std::pair<float, Vector3f> > OBSTACLE_POSITION = 
            { make_pair(0.8f, Vector3f(-3.0, 2.5, 0)),
              make_pair(1.0f, Vector3f(-3.0, -1.5, 0)),
              make_pair(0.7f, Vector3f(3.0, 1.5, 0)),
              make_pair(1.0f, Vector3f(1.5, -1.5, 0)),
              make_pair(1.2f, Vector3f(5.0, 3.0, 0)) };
float maxVelocity = 1.0f;
const float MIN_FLOAT = -999999.0;

Boid::Boid()
{
    // initialize the boid system
    std::vector<Vector3f> initialState;
    for (int i=0;i<20;i++) {
        initialState.push_back(Vector3f(rand_uniform(0.0, 2.0),rand_uniform(-5.0, -4.0),0.0));
        initialState.push_back(Vector3f(0.0,1.0,0.0));
    }
    for (int i=0;i<20;i++) {
        initialState.push_back(Vector3f(rand_uniform(-5.0, -4.0),rand_uniform(0.0, 2.0),0.0));
        initialState.push_back(Vector3f(1.0,0.0,0.0));
    }
    setState(initialState);
}

float getDistance(Vector3f a, Vector3f b)
{
    return (a - b).abs();
}

bool isNeighbor(Vector3f current, Vector3f neighbor) 
{
    if (getDistance(current, neighbor) < NEIGHBOR_RADIUS - 0.00001) {
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
        // translation, scaling, and adjustment
        float targetX = (cursorPosition[0] - 512.0f) * (4.0f/1024.0f) - 0.66; 
        // translation, scaling, reflection, and adjustment
        float targetY = -1 * (cursorPosition[1] - 402.0f) * (4.0f/804.0f) - 0.66;
        Vector3f target = Vector3f(targetX, targetY, 0);
        Vector3f desiredVelocity = (target - curPos).normalized();
        Vector3f steeringForce = desiredVelocity - curVel;
        return steeringForce;
}

bool isLineSphereIntersection(Vector3f headerVector, Vector3f headerVector2, pair<float, Vector3f> obstaclePos)
{
    float eps = 0.00000001f;
    return (getDistance(headerVector, obstaclePos.second) + eps) < obstaclePos.first ||
           (getDistance(headerVector2, obstaclePos.second) + eps) < obstaclePos.first;
}

pair<float, Vector3f> getClosestObstacle(Vector3f headerVector, Vector3f headerVector2, Vector3f curPos) 
{
    pair<float, Vector3f> res = make_pair(MIN_FLOAT, Vector3f(MIN_FLOAT, MIN_FLOAT, 0));
    // not draw obstacle
    if (!drawObstacle) return res;

    float eps = 0.00000001f;
    for (int i=0;i<OBSTACLE_POSITION.size();i++) 
    {
        float radius = OBSTACLE_POSITION[i].first;
        Vector3f obstaclePos = OBSTACLE_POSITION[i].second;
        bool isIntersect = isLineSphereIntersection(headerVector, headerVector2, OBSTACLE_POSITION[i]);
        // when res doesn't have obstacle yet, or if there's obstacle within line of sight.
        float distBoidToObstacle = getDistance(curPos, obstaclePos);
        float distBoidToCurClosestObstacle = getDistance(curPos, res.second);
        if (isIntersect && (abs(res.first-MIN_FLOAT) < eps || distBoidToObstacle < distBoidToCurClosestObstacle))
        {
            res = make_pair(radius, obstaclePos);
        }
    }
    return res;
}

Vector3f getCollisionAvoidanceForce(std::vector<Vector3f> &state, int birdIndex) 
{
    Vector3f &curPos = state[birdIndex * 2];
    Vector3f &curVel = state[birdIndex * 2 + 1];
    Vector3f headerVector = curPos + curVel.normalized() * (curVel.abs()/maxVelocity);
    Vector3f headerVector2 = curPos + curVel.normalized() * (curVel.abs()/maxVelocity) * 0.5f;
    pair<float, Vector3f> closestObstacle = getClosestObstacle(headerVector, headerVector2, curPos);
    if (closestObstacle.first > MIN_FLOAT) {
        return SEE_FRONT * (headerVector - closestObstacle.second).normalized();
    }
    else return Vector3f(0);
}

std::vector<Vector3f> Boid::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    // for calculating max velocity
    for (int i = 0; i < state.size()/2; ++i) {
        Vector3f &vel = state[i * 2 + 1];
        maxVelocity = max(maxVelocity, vel.abs());
    }
    for (int i = 0; i < state.size()/2; ++i) {
        Vector3f &pos = state[i * 2];
        Vector3f &vel = state[i * 2 + 1];
        Vector3f separationForce = getSeparationForce(state, i);
        Vector3f alignmentForce = getAlignmentForce(state, i);
        Vector3f cohesionForce = vel + alignmentForce;
        Vector3f seekForce = getSeekForce(state, i);
        Vector3f collisionAvoidanceForce = getCollisionAvoidanceForce(state, i);
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
        if (velocity.x() < 0) rad = -rad;
        gl.updateModelMatrix(Matrix4f::translation(position)*Matrix4f::rotation(Vector3f(0,0, -1), rad));
        drawBird(0.15f);
    } 
}

void Boid::setCursorPosition(int cursorX, int cursorY) {
    cursorPosition = Vector3f(cursorX, cursorY, 0);
}

void Boid::setDrawObstacle(bool isDraw) {
    drawObstacle = isDraw;
}