#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

// your system should at least contain 8x8 particles.
const int W = 10;
const int H = 10;

const float PARTICLE_MASS = 1;
const float DRAG = 150;
const float RESTING_LENGTH_VERTICAL = 0.19;
const float RESTING_LENGTH = 0.19; 
const float RESTING_LENGTH_SHEAR = 0.27; 
const float STIFFNESS = 500;
const float STIFFNESS_STRUCT_SIDE = 250;
const float STIFFNESS_SHEAR = 85;
const float RESTING_LENGTH_FLEX = 0.38; 
const float STIFFNESS_FLEX = 1000;
const Vector3f g = Vector3f(0, -9.18, 0);

ClothSystem::ClothSystem()
{
    // 5. Initialize m_vVecState with cloth particles. 
    std::vector<Vector3f> initialState;
    Vector3f O(0.4f, 1, 0);
    Vector3f l;
    float w = 0.2f;
    for (int i = 0; i< W; i++) {
        for (int j = 0; j< H; j++) {
            l = O + Vector3f(i * w, - j * w, 0);
            initialState.push_back(l);
            initialState.push_back(Vector3f(0.0,0.0,0.0));
        } 
    } 
    setState(initialState);
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
}


std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    Vector3f O(0.4f, 1, 0);
    float w = 0.2f;
    // 5. implement evalF
    // - gravity
    // - viscous drag
    // - structural springs
    // - shear springs
    // - flexion springs
     
    for (int i = 0; i < state.size()/2; ++i) {
        Vector3f &pos = state[i * 2];
        Vector3f &vel = state[i * 2 + 1];
        // calculate gravity 
        Vector3f gravity = PARTICLE_MASS * g; // TODO put gravity back
        // calculate viscous drag
        Vector3f viscous_drag = -DRAG * vel;
        // calculate springs 
        // structural springs
        int x = i / H; 
        int y = i % H;
        Vector3f d; 
        Vector3f structuralSpringForce = Vector3f(0,0,0);
        Vector3f shearSpringForce = Vector3f(0,0,0);
        Vector3f flexSpringForce = Vector3f(0,0,0);
        // draw vertical structural springs if x < W and y < (H-1) -- connect (x, y) to (x, y + 1)
        if (y < (H-1)) {
            // Vector3f position1 = state[((x * H) + y) * 2];
            Vector3f position1 = pos;
            Vector3f position2 = state[(i + 1) * 2];
            d =  position1 - position2;
            structuralSpringForce += -STIFFNESS * (d.abs() - RESTING_LENGTH_VERTICAL)* d.normalized(); 
        }
        // draw vertical structural springs if x < W and y < (H-1) -- connect (x, y - 1) to (x, y)
        if (y > 0) {
            Vector3f position1 = state[(i - 1) * 2];
            Vector3f position2 = pos;
            d =  position1 - position2;
            structuralSpringForce += STIFFNESS * (d.abs() - RESTING_LENGTH_VERTICAL)* d.normalized(); 
        }
        // draw horizontal structural springs if x < (W-1) -- connect (x, y) to (x + 1, y)
        if (x < (W-1)) {
            Vector3f position1 = pos;
            Vector3f position2 = state[(i + W) * 2];
            d =  position1 - position2;
            structuralSpringForce += -STIFFNESS_STRUCT_SIDE * (d.abs() - RESTING_LENGTH)* d.normalized(); 
        }
        // draw horizontal structural springs if x > 0 -- connect (x - 1, y) to (x, y)
        if (x > 0) {
            Vector3f position1 = state[(i - W) * 2];
            Vector3f position2 = pos;
            d =  position1 - position2;
            structuralSpringForce += STIFFNESS_STRUCT_SIDE * (d.abs() - RESTING_LENGTH)* d.normalized(); 
        }
        // shear 
        if (y < (H-1) && x < (W-1)) {
            Vector3f position1 = pos;
            Vector3f position2 = state[(((x + 1) * H) + y + 1) * 2];
            d =  position1 - position2;
            shearSpringForce += -STIFFNESS_SHEAR * (d.abs() - RESTING_LENGTH_SHEAR)* d.normalized(); 
        }
        if (y > 0 && x > 0) {
            Vector3f position1 = state[(((x - 1) * H) + y - 1) * 2];
            Vector3f position2 = pos;
            d =  position1 - position2;
            shearSpringForce += STIFFNESS_SHEAR * (d.abs() - RESTING_LENGTH_SHEAR)* d.normalized(); 
        }
        // draw vertical flexion springs
        if (y < (H-2)) {
            Vector3f position1 = pos;
            Vector3f position2 = state[(i + 2) * 2];
            d =  position1 - position2;
            flexSpringForce += -STIFFNESS_FLEX * (d.abs() - RESTING_LENGTH_FLEX)* d.normalized(); 
        }
        // draw vertical flexion springs
        if (y > 1) {
            Vector3f position1 = state[(i - 2) * 2];
            Vector3f position2 = pos;
            d =  position1 - position2;
            flexSpringForce += STIFFNESS_FLEX * (d.abs() - RESTING_LENGTH_FLEX)* d.normalized(); 
        }
        // draw horizontal flexion springs
        if (x < (W-2)) {
            Vector3f position1 = pos;
            Vector3f position2 = state[(i + 2 * W) * 2];
            d =  position1 - position2;
            flexSpringForce += -STIFFNESS_FLEX * (d.abs() - RESTING_LENGTH_FLEX)* d.normalized(); 
        }
        // draw horizontal flexion springs
        if (x > 1) {
            Vector3f position1 = state[(i - 2 * W) * 2];
            Vector3f position2 = pos;
            d =  position1 - position2;
            flexSpringForce += STIFFNESS_FLEX * (d.abs() - RESTING_LENGTH_FLEX)* d.normalized(); 
        }

        Vector3f netForce = gravity + viscous_drag + structuralSpringForce + shearSpringForce + flexSpringForce;
        if (i == 0 || i == ((W-1) * H)) {
            f.push_back(Vector3f(0,0,0));
            f.push_back(Vector3f(0,0,0));
        } else {
            f.push_back(vel);
            f.push_back(netForce);
        }
    }

    return f;
}


void ClothSystem::draw(GLProgram& gl)
{
    // 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);

    Vector3f O(0.4f, 1, 0);
    float w = 0.2f;
    std::vector<Vector3f> currentState = getState();
    for (int i = 0; i < currentState.size()/2; ++i) {
        Vector3f position = currentState[i * 2];
        gl.updateModelMatrix(Matrix4f::translation(position));
        drawSphere(0.075f, 10, 10);
    } 
    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;

    // draw vertical lines
    for (int i = 0; i< W; i++) {
        for (int j = 0; j<  (H - 1); j++) {
            // connect (i, j) to (i, j + 1)
            Vector3f position1 = currentState[((i * H) + j) * 2];
            Vector3f position2 = currentState[((i * H) + j + 1) * 2];
            rec.record(position1, CLOTH_COLOR);
            rec.record(position2, CLOTH_COLOR);
        } 
    }

    // draw horizontal lines
    for (int i = 0; i< (W - 1); i++) {
        for (int j = 0; j<  H; j++) {
            // connect (i, j) to (i + 1)
            Vector3f position1 = currentState[((i * H) + j) * 2];
            Vector3f position2 = currentState[(((i + 1) * H) + j) * 2];
            rec.record(position1, CLOTH_COLOR);
            rec.record(position2, CLOTH_COLOR);
        } 
    }  

    glLineWidth(3.0f);
    rec.draw(GL_LINES);

    gl.enableLighting(); // reset to default lighting model
}

