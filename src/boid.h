#ifndef BOID_H
#define BOID_H

#include <vector>

#include "particlesystem.h"

class Boid : public ParticleSystem
{
public:
    Boid();
    // evalF is a method defined in the ParticleSystem
    // interface. The use of virtual functions allows timesteppers to work
    // with any particle system (simple, pendulum, cloth), without
    // knowing which particular system it is.
    // Each ParticleSystem subclass must provide an implementation of evalF.
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    // this is called from main.cpp when it's time to draw a new frame.
    void draw(GLProgram&);

    void setCursorPosition(int cursorX, int cursorY);

    // inherits 
    // std::vector<Vector3f> m_vVecState;
};
#endif