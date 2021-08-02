#ifndef TRIANGLE_PHYSICSWORLD_H
#define TRIANGLE_PHYSICSWORLD_H

#include "Colliders.h"
#include "Constraint.h"
#include "RigidBody.h"

class Constraint;

struct collision {
    RigidBody *rb1, *rb2;
    CollisionPoint p;
};

class PhysicsWorld {
public:
    glm::vec3 gravity{0, -9.81, 0};
    int NUM_OF_ITERATIONS_IMPULSE = 10;
    int NUM_OF_ITERATIONS_POSITION = 4;
    float timestep = 1 / 60.f;
    std::vector<Constraint*> constraints;

    PhysicsWorld* addConstraint(Constraint* c);

    void step(float dt, const std::vector<RigidBody*>& rb);
    std::vector<collision> getCollisionPoints(const std::vector<RigidBody *>& rb);

    void gui(std::vector<RigidBody*> rbs);

    ~PhysicsWorld();
};

#endif //TRIANGLE_PHYSICSWORLD_H
