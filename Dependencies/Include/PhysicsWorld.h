#ifndef TRIANGLE_PHYSICSWORLD_H
#define TRIANGLE_PHYSICSWORLD_H

#include "Colliders.h"
#include "Constraint.h"
#include "RigidBody.h"

class Constraint;


class PhysicsWorld {
public:
    glm::vec3 gravity{0, -9.81, 0};

    std::vector<std::vector<std::vector<Constraint*>>> constraints;

    PhysicsWorld* addConstraint(Constraint* c, std::vector<RigidBody*> &rbs);

    void step(float dt, const std::vector<RigidBody*>& rb);
    std::vector<std::pair<std::pair<size_t, size_t>, CollisionPoint>> getCollisionPoints(const std::vector<RigidBody *>& rb);
    
    ~PhysicsWorld();
};

#endif //TRIANGLE_PHYSICSWORLD_H
