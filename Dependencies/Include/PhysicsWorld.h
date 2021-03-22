//
// Created by Stefan on 22-Mar-21.
//

#ifndef TRIANGLE_PHYSICSWORLD_H
#define TRIANGLE_PHYSICSWORLD_H

#include "BoundingVolumes.h"

struct RigidBody {
    Collider* collider;
    glm::vec3 force, velocity;
    float mass;
};

class Solver {
public:
    virtual void solve(float dt, std::vector<std::pair<std::pair<RigidBody*, RigidBody*>, CollisionPoint>> col) = 0;
};

class ImpulseSolver : Solver {
public:
    void solve(float dt, std::vector<std::pair<std::pair<RigidBody*, RigidBody*>, CollisionPoint>> col) override;
};
class PositionSolver : Solver {
public:
    void solve(float dt, std::vector<std::pair<std::pair<RigidBody*, RigidBody*>, CollisionPoint>> col) override;
};
class RotationSolver : Solver {
public:
    void solve(float dt, std::vector<std::pair<std::pair<RigidBody*, RigidBody*>, CollisionPoint>> col) override;
};


class PhysicsWorld {
public:
    glm::vec3 gravity{0, -90, 0};

    std::vector<Solver*> solvers;

    PhysicsWorld* addSolver(Solver* solver);
    PhysicsWorld* removeSolver(Solver* solver);

    void step(float dt, const std::vector<RigidBody*>& rb);
    std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> getCollisionPoints(const std::vector<RigidBody *>& rb);
    
    ~PhysicsWorld();
};

#endif //TRIANGLE_PHYSICSWORLD_H
