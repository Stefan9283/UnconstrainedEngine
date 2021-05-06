//
// Created by Stefan on 22-Mar-21.
//

#ifndef TRIANGLE_PHYSICSWORLD_H
#define TRIANGLE_PHYSICSWORLD_H

#include "BoundingVolumes.h"

#define IMMUTABLE_MASS 9999

class RigidBody {
public:
    Collider* collider;
    glm::vec3
            force, velocity,
            torque, angularVel,
            position;
    float mass;
    bool movable = true;
    float drag = 0.01f;
    RigidBody(Collider* c,  float m = 1);
    void setTransform(glm::vec3 translation, glm::quat rotation, glm::vec3 scale);
    glm::mat4 getTransform();
};

class Solver {
public:
    virtual void solve(float dt, std::vector<std::pair<std::pair<RigidBody*, RigidBody*>, CollisionPoint>> col) = 0;
};

class ImpulseSolver : public Solver {
public:
    void solve(float dt, std::vector<std::pair<std::pair<RigidBody*, RigidBody*>, CollisionPoint>> col) override;
};

class RestingForceSolver : public Solver {
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
