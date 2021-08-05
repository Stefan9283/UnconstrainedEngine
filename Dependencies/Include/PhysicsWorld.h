#ifndef TRIANGLE_PHYSICSWORLD_H
#define TRIANGLE_PHYSICSWORLD_H

#include "Colliders.h"
#include "Constraint.h"
#include "RigidBody.h"
#include "BVH.h"

template <typename S, typename T>
struct std::hash<std::pair<S, T>> {
    std::size_t operator()(std::pair<S, T> const& val) const noexcept {
        size_t seed = 0;
        std::size_t h1 = std::hash<S>{}(val.first);
        std::size_t h2 = std::hash<T>{}(val.second);
        return h1 ^ (h2 << 1);
        //seed ^= h1 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        //seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        //return seed;
    }
};
class Constraint;

class PhysicsWorld {
public:
    glm::vec3 gravity{0, -9.81, 0};
    int NUM_OF_ITERATIONS_IMPULSE = 4;
    int NUM_OF_ITERATIONS_POSITION = 4;
    float timestep = 1 / 60.f;
    BVH* bvh = nullptr;

    std::unordered_map<std::pair<RigidBody*, RigidBody*>, RestingConstraint*> restingConstraints;
    std::unordered_map<std::pair<RigidBody*, RigidBody*>, std::vector<Constraint*>> constraints;

    //std::vector<Constraint*> constraints;

    PhysicsWorld(std::vector<RigidBody*>& rb);
    void addRigidBody(RigidBody* rb);


    PhysicsWorld* addConstraint(Constraint* c);

    void step(float dt, std::vector<RigidBody*>& rb);
    std::vector<CollisionPoint> getCollisionPoints(const std::vector<RigidBody *>& rb);
    void gui(std::vector<RigidBody*> rbs);

    ~PhysicsWorld();
};

#endif //TRIANGLE_PHYSICSWORLD_H
