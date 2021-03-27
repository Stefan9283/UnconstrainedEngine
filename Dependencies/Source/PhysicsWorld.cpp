//
// Created by Stefan on 22-Mar-21.
//

#include "PhysicsWorld.h"

void PhysicsWorld::step(float dt, const std::vector<RigidBody *>& rb) {

    std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> collisionPoints = getCollisionPoints(rb);

    for (auto* solver : solvers)
        solver->solve(dt, collisionPoints);

    // calculate new velocities
    for (auto* r : rb) {
        r->force += r->mass * gravity;
        r->velocity += r->force / r->mass * dt;
        r->force = glm::vec3(0);

        r->position += r->velocity * dt;
    }

}

std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> PhysicsWorld::getCollisionPoints(const std::vector<RigidBody *>& rb) {
    std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> collisionPoints;

    // get all collision points
    for (auto* rb1 : rb) {
        for (auto* rb2 : rb) {
            if (rb1 == rb2) break;

            CollisionPoint p = rb1->collider->checkCollision(rb2->collider);
            if (p.hasCollision)
                collisionPoints.emplace_back(std::make_pair(rb1, rb2), p);
        }
    }

    return collisionPoints;
}

PhysicsWorld *PhysicsWorld::addSolver(Solver *solver) {
    solvers.push_back(solver);
    return this;
}
PhysicsWorld *PhysicsWorld::removeSolver(Solver* solver) {
    for (size_t i = solvers.size() - 1; i >= 0; i--)
        solvers.erase(solvers.begin() + i);
    return this;
}

PhysicsWorld::~PhysicsWorld() {
    delete[] solvers.data();
}

// TODO Solvers
void ImpulseSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> col) {
    for (auto c: col) {
    }
}
void PositionSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> col) {
}
void RotationSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> col) {
}

RigidBody::RigidBody(Collider* c, float m) {
    collider = c;
    c->parent = this;
    force = glm::vec3(0);
    mass = m;
    velocity = glm::vec3(0);
    position = glm::vec3(0);
}
void RigidBody::setTransform(glm::vec3 translation, glm::quat rotation, glm::vec3 scale) {
    collider->setTransform(translation, rotation, scale);
}
glm::mat4 RigidBody::getTransform() {
    glm::mat4 T = glm::mat4(1), R = glm::mat4(1), S = glm::mat4(1); //TODO
    T = glm::translate(glm::mat4(1), position);
    return T;

}
