//
// Created by Stefan on 22-Mar-21.
//

#include "PhysicsWorld.h"

PhysicsWorld::~PhysicsWorld() {
    std::set<Constraint*> unique_constraints;

    for (auto& row : constraints)
        for (auto &constrVec : row)
            for (auto *c : constrVec)
                unique_constraints.insert(c);

    for (auto c : unique_constraints)
        delete c;
}
std::vector<std::pair<std::pair<size_t, size_t>, CollisionPoint>> PhysicsWorld::getCollisionPoints(const std::vector<RigidBody *>& rb) {
    std::vector<std::pair<std::pair<size_t, size_t>, CollisionPoint>> collisionPoints;
    // get all collision points
    for (size_t i = 0; i < rb.size(); i++) {
        for (size_t j = i + 1; j < rb.size(); j++) {
            CollisionPoint p = rb[i]->collider->checkCollision(rb[j]->collider);
            if (p.hasCollision)
                collisionPoints.emplace_back(std::make_pair(i, j), p);
        }
    }

    return collisionPoints;
}


glm::vec3 max_velocity = glm::vec3(0); // TODO delete me after fixing aabb sphere collision

#define NUM_OF_ITERATIONS 1

void PhysicsWorld::step(float dt, const std::vector<RigidBody *>& rb) {
    for (auto* r : rb) {
        if (r->movable) {
            // add gravity
            r->force += r->mass * gravity;
            r->velocity += r->force / r->mass * dt;
            r->force = glm::vec3(0);
        }
    }

    std::vector<std::pair<std::pair<size_t, size_t>, CollisionPoint>> collisionPoints = getCollisionPoints(rb);


    size_t max = 0;
    for (auto col : collisionPoints)
        max = std::max(max, std::max(col.first.first, col.first.second));
    max++;

    if (constraints.size() < max) {
        constraints.resize(max);
        for (auto& constr : constraints)
            constr.resize(max);
    }


    for (int l = 0; l < NUM_OF_ITERATIONS; l++) {
        for (int colIndex = 0; colIndex < collisionPoints.size(); colIndex++) {
            size_t i, j;
            i = collisionPoints[colIndex].first.first;
            j = collisionPoints[colIndex].first.second;

            for (auto c : constraints[i][j])
                c->solve(collisionPoints[colIndex].second, dt/NUM_OF_ITERATIONS);
        }
    }
    // calculate final velocities
    for (auto* r : rb) {
        if (r->movable) {
            r->velocity += r->force / r->mass * dt;
            r->setTransform(r->velocity * dt, glm::quat(), glm::vec3(1));
            r->angularVel += r->torque / dt;
            r->force = glm::vec3(0);
        }
    }

}

int getRbIndex(RigidBody* rb, const std::vector<RigidBody*>& rbs) {
    int i = 0;
    for (auto body : rbs) {
        if (body == rb)
            return i;
        i++;
    }
    assert("RigidBody wasn't found in rbs vector " && rb);
    return -1;
}
PhysicsWorld *PhysicsWorld::addConstraint(Constraint* c, std::vector<RigidBody*> &rbs) {
    size_t i, j;
    i = getRbIndex(c->rb1, rbs);
    j = getRbIndex(c->rb2, rbs);

    size_t maxValue = std::max(constraints.size(), std::max(i, j)) + 1;

    constraints.resize(maxValue);

    constraints[i].resize(maxValue);
    constraints[j].resize(maxValue);

    constraints[i][j].push_back(c);
    constraints[j][i].push_back(c);

    return this;
}

RigidBody::RigidBody(Collider* c, float m) {
    collider = c;
    c->parent = this;

    force = glm::vec3(0);
    torque = glm::vec3(0);
    velocity = glm::vec3(0);
    angularVel = glm::vec3(0);

    mass = m;
    position = glm::vec3(0);
}
void RigidBody::setTransform(glm::vec3 translation, glm::quat rotation, glm::vec3 scale) {
    collider->setTransform(translation, rotation, scale);
    position += translation;
}
glm::mat4 RigidBody::getTransform() {
    glm::mat4 T = glm::mat4(1), R = glm::mat4(1), S = glm::mat4(1); //TODO
    T = glm::translate(glm::mat4(1), position);
    return T;
}

void RigidBody::moveObject(glm::vec3 pos) {
    if (movable)
        position += pos;
}

