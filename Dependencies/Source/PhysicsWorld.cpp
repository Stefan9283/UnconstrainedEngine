//
// Created by Stefan on 22-Mar-21.
//

#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include "PhysicsWorld.h"



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
    for (auto* solv : solvers)
        delete solv;
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
void PhysicsWorld::step(float dt, const std::vector<RigidBody *>& rb) {



    for (auto* r : rb) {
        if (r->movable) {
            // add gravity
            r->force += r->mass * gravity;
            r->velocity += r->force / r->mass * dt;
            r->force = glm::vec3(0);
        }
    }

    std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> collisionPoints = getCollisionPoints(rb);

    for (auto* solver : solvers)
        solver->solve(dt, collisionPoints);

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


void ImpulseSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> col) {

    for (auto c : col) {
        glm::vec3 reflected1, reflected2;

        reflected1 = glm::reflect(c.first.first->velocity, c.second.normal);
        reflected2 = glm::reflect(c.first.second->velocity, c.second.normal);

        if (c.first.first->movable == false) {
            c.first.second->force += c.first.second->mass * reflected2 / dt;
        } else if (c.first.second->movable == false) {
            c.first.first->force += c.first.first->mass * reflected1 / dt;
        } else {
            glm::vec3 dv1, dv2, v1, v2, x1, x2;

            float m1, m2;
            m1 = c.first.first->mass;
            m2 = c.first.second->mass;

            v1 = c.first.first->velocity;
            v2 = c.first.second->velocity;

            x1 = c.second.B;
            x2 = c.second.A;

            dv1 = - 2 * m2 / (m1 + m2) * glm::dot(v1 - v2, x1 - x2) / (c.second.depth * c.second.depth) * (x1 - x2);
            dv2 = - 2 * m1 / (m1 + m2) * glm::dot(v2 - v1, x2 - x1) / (c.second.depth * c.second.depth) * (x2 - x1);

            //std::cout << glm::to_string((x2 - x1)) << " "
            //<< glm::dot(glm::normalize(v1 - v2), c.second.normal)
            //<< " / "
            //<< c.second.depth
            //<< " = "
            //<< glm::dot(glm::normalize(v1 - v2), c.second.normal) / c.second.depth
            //<< "\n";
            //c.first.first->velocity = v1f;
            //c.first.second->velocity = v2f;

            c.first.first->force  += m1 * dv1 / dt;
            c.first.second->force += m2 * dv2 / dt;

            //std::cout << glm::to_string(v1f) << glm::to_string(v2f) << "\n";
            //std::cout << glm::to_string(c.first.first->force) << glm::to_string(c.first.second->force) << "\n\n";
        }
    }
}
// TODO Solvers
void RestingForceSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>> col) {
    for (auto c : col) {
        if (c.first.first->movable == false) {
            c.first.second->force += c.first.first->mass * c.second.normal * c.second.depth;
        } else if (c.first.second->movable == false) {
            c.first.first->force += c.first.first->mass * c.second.normal * c.second.depth;
        } else {
            c.first.second->force += c.first.second->mass * c.second.normal * c.second.depth;
            c.first.first->force  += c.first.first->mass * c.second.normal * c.second.depth;
        }
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

