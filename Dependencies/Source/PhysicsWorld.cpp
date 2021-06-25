//
// Created by Stefan on 22-Mar-21.
//

#include <algorithm>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

#include <unordered_map> // maybe delete later? (used in RestingForceSolver)
#include <limits>
#include <set>

#include "PhysicsWorld.h"


//PhysicsWorld *PhysicsWorld::addSolver(Solver *solver) {
//    solvers.push_back(solver);
//    return this;
//}
//PhysicsWorld *PhysicsWorld::removeSolver(Solver* solver) {
//    for (size_t i = solvers.size() - 1; i >= 0; i--)
//        solvers.erase(solvers.begin() + i);
//    return this;
//}
PhysicsWorld::~PhysicsWorld() {
//    for (auto* solv : solvers)
//        delete solv;
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

int getRbIndex(RigidBody* rb, const std::vector<RigidBody *>& rbs) {
    int i = 0;
    for (auto body : rbs) {
        if (body == rb)
            return i;
        i++;
    }
    assert("RigidBody wasn't found in rbs vector " && rb);
    return -1;
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

    std::vector<std::pair<int, int>> indices(collisionPoints.size());

    int i = 0;
    int max = 0;
    for (auto col : collisionPoints) {
        indices[i].first = getRbIndex(col.first.first, rb);
        indices[i].second = getRbIndex(col.first.second, rb);
        max = std::max(max, std::max(indices[i].first, indices[i].second));
        i++;
    }
    max++;

    if (constraints.size() < max) {
        constraints.resize(max);
        for (auto& constr : constraints)
            constr.resize(max);
    }

    for (int l = 0; l < 5; l++) {
        for (int colIndex = 0; colIndex < collisionPoints.size(); colIndex++) {
            int i, j;
            i = indices[colIndex].first;
            j = indices[colIndex].second;
            for (auto& c : constraints[i][j])
                c.solve(collisionPoints[colIndex].second, dt);
            //exit(69); // TODO delete me
        }
    }
//    for (auto* solver : solvers)
//        solver->solve(dt, collisionPoints);

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

PhysicsWorld *PhysicsWorld::addConstraint(Constraint c, std::vector<RigidBody*> &rbs) {
    size_t i, j;
    i = getRbIndex(c.rb1, rbs);
    j = getRbIndex(c.rb2, rbs);


    size_t maxValue = std::max(constraints.size(), std::max(i, j)) + 1;

    constraints.resize(maxValue);

    constraints[i].resize(maxValue);
    constraints[j].resize(maxValue);

    constraints[i][j].push_back(c);
    constraints[j][i].push_back(c);

    return this;
}

/*
void ImpulseSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>>& col) {
    for (auto c : col) {
        if (glm::length(c.first.second->velocity) > 0.1 || glm::length(c.first.first->velocity) > 0.1) { // TODO added
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


//            glm::vec3 gravity = glm::normalize(glm::vec3(0, -90, 0));
//            glm::vec3 normal = glm::normalize(c.first.first->position - c.first.second->position);
//
//            std::cout << glm::to_string(normal) << "\n";
//            std::cout << glm::length(normal + gravity) << "\n";
//            std::cout << glm::length(- normal + gravity) << "\n";



                x1 = c.second.B;
                x2 = c.second.A;

                dv1 = -2 * m2 / (m1 + m2) * glm::dot(v1 - v2, x1 - x2) / (c.second.depth * c.second.depth) * (x1 - x2);
                dv2 = -2 * m1 / (m1 + m2) * glm::dot(v2 - v1, x2 - x1) / (c.second.depth * c.second.depth) * (x2 - x1);

                //std::cout << glm::to_string((x2 - x1)) << " "
                //<< glm::dot(glm::normalize(v1 - v2), c.second.normal)
                //<< " / "
                //<< c.second.depth
                //<< " = "
                //<< glm::dot(glm::normalize(v1 - v2), c.second.normal) / c.second.depth
                //<< "\n";
                //c.first.first->velocity = v1f;
                //c.first.second->velocity = v2f;

                c.first.first->force += m1 * dv1 / dt;
                c.first.second->force += m2 * dv2 / dt;

                //std::cout << glm::to_string(v1f) << glm::to_string(v2f) << "\n";
                //std::cout << glm::to_string(c.first.first->force) << glm::to_string(c.first.second->force) << "\n\n";
            }
        }
    }
}

// TODO RestingForceSolver
void RestingForceSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>>& col) {
//    for (auto c : col) {
//        if (c.first.first->movable == false) {
//            c.first.second->force += c.first.first->mass * c.second.normal * c.second.depth;
//        } else if (c.first.second->movable == false) {
//            c.first.first->force += c.first.first->mass * c.second.normal * c.second.depth;
//        } else {
//            c.first.second->force += c.first.second->mass * c.second.normal * c.second.depth;
//            c.first.first->force  += c.first.first->mass * c.second.normal * c.second.depth;
//        }
//    }

    std::unordered_map<RigidBody*, std::pair<std::vector<RigidBody*>, float>> massUnderObj;
    std::vector<RigidBody*> objects;

    {
        std::set<RigidBody*> objSet;

        for (auto &c : col) {
            // c.second.depth < 0.01 ||
            if ((glm::length(c.first.second->velocity) < 0.1 && glm::length(c.first.first->velocity) < 0.1)) { // TODO added
                RigidBody *o1, *o2;
                o1 = c.first.first;
                o2 = c.first.second;
                if (o1->position.y < o2->position.y)
                    massUnderObj[c.first.first].first.push_back(c.first.second);
                else
                    massUnderObj[c.first.second].first.push_back(c.first.first);

                objSet.insert(c.first.first);
                objSet.insert(c.first.second);
            }
        }

        objects.insert(objects.begin(), objSet.begin(), objSet.end());
    }

    std::sort(objects.begin(), objects.end(), [](RigidBody* o1, RigidBody* o2) -> bool {
        return o1->position.y < o2->position.y;
    });


    glm::vec3 gravity = glm::vec3(0, -9.81, 0);

    for (RigidBody* o : objects) {
        o->velocity += o->force / o->mass * dt;
        o->setTransform(o->velocity * dt, glm::quat(), glm::vec3(1));
        o->velocity = glm::vec3(0);

        massUnderObj[o].second += o->mass;

        for (RigidBody* o2 : massUnderObj[o].first)
            massUnderObj[o2].second += massUnderObj[o].second;
        if (o->movable)
            o->force -= gravity * massUnderObj[o].second;

//        std::cout << glm::to_string(o->position) << "\n";
//        for (RigidBody* o2 : massUnderObj[o].first)
//            std::cout << "\t" << glm::to_string(o2->position) << " " << massUnderObj[o2].second << "\n";
    }
//    std::cout << "\n\n";

}

void PositionSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>>& col) {
}
void RotationSolver::solve(float dt, std::vector<std::pair<std::pair<RigidBody *, RigidBody *>, CollisionPoint>>& col) {
}
*/

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

