#include "PhysicsWorld.h"
#include "Mesh.h"

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

#define NUM_OF_ITERATIONS_IMPULSE 2
#define NUM_OF_ITERATION_INTERPENETRATION_FIX 3
#define NUM_OF_ITERATIONS_POSITION 4

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


//    if (collisionPoints.size())
//        std::cout << "////////////// STEP /////////////\n";

    size_t max = 0;
    for (auto& col : collisionPoints)
        max = std::max(max, std::max(col.first.first, col.first.second));
    max++;

    if (constraints.size() < max) {
        constraints.resize(max);
        for (auto& constr : constraints)
            constr.resize(max);
    }

//    for (auto & collisionPoint : collisionPoints) {
//        std::cout << collisionPoint.second.toString() << "\n";
//        size_t i, j;
//        i = collisionPoint.first.first;
//        j = collisionPoint.first.second;
//        for (auto c : constraints[i][j]) {
//            std::cout << c->rb1->collider->toString();
//            std::cout << c->rb2->collider->toString();
//        }
//        std::cout << "//////////////////////////////\n";
//    }


        // sequential impulse solver
    for (int l = 0; l < NUM_OF_ITERATIONS_IMPULSE; l++)
        for (int colIndex = 0; colIndex < collisionPoints.size(); colIndex++) {
            size_t i, j;
            i = collisionPoints[colIndex].first.first;
            j = collisionPoints[colIndex].first.second;

            for (auto c : constraints[i][j])
                if (dynamic_cast<RestingConstraint*>(c))
                    c->solve(collisionPoints[colIndex].second, dt);
        }

    // TODO position solver
//    for (int l = 0; l < NUM_OF_ITERATION_INTERPENETRATION_FIX; l++) {
//        for (int colIndex = 0; colIndex < collisionPoints.size(); colIndex++) {
//            size_t i, j;
//            i = collisionPoints[colIndex].first.first;
//            j = collisionPoints[colIndex].first.second;
//
//            for (auto c : constraints[i][j]) {
//                if (dynamic_cast<DistanceConstraint*>(c))
//                    c->solve(collisionPoints[colIndex].second, dt);
//            }
//        }
//    }
//
//    for (int l = 0; l < NUM_OF_ITERATIONS_POSITION; l++)
//        for (auto line : constraints)
//            for (auto row : line)
//                for (auto c : row)
//                    if (dynamic_cast<DistanceConstraint*>(c))
//                        ((DistanceConstraint*)c)->check(dt);
//
    // calculate final velocities
    for (auto* r : rb) {
        if (r->movable) {
            r->velocity += r->force / r->mass * dt;
            r->position += r->velocity * dt;
            r->angularVel += r->torque / dt;
            r->force = glm::vec3(0);
        } else {
            r->velocity = {};
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


