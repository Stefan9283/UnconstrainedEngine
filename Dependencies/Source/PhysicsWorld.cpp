#include "PhysicsWorld.h"
#include "Mesh.h"

PhysicsWorld::~PhysicsWorld() {
    std::set<Constraint*> unique_constraints;

    for (auto c : constraints)
        delete c;
}



std::vector<collision> PhysicsWorld::getCollisionPoints(const std::vector<RigidBody *>& rb) {
    std::vector<collision> collisionPoints;
    // get all collision points
    for (auto c : constraints)
        if (dynamic_cast<RestingConstraint*>(c)) {
            CollisionPoint p = c->rb1->collider->checkCollision(c->rb2->collider);
            if (p.hasCollision) {
                collisionPoints.push_back({c->rb1, c->rb2, p});
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


    std::vector<collision> collisionPoints = getCollisionPoints(rb);

    // sequential impulse solver
    for (int l = 0; l < NUM_OF_ITERATIONS_IMPULSE; l++)
        for (auto & collisionPoint : collisionPoints) {
            for (auto c : constraints)
                if (dynamic_cast<RestingConstraint*>(c)) {
                    if (c->rb1 == collisionPoint.rb1 && c->rb2 == collisionPoint.rb2) {
                        CollisionPoint reversed = reverseCollisionPoint(collisionPoint.p);
                        c->solve(reversed, dt);
                    } else if (c->rb2 == collisionPoint.rb1 && c->rb1 == collisionPoint.rb2) {
                        c->solve(collisionPoint.p, dt);
                    }
                }
        }

    //   calculate final velocities
    for (auto* r : rb) {
        if (r->movable) {
            r->position += r->velocity * dt;

            r->rotation = glm::rotate(r->rotation, r->angularVel.x * dt, glm::vec3(1, 0, 0));
            r->rotation = glm::rotate(r->rotation, r->angularVel.y * dt, glm::vec3(0, 1, 0));
            r->rotation = glm::rotate(r->rotation, r->angularVel.z * dt, glm::vec3(0, 0, 1));

            r->force = glm::vec3(0);
        }
    }

    // TODO position solver
    for (int l = 0; l < NUM_OF_ITERATIONS_POSITION; l++)
        for (auto c : constraints)
            if (dynamic_cast<DistanceConstraint*>(c))
                if (!((DistanceConstraint*)c)->check())
                    ((DistanceConstraint*)c)->solve(dt);


}

PhysicsWorld *PhysicsWorld::addConstraint(Constraint* c) {
    constraints.push_back(c);
    return this;
}

void PhysicsWorld::gui() {
    if (ImGui::TreeNode("PhysicsWorld")) {
        float g[] = {gravity.x, gravity.y, gravity.z};
        ImGui::SliderFloat3("Gravity ", g, -100, 100);
        gravity = glm::vec3(g[0], g[1], g[2]);

        for (int i = 0; i < constraints.size(); ++i) {
            if (ImGui::TreeNode(("Constraint" + std::to_string(i)).c_str())) {
                constraints[i]->gui(i);
                if (ImGui::Button("Remove Constraint? WIP")) {} // TODO
                ImGui::TreePop();
            }
        }

        if (ImGui::Button("Add Constraint? WIP")) {}  // TODO

        ImGui::TreePop();
    }
}
