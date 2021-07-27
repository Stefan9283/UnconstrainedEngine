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
    for (auto c : constraints) {
        auto *constr = dynamic_cast<RestingConstraint *>(c);
        if (constr) {
            CollisionPoint p = constr->rb1->collider->checkCollision(constr->rb2->collider);
            if (p.hasCollision) {
                collisionPoints.push_back({constr->rb1, constr->rb2, p});
            }
        }
    }
    return collisionPoints;
}

#define NUM_OF_ITERATIONS_IMPULSE 1
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
            for (auto c : constraints) {
                if (dynamic_cast<RestingConstraint *>(c)) {
                    auto* constr = dynamic_cast<RestingConstraint *>(c);
                    if (constr->rb1 == collisionPoint.rb1 && constr->rb2 == collisionPoint.rb2) {
                        CollisionPoint reversed = reverseCollisionPoint(collisionPoint.p);
                        constr->solve(reversed, dt);
                    } else if (constr->rb2 == collisionPoint.rb1 && constr->rb1 == collisionPoint.rb2) {
                        constr->solve(collisionPoint.p, dt);
                    }
                } else if (dynamic_cast<BallSocketConstraint *>(c)) {
//                    c->solve(dt);
                }

            }
        }

    //   calculate final positions and rotations
    for (auto* r : rb) {
        if (r->movable) {
            std::cout << glm::to_string(r->velocity * dt) << "\n";
            r->position += r->velocity * dt;
        }// else r->velocity = glm::vec3(0);
        if (r->canBeRotated) {
            r->rotation = glm::rotate(r->rotation, r->angularVel.x * dt, glm::vec3(1, 0, 0));
            r->rotation = glm::rotate(r->rotation, r->angularVel.y * dt, glm::vec3(0, 1, 0));
            r->rotation = glm::rotate(r->rotation, r->angularVel.z * dt, glm::vec3(0, 0, 1));
        } //else r->angularVel = glm::vec3(0);

        r->force = glm::vec3(0);

    }

//    // sequential position solver
//    for (int l = 0; l < NUM_OF_ITERATIONS_POSITION; l++)
//        for (auto c : constraints) {
//            if (dynamic_cast<DistanceConstraint *>(c))
//                ((DistanceConstraint *) c)->solve(dt);
////            else if (dynamic_cast<GenericConstraint *>(c))
////                ((GenericConstraint*)c)->solve(dt);
//            else if (dynamic_cast<BallSocketConstraint *>(c))
//                ((BallSocketConstraint*)c)->solve(dt);
//        }
//
//    // calculate final positions and rotations
//    for (auto* r : rb) {
//        if (r->movable) {
//            r->position += r->velocity * dt;
//        } else r->velocity = glm::vec3(0);
//        if (r->canBeRotated) {
//            r->rotation = glm::rotate(r->rotation, r->angularVel.x * dt, glm::vec3(1, 0, 0));
//            r->rotation = glm::rotate(r->rotation, r->angularVel.y * dt, glm::vec3(0, 1, 0));
//            r->rotation = glm::rotate(r->rotation, r->angularVel.z * dt, glm::vec3(0, 0, 1));
//        } else r->angularVel = glm::vec3(0);
//
//        r->force = glm::vec3(0);
//
//    }

    std::cout << "////////////////////////////////////\n";
}

PhysicsWorld *PhysicsWorld::addConstraint(Constraint* c) {
    constraints.push_back(c);
    return this;
}

const char* Constraints[] = {"Distance", "ContactPoint", "Generic"};
int selectedType = -1;
std::string rb1str = "";
std::string rb2str = "";
int rb1 = -1, rb2 = -1;

void PhysicsWorld::gui(std::vector<RigidBody*> rbs) {
    float g[] = {gravity.x, gravity.y, gravity.z};
    ImGui::SliderFloat3("Gravity ", g, -100, 100);
    gravity = glm::vec3(g[0], g[1], g[2]);

    int toBeRemoved = -1;


    if (ImGui::TreeNode("Constraints")) {
        for (size_t i = 0; i < constraints.size(); ++i) {
            if (ImGui::TreeNode(("Constraint" + std::to_string(i)).c_str())) {
                constraints[i]->gui(i);
                if (ImGui::Button("Remove Constraint? WIP")) {
                    toBeRemoved = i;
                } // TODO
                ImGui::TreePop();
            }
        }
        if (ImGui::TreeNode("Add Constraint?")) {
            ImGui::Combo("Type", &selectedType, Constraints,
                            IM_ARRAYSIZE(Constraints), 4);
            if (selectedType != -1) {
                if(ImGui::BeginCombo("RigidBody1", rb1str.c_str())) {
                    bool selected = false;
                    int index = 0;
                    for (auto rb : rbs) {
                        std::string s = std::to_string(index) + typeid(rb).name();
                        if (ImGui::Selectable(s.c_str(), &selected)) {
                            rb1str = s;
                            rb1 = index;
                        }
                        index++;
                    }
                    ImGui::EndCombo();
                }
                if(ImGui::BeginCombo("RigidBody2", rb2str.c_str())) {
                    bool selected = false;
                    int index = 0;
                    for (auto rb : rbs) {
                        std::string s = std::to_string(index) + typeid(rb).name();
                        if (ImGui::Selectable(s.c_str(), &selected)) {
                            rb2str = s;
                            rb2 = index;
                        }
                        index++;
                    }
                    ImGui::EndCombo();
                }

                if (rb1 != -1 && rb2 != -1)
                    if(ImGui::Button("Add Constraint")) {
                        std::string type = std::string(Constraints[selectedType]);
                        Constraint* c;
                        if (type == "Distance")
                            c = new DistanceConstraint(rbs[rb1], rbs[rb2], 0, 1);
                        if (type == "ContactPoint")
                            c = new RestingConstraint(rbs[rb1], rbs[rb2]);
                        if (type == "Generic")
                            c = new GenericConstraint(rbs[rb1], rbs[rb2]);
                        addConstraint(c);
                        rb1 = -1;
                        rb2 = -1;
                        rb1str = "";
                        rb2str = "";
                        selectedType = -1;
                    }
            }
            ImGui::TreePop();
        }
        ImGui::TreePop();
    }
    if (toBeRemoved != -1)
        constraints.erase(constraints.begin() + toBeRemoved);
}
