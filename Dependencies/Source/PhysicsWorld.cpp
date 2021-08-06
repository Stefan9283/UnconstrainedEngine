#include "PhysicsWorld.h"
#include "Mesh.h"

PhysicsWorld::~PhysicsWorld() {
    delete bvh;

    for (auto& c : restingConstraints)
        delete c.second;
    for (auto& v : constraints)
        for (auto* c : v.second)
            delete c;
}

PhysicsWorld::PhysicsWorld(std::vector<RigidBody*>& rb) {
    delete bvh;
    bvh = new BVH(rb);
}
void PhysicsWorld::addRigidBody(RigidBody* rb) {
    bvh->insertRigidBody(rb);
}

void PhysicsWorld::step(float dt, std::vector<RigidBody*>& rb) {
    Timer t(true, "physics step");

    for (auto* r : rb) {
        if (r->movable && !r->sleep) {
            // add gravity
            r->force += r->mass * gravity;
            r->velocity += r->force / r->mass * dt;
            r->force = glm::vec3(0);

            BVHNode* node = bvh->removeRigidBody(r);
            node->resizeSelf();
            bvh->insertRigidBody(node);
        }
    }

    std::vector<CollisionPoint> collisionPoints;

    for (auto* r : rb)
        if (r->movable && !r->sleep)
            bvh->getCollisions(&collisionPoints, r);

    std::cout << collisionPoints.size() << " collision points\n";

    for (auto& collisionPoint : collisionPoints) {
        collisionPoint.c1->parent->sleep = false;
        collisionPoint.c2->parent->sleep = false;
    }

    //bvh->Draw(s);

    // sequential impulse solver
    for (int l = 0; l < NUM_OF_ITERATIONS_IMPULSE; l++) {
        for (auto& point : collisionPoints) {
            auto key = std::make_pair(point.c1->parent, point.c2->parent);
            if (restingConstraints.find(key) != restingConstraints.end()) {
                auto constraint = restingConstraints[key];
                constraint->solve(point, dt);
            }
        }
        for (auto& pair : constraints) {
            for (auto& c : pair.second) {
                if (c->type == constraintType::ballsocket) {
                    c->solve(dt);
                }
                else if (c->type == constraintType::slider) {
                    c->solve(dt);
                }
            }
        }
    }

    uint16_t sleep = 0;
    //   calculate final positions and rotations
    for (auto* r : rb) {
        if (r->sleep)
            sleep++;
        if (r->movable && !r->sleep) {
            glm::vec3 oldPos = r->position;
            r->position += r->velocity * dt;
            //if (glm::length(r->velocity) < 0.1f)
            //    r->sleep = true;
        } else r->velocity = glm::vec3(0);
        if (r->canBeRotated) {
            r->rotation = glm::rotate(r->rotation, r->angularVel.x * dt, glm::vec3(1, 0, 0));
            r->rotation = glm::rotate(r->rotation, r->angularVel.y * dt, glm::vec3(0, 1, 0));
            r->rotation = glm::rotate(r->rotation, r->angularVel.z * dt, glm::vec3(0, 0, 1));
        } else r->angularVel = glm::vec3(0);

        r->force = glm::vec3(0);

    }

    std::cout << sleep << "\n";

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
}
void PhysicsWorld::addConstraint(Constraint* c) {
    //std::cout << c->first->id << " " << c->second->id << "\n";
    auto key = std::make_pair(c->first, c->second);
    if (key.first->id > key.second->id) {
        std::swap(key.first, key.second);
    }
    else {
        if (c->type == constraintType::resting)
            std::swap(c->first, c->second);
    }
    switch (c->type)
    {
        case constraintType::resting:
        {
            restingConstraints[key] = (RestingConstraint*)c;
            break;
        }
        case constraintType::slider:
        case constraintType::ballsocket:
        case constraintType::generic:
        case constraintType::distance:
            constraints[key].push_back(c);
            break;

    }
}

const char* Constraints[] = {"Distance", "ContactPoint", "BallSocket", "Slider", "Generic"};
int selectedType = -1;
std::string rb1str = "";
std::string rb2str = "";
int rb1 = -1, rb2 = -1;

void PhysicsWorld::gui(std::vector<RigidBody*> rbs) {
    float g[] = { gravity.x, gravity.y, gravity.z };
    ImGui::SliderFloat3("Gravity ", g, -100, 100);
    gravity = glm::vec3(g[0], g[1], g[2]);

    int toBeRemoved = -1;

    ImGui::SliderInt("Number of iterations velocity solver", &NUM_OF_ITERATIONS_IMPULSE, 1, 10);
    ImGui::SliderInt("Number of iterations position solver", &NUM_OF_ITERATIONS_POSITION, 1, 10);
    ImGui::SliderFloat("Timestep", &timestep, 0.01, 1);

    bvh->gui();

    if (ImGui::TreeNode("Constraints")) {
        for (auto& pair : constraints) {
            int index = 0;
            for (auto c : pair.second) {
                std::string type;
                switch (c->type)
                {
                case constraintType::ballsocket:
                    type = "BallSocket";
                    break;
                case constraintType::slider:
                    type = "Slider";
                    break;
                case constraintType::distance:
                    type = "Distance";
                    break;
                case constraintType::generic:
                    type = "Generic";
                    break;
                }
                if (ImGui::TreeNode((type + " Constraint " + std::to_string(c->first->id * 100 + c->second->id * 10 + index)).c_str())) {
                    c->gui(index);
                    index++;
                    if (ImGui::Button("Remove Constraint? WIP"))
                        toBeRemoved = index;
                    ImGui::TreePop();
                }
            }
            if (toBeRemoved != -1)
                pair.second.erase(pair.second.begin() + toBeRemoved);
        }
        
        if (ImGui::TreeNode("Add Constraint?")) {
            ImGui::Combo("Type", &selectedType, Constraints,
                    IM_ARRAYSIZE(Constraints), 4);
            if (selectedType != -1) {
                    if (ImGui::BeginCombo("RigidBody1", rb1str.c_str())) {
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
                    if (ImGui::BeginCombo("RigidBody2", rb2str.c_str())) {
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
                        if (ImGui::Button("Add Constraint")) {
                            std::string type = std::string(Constraints[selectedType]);
                            Constraint* c = nullptr;
                            if (type == "Distance")
                                c = new DistanceConstraint(rbs[rb1], rbs[rb2], 0, 1);
                            if (type == "ContactPoint")
                                c = new RestingConstraint(rbs[rb1], rbs[rb2]);
                            if (type == "Generic")
                                c = new GenericConstraint(rbs[rb1], rbs[rb2]);
                            if (type == "BallSocket")
                                c = new BallSocketConstraint(rbs[rb1], rbs[rb2]);
                            if (type == "Slider")
                                c = new SliderConstraint(rbs[rb1], rbs[rb2]);
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
}

void PhysicsWorld::DrawConstraints(Shader* s) {
    for (auto& pair : constraints)
        for (auto& c : pair.second)
            if (c->render)
                c->Draw(s);
}
