#define GLM_FORCE_SWIZZLE 
#define GLM_FORCE_INLINE 
#define USE_IMGUI_API
#define STBI_IMAGE_IMPLEMENTATION
//#define GLFW_INCLUDE_NONE // it's already defined somewhere

#include "Common.h"

#include <PhysicsWorld.h>
#include "ObjLoad.h"
#include "Camera.h"
#include "Colliders.h"
#include "RigidBody.h"
#include "Octree.h"
#include "Mesh.h"
#include "BVH.h"

static void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}
void resize(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}
extern ImGuizmo::OPERATION mCurrentGizmoOperation;




bool continuouslyChecking = false;
GLFWwindow* window = nullptr;
Shader* s = nullptr;
Camera* c = nullptr;
Ray* r = nullptr;
Mesh* crosshair = nullptr;

void createCrosshair() {
    crosshair = new Mesh();
    crosshair->vertices.push_back(Vertex{ glm::vec3(0,  1,  -25.0f) });
    crosshair->vertices.push_back(Vertex{ glm::vec3(0, -1,  -25.0f) });
    crosshair->vertices.push_back(Vertex{ glm::vec3(1,  0,  -25.0f) });
    crosshair->vertices.push_back(Vertex{ glm::vec3(-1, 0,  -25.0f) });
    crosshair->indices.push_back(0);
    crosshair->indices.push_back(1);
    crosshair->indices.push_back(0);
    crosshair->indices.push_back(2);
    crosshair->indices.push_back(3);
    crosshair->indices.push_back(2);
    crosshair->wireframeON = true;
    crosshair->solidON = true;
    crosshair->prepare();
}

void testBasicCollision(std::vector<Mesh*> meshes) {
    std::vector<bool> collision;
    for (size_t i = 0; i < meshes.size() ; i++)
        collision.push_back(false);
    collision.push_back(false);

    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        s->setFloat("flatColorsON", 0);
        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());
        s->setVec3("cameraPos", c->position);
        c->update_proj(window);
        c->Move(window);

        ImGuizmo::BeginFrame();

        float* camviewvec = (float*)glm::value_ptr(c->view);
        float* projvec = (float*)glm::value_ptr(c->proj);
        float* identity = (float*)glm::value_ptr(glm::mat4(1));
        //ImGuizmo::DrawGrid(camviewvec, projvec, identity, 100.f);

        s->setFloat("flatColorsON", 0);
        for (int i = 0; i < collision.size() - 1 ; ++i) {
            //EditTransform(c, &c->view, &c->proj, meshes[i]);
            meshes[i]->gui(i);
            if (collision[i])
                s->setVec3("color", glm::vec3(1,0,0));
            else s->setVec3("color", glm::vec3(0,1,0));
            meshes[i]->Draw(s);
        }

        s->setFloat("flatColorsON", 1);
        if (collision[collision.size() - 1])
            s->setVec3("color", glm::vec3(1,0,0));
        else s->setVec3("color", glm::vec3(0,1,0));
        if (r) r->Draw(s);


        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glm::mat4 proj = *c->getprojmatrix();
        glm::mat4 model = glm::mat4(1);
        s->setMat4("proj", &proj);
        s->setMat4("view", &model);
        //model = glm::scale(model, glm::vec3(0.0001f));
        model = glm::translate(model, c->goFront);
        s->setMat4("model", &model);
        glViewport(0, 0, display_w, display_h);

        ImGui::SliderFloat("cam speed", &c->speed, 0.001f, 0.5f);

        s->setFloat("flatColorsON", 1);
        crosshair->Draw(s);

        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::Checkbox("Continuously checking for collisions", &continuouslyChecking);
        if(ImGui::Button("Check collisions" ) || continuouslyChecking) {

            //if (hit_or_nah.size()) {
            //    delete hit_or_nah[0];
            //    delete hit_or_nah[1];
            //}

            //hit_or_nah = wasMeshHit(meshes[0]->rigidbody, r);


            for (auto && l : collision) {
                l = false;
            }
            for(size_t i=0; i<meshes.size(); i++) {
                Mesh* m = meshes[i];
                for (size_t j=0; j<meshes.size(); j++) {
                    Mesh* m2 = meshes[j];
                    if (m != m2)
                        if(m->rigidbody->collider->checkCollision(m2->rigidbody->collider).hasCollision) {
                            collision[i] = true;
                            collision[j] = true;
                        }
                }
            }
            if(r) {
                for (size_t i=0; i<meshes.size(); i++) {
                    Mesh* m = meshes[i];
                    if (m->rigidbody->collider->checkCollision(r).hasCollision) {
                        collision[i] = true;
                        collision[collision.size() - 1] = true;
                    }
                }
            }
        }

        Ray* tmp_r = Ray::generateRay(window, c);
        if (tmp_r) {
            delete r;
            r = tmp_r;
            collision[collision.size() - 1] = false;
        }
        if (r) {
            float start[4] = {
                    r->origin.x,
                    r->origin.y,
                    r->origin.z,
                    1
            },
                    end[4] = {
                    r->direction.x,
                    r->direction.y,
                    r->direction.z,
                    1
            },
                    length = r->length;


            ImGui::SliderFloat3("origin", start, -10, 10);
            ImGui::SliderFloat3("direction", end, -1, 1);
            ImGui::SliderFloat("length", &length, 0, 10);
            delete r;
            r = new Ray(glm::vec3(start[0], start[1], start[2]), glm::normalize(glm::vec3(end[0], end[1], end[2])), length, true);
        }

        if(ImGui::Button("Toggle bounding volumes all" )) {
            for (auto m : meshes) {
                m->boundingBoxON = !m->boundingBoxON;
            }
        }
        if(ImGui::Button("Toggle solid all" )) {
            for (auto m : meshes) {
                m->solidON = !m->solidON;
            }
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}
void testRayMeshIntersection(Mesh* mesh) {
    bool rayCollision = false;
    std::vector<Mesh*> hit_or_nah;
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        s->setFloat("flatColorsON", 0);
        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());
        s->setVec3("cameraPos", c->position);
        c->update_proj(window);
        c->Move(window);

        ImGuizmo::BeginFrame();

        float* camviewvec = (float*)glm::value_ptr(c->view);
        float* projvec = (float*)glm::value_ptr(c->proj);
        float* identity = (float*)glm::value_ptr(glm::mat4(1));
        //ImGuizmo::DrawGrid(camviewvec, projvec, identity, 100.f);

        s->setFloat("flatColorsON", 0);

        s->setFloat("flatColorsON", 1);

        if (r) r->Draw(s);

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glm::mat4 proj = *c->getprojmatrix();
        glm::mat4 model = glm::mat4(1);
        s->setMat4("proj", &proj);
        s->setMat4("view", &model);
        model = glm::translate(model, c->goFront);
        s->setMat4("model", &model);
        glViewport(0, 0, display_w, display_h);

        ImGui::SliderFloat("cam speed", &c->speed, 0.001f, 0.5f);

        s->setVec3("color", glm::vec3(0, 1, 0));

        s->setFloat("flatColorsON", 1);
        crosshair->Draw(s);

        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());

        mesh->gui(0);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::Checkbox("Continuously checking for collisions", &continuouslyChecking);
        if(ImGui::Button("Check collisions" ) || continuouslyChecking) {
            if (!hit_or_nah.empty()) {
                delete hit_or_nah[0];
                delete hit_or_nah[1];
            }

            if (r)
                hit_or_nah = wasMeshHit(mesh->rigidbody->collider, r);
            else std::cout << "You need to generate a ray first\n";
        }

        s->setFloat("flatColorsON", 0);
        mesh->Draw(s);

        if (hit_or_nah.size()) {
            s->setVec3("color", glm::vec3(1,0,0));
            hit_or_nah[0]->Draw(s);
            s->setVec3("color", glm::vec3(0,1,0));
            hit_or_nah[1]->Draw(s);
        }


        Ray* tmp_r = Ray::generateRay(window, c);
        if (tmp_r) {
            delete r;
            r = tmp_r;
            rayCollision = false;
        }
        if (r) {
            float start[4] = {
                    r->origin.x,
                    r->origin.y,
                    r->origin.z,
                    1
            },
                    end[4] = {
                    r->direction.x,
                    r->direction.y,
                    r->direction.z,
                    1
            },
                    length = r->length;


            ImGui::SliderFloat3("origin", start, -10, 10);
            ImGui::SliderFloat3("direction", end, -1, 1);
            ImGui::SliderFloat("length", &length, 0, 10);
            delete r;
            r = new Ray(glm::vec3(start[0], start[1], start[2]), glm::normalize(glm::vec3(end[0], end[1], end[2])), length, true);
        }

        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwSwapBuffers(window);

        glfwPollEvents();
    }
}
void testOctree(Mesh* mesh) {
    Octree* octree = nullptr;

    int max_depth = 1;
    int draw_until_this_level = 1;

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuizmo::BeginFrame();

        s->setFloat("flatColorsON", 0);
        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());
        glm::mat4 model = glm::mat4(1);
        s->setMat4("model", &model);
        s->setVec3("cameraPos", c->position);
        c->update_proj(window);
        c->Move(window);
        s->setVec3("color", glm::vec3(1,0,0));

        if(ImGui::Button("Build Octree")) {
            delete octree;
            octree = new Octree(mesh, max_depth);
        }

        if (ImGui::Button("Solid Body"))
            mesh->solidON = !mesh->solidON;
        mesh->Draw(s);

        ImGui::SliderInt("Drawn level", &draw_until_this_level, 0, max_depth);
        ImGui::SliderInt("Octree Depth", &max_depth, 0, 10);

        s->setVec3("color", glm::vec3(0,1,0));
        if (octree) {
            octree->Draw(draw_until_this_level, s);
        }
        if (glfwGetKey(window, GLFW_KEY_ESCAPE))
            break;

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        resize(window, display_w, display_h);
    }
    delete octree;
}

#define TIMESTEP 1 / 60.f

void testPhysics(std::vector<RigidBody*> rbs) {
    PhysicsWorld physicsWorld(rbs);
    
    //for (size_t i = 0; i < rbs.size(); i += 2) {
    //    physicsWorld.addConstraint(new FixedDistanceConstraint(rbs[i], rbs[i + 1], 5));
    //}

//    physicsWorld.addConstraint(new RestingConstraint(rbs[0], rbs[2]));
//    physicsWorld.addConstraint(new RestingConstraint(rbs[1], rbs[0]));
//    physicsWorld.addConstraint(new FixedDistanceConstraint(rbs[1], rbs[2], 6));
    physicsWorld.addConstraint(new SliderConstraint(rbs[2], rbs[1]));
//    physicsWorld.addConstraint(new BallSocketConstraint(rbs[2], rbs[1]));
//    physicsWorld.addConstraint(new HingeConstraint(rbs[1], rbs[2]));


//    auto* generic = new GenericConstraint(rbs[0], rbs[1]);
//    generic->linear[0].toggle(0, 5);
//    generic->linear[1].toggle(0, 5);
//    generic->angular[0].toggle(0, 0);
//    generic->linear[2].toggle(0, 5);
//    physicsWorld.addConstraint(generic);
//    rbs[0]->angularVel = glm::vec3(1, 0, 0);
//    auto* dist = new DistanceConstraint(rbs[0], rbs[1], 0, 10);
//    physicsWorld.addConstraint(dist);
      for (auto r1 : rbs)
          for (auto r2 : rbs)
              if (r1 != r2) {
                  physicsWorld.addConstraint(new RestingConstraint(r1, r2));
              } else break;

    bool runWithPhysics = false;

    std::future<void> phy;

    while (!glfwWindowShouldClose(window))
    {
        if (runWithPhysics) {
            phy = std::async(&PhysicsWorld::step, &physicsWorld, TIMESTEP, rbs);
        }
        bool runWithPhysicsOld = runWithPhysics;

        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        bool doOneStep = false;

        if (ImGui::BeginTabBar("Bar")) {
            if (ImGui::BeginTabItem("Physics")) {
                physicsWorld.gui(rbs);
                doOneStep = ImGui::Button("Do one simulation step");
                if (!runWithPhysics && doOneStep) {
                    phy = std::async(&PhysicsWorld::step, &physicsWorld, TIMESTEP, rbs);
                }
                ImGui::Checkbox("run with physics", &runWithPhysics);
                ImGui::EndTabItem();
            }
            
            if (ImGui::BeginTabItem("RigidBodies")) {
                int index = 0;
                for (auto rb : rbs)
                    rb->gui(++index);
                doOneStep = ImGui::Button("Do one simulation step");
                if (!runWithPhysics && doOneStep) {
                    phy = std::async(&PhysicsWorld::step, &physicsWorld, TIMESTEP, rbs);
                }
                ImGui::Checkbox("run with physics", &runWithPhysics);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        
       
        c->Move(window);

        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());
        s->setVec3("cameraPos", c->position);

        for (auto* rb : rbs) {
            s->setVec3("color", glm::vec3(0.0, 0.5, 0.1));
            rb->collider->Draw(s);
        }

        physicsWorld.bvh->Draw(s);
        physicsWorld.DrawConstraints(s);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        resize(window, display_w, display_h);
        if ((runWithPhysicsOld && runWithPhysics) || (!runWithPhysics && doOneStep)) {
            phy.get();
        }
    }
}
void testBasicCollisionWithPoints(Mesh* m1, Mesh* m2) {
    CollisionPoint p{};

    Mesh *pointA = nullptr, *pointB = nullptr;
    
    m1->solidON = false;
    m2->solidON = false;

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        s->setFloat("flatColorsON", 0);
        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());
        s->setVec3("cameraPos", c->position);
        c->update_proj(window);
        c->Move(window);

        ImGuizmo::BeginFrame();

        auto* camviewvec = (float*)glm::value_ptr(c->view);
        auto* projvec = (float*)glm::value_ptr(c->proj);
        float* identity = (float*)glm::value_ptr(glm::mat4(1));
        //ImGuizmo::DrawGrid(camviewvec, projvec, identity, 100.f);

        s->setFloat("flatColorsON", 0);

        if (p.hasCollision) s->setVec3("color", glm::vec3(1,0,0));
        else s->setVec3("color", glm::vec3(0,1,0));

        m1->gui(1);
        m2->gui(2);

        if (pointA) {
            s->setVec3("color", glm::vec3(252, 223, 3) / 255.0f);
            m1->Draw(s);
            pointA->Draw(s);
        } else {
            m1->Draw(s);
        }
        if (pointB) {
            s->setVec3("color", glm::vec3(152, 3, 252)/255.0f);
            m2->Draw(s);
            pointB->Draw(s);
        } else {
            m2->Draw(s);
        }

        if(m1->rigidbody->collider->type == colliderType::ray) {
            Ray* tmp_r = Ray::generateRay(window, c);
            if (tmp_r) {
                delete m1->rigidbody->collider;
                m1->rigidbody->collider = tmp_r;
            }
        }
        if(m2->rigidbody->collider->type == colliderType::ray) {
            Ray* tmp_r = Ray::generateRay(window, c);
            if (tmp_r) {
                delete m2->rigidbody->collider;
                m2->rigidbody->collider = tmp_r;
            }
        }

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glm::mat4 proj = *c->getprojmatrix();
        glm::mat4 model = glm::mat4(1);
        s->setMat4("proj", &proj);
        s->setMat4("view", &model);

        model = glm::translate(model, c->goFront);
        s->setMat4("model", &model);
        glViewport(0, 0, display_w, display_h);

        ImGui::SliderFloat("cam speed", &c->speed, 0.001f, 0.5f);

        //s->setFloat("flatColorsON", 1);
        //crosshair->Draw(s);

        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());


        if (pointA) {
            s->setVec3("color", glm::vec3(255, 0, 0) / 255.0f);

            std::string s = "yellow ";
            s.append(glm::to_string(pointA->localTransform.tr));
            ImGui::Text(s.c_str());

            s = "purple ";
            s.append(glm::to_string(pointB->localTransform.tr));
            ImGui::Text(s.c_str());
        }


        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::Checkbox("Continuously checking for collisions", &continuouslyChecking);
        if(ImGui::Button("Check collisions" ) || continuouslyChecking) {
            p = m1->rigidbody->collider->checkCollision(m2->rigidbody->collider);
            delete pointA;
            pointA = nullptr;
            
            delete pointB;
            pointB = nullptr;
            
            if (p.hasCollision) {
                pointA = readObj("3D/Sphere.obj");
                pointA->localTransform.sc = glm::vec3(0.1f);
                pointA->localTransform.rot = glm::quat();
                pointA->localTransform.tr = p.A;

                pointB = readObj("3D/Sphere.obj");
                pointB->localTransform.sc = glm::vec3(0.1f);
                pointB->localTransform.rot = glm::quat();
                pointB->localTransform.tr = p.B;
            }
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    delete pointA;
    delete pointB;
}
void testBVH(std::vector<RigidBody*> rbs) {
    BVH* tree;
    {
        Timer t(true, "creating BVH");
        tree = new BVH(rbs);
    }
    tree->root->asciiprint();

    std::vector<bool> hasCollision(rbs.size());
    hasCollision.assign(hasCollision.size(), false);
    bool continuouslyCheckForCollisions{};
    int reinsertThisRB = 0;

    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        c->Move(window);

        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());
        s->setVec3("cameraPos", c->position);

        ImGui::SliderInt("Reinsert this node", &reinsertThisRB, 0, rbs.size() - 1);
        if (ImGui::Button("reinsert rb")) {
            BVHNode* n = tree->removeRigidBody(rbs[reinsertThisRB]);
            n->resizeSelf();
            tree->insertRigidBody(n);
        }

        if (ImGui::Button("Rebuild")) {
            Timer t(true, "creating BVH");
            
            delete tree;
            tree = new BVH(rbs);

            //for (size_t i = 0; i < 100; i++) {
            //    tree->removeRigidBody(rbs[i]);
            //}
            //for (size_t i = 0; i < 100; i++) {
            //    tree->insertRigidBody(rbs[i]);
            //}
        }
        
        std::vector<CollisionPoint> collisionPoints;

        ImGui::Checkbox("Continuously Check For Collisions", &continuouslyCheckForCollisions);

        if (ImGui::Button("Get Collisions") || continuouslyCheckForCollisions)
        {
            collisionPoints.clear();
            hasCollision.assign(hasCollision.size(), false);

            {
                //Timer t(true, "generating collision points");
                for (size_t i = 0; i < rbs.size(); i++) {
                    size_t before = collisionPoints.size();
                    tree->getCollisions(&collisionPoints, rbs[i]);
                    size_t after = collisionPoints.size();
                    if (after - before != 0) {
                        hasCollision[i] = true;
                    }
                }
            }
            //std::cout << collisionPoints.size() << " collision points\n";
        }
        
        {
            //Timer t(true, "drawing everything");
            for (size_t i = 0; i < rbs.size(); i++) {
                if (hasCollision[i])
                    s->setVec3("color", glm::vec3(1, 0, 0));
                else
                    s->setVec3("color", glm::vec3(0, 1, 0));
                rbs[i]->collider->Draw(s);
            }

            int i = 0;
            if (ImGui::TreeNode("RigidBodies")) {
                for each (auto rb in rbs) {
                    rb->gui(i);
                    i++;
                }
                ImGui::TreePop();
            }
        }

        if (tree) {
            tree->gui();
            tree->Draw(s);
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        resize(window, display_w, display_h);
    }
    delete tree;
}

int main() {
#pragma region prepare OGL
    if (!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);

    window = glfwCreateWindow(2040, 1040, "Physics Engine", nullptr, nullptr);

    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetKeyCallback(window, key_callback);
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glfwSwapInterval(1);

    glDepthMask(GL_TRUE);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init( "#version 330");
    glEnable(GL_DEPTH_TEST);

#pragma endregion
    s = new Shader("Dependencies/Shaders/simpleVert.glsl", "Dependencies/Shaders/simpleFrag.glsl");
    s->bind();
    c = new Camera(window);
    glClearColor(0, 102 / 255.f, 102 / 255.f, 1);
#pragma region meshes
    std::vector<Mesh*> meshes;


    Mesh* cube = readObj("3D/Box.obj");
    cube->addBody(new RigidBody(new AABB(cube)));
    meshes.push_back(cube);

   Mesh* sphere = readObj("3D/Sphere.obj");
   sphere->addBody(new RigidBody(new Sphere(sphere)));
   meshes.push_back(sphere);
  
   Mesh* cube2 = readObj("3D/Box.obj");
   cube2->addBody(new RigidBody(new AABB(cube2)));
   cube2->rigidbody->position = glm::vec3(0.7, 1.2, 0);
   meshes.push_back(cube2);
  
   Mesh* sphere2 = readObj("3D/Sphere.obj");
   sphere2->addBody(new RigidBody(new Sphere(sphere2)));
   sphere2->rigidbody->position = glm::vec3(0, 1.5f, 0);
   meshes.push_back(sphere2);
  
   Mesh* Yen = readObj("3D/Yen.obj");
   Yen->addBody(new RigidBody(new Capsule(4.07f, 1.27f))); //Capsule::generateCapsule(Yen);
   Yen->rigidbody->position = glm::vec3(0, 4.6f, 0);
   //Yen->addBody(new RigidBody(new Capsule(sphere))); //Capsule::generateCapsule(Yen);
   meshes.push_back(Yen);
  
   Mesh* Mercy = readObj("3D/Mercy2.obj");
   Mercy->addBody(new RigidBody(new TriangleMesh(Mercy)));
   meshes.push_back(Mercy);

    Mesh* Triangle = readObj("3D/Triangle.obj");
    Triangle->addBody(new RigidBody(new TriangleMesh(Triangle)));
    meshes.push_back(Triangle);


    Mesh* cube3 = readObj("3D/Box.obj");
    cube3->addBody(new RigidBody(new OBB(1,1,1)));
    cube3->rigidbody->rotation = glm::quat(0.855f, 0.157f, -0.494f, 0);
    meshes.push_back(cube3);

    r = new Ray(glm::vec3(-0.117, 1.522, 0.281), glm::vec3(0.143, -0.057, -0.988), 100, true); // nullptr;

    createCrosshair();
#pragma endregion
    //testOctree(Mercy);
    
    // testBasicCollision(meshes);

    // TODO FIX IT
//    testRayMeshIntersection(Triangle);

    //testBasicCollisionWithPoints(&ray, Mercy); // Ray Sphere
    // testBasicCollisionWithPoints(&ray, cube); // Ray AABB

//     testBasicCollisionWithPoints(sphere, cube); // AABB Sphere
    // testBasicCollisionWithPoints(sphere, sphere2); // Sphere Sphere
    // testBasicCollisionWithPoints(cube, cube2); // AABB AABB
//     testBasicCollisionWithPoints(sphere, Yen); // Capsule Sphere
//     testBasicCollisionWithPoints(Yen, cube); // AABB Capsule

    // TODO Ovidiu OBB collisions
    //testBasicCollisionWithPoints(cube3, cube); // AABB OBB
    // testBasicCollisionWithPoints(sphere, cube3); // Sphere OBB

    std::vector<RigidBody*> rbs;
    /*

    for (int i = -5; i < 5; i++) {
        RigidBody* top = new RigidBody(new Sphere);
        top->movable = false;
        top->position = glm::vec3(i * 2, 10, 0);
        RigidBody* bottom= new RigidBody(new Sphere);
        bottom->position = glm::vec3(2 * i, 5, 0);
        rbs.push_back(top);
        rbs.push_back(bottom);
    }

    */
    auto* cubePhy = new RigidBody(new AABB(2, 100, 100));
    cubePhy->position = glm::vec3(0, -10, 0);
    auto* spherePhy = new RigidBody(new Sphere());
    spherePhy->movable = true;


    if (false) {
        spherePhy->movable = false;
        rbs.push_back(spherePhy);
    } else {
        cubePhy->movable = false;
        rbs.push_back(cubePhy);
    }

    {
        rbs.push_back(new RigidBody(new Sphere()));
        rbs.push_back(new RigidBody(new Sphere()));
        rbs[1]->movable = false;
        //rbs[2]->movable = false;
        rbs[1]->position = glm::vec3(-2, 3, 1);
        rbs[2]->position = glm::vec3(2, 3, -1);
    }

    /*
    {
        RigidBody* tmp;
        for (int i = 0; i < 3; i++)
            for (int j = 1; j < 3; j++) {
                tmp = new RigidBody(new Sphere(glm::vec3(0), 1, true));
                tmp->position = glm::vec3(i * 3, j * 2, 0);
                if (j == 0)
                    tmp->movable = false;
                rbs.push_back(tmp);
            }

        rbs[0]->position = glm::vec3(5, 0, 0);
        rbs[1]->position = glm::vec3(0, 3, 0);
    }
    for (int j = -5; j < 5; j++) {
        for (int i = -55; i < 55; ++i) {
            RigidBody* tmp = nullptr;
            switch (std::abs(i) % 2) {
            case 0:
                tmp = new RigidBody(new Sphere(glm::vec3(0), 1), 1);
                break;
            case 1:
                tmp = new RigidBody(new AABB(3, 3, 3), 1);
                break;
            case 2:
                tmp = new RigidBody(new Capsule(), 1);
                break;
            }
            tmp->position = glm::vec3((i * 2) % 50 + 2, 3 + std::abs(i + j) , (j * 2) % 50 + 2);
            rbs.push_back(tmp);
        }
    }
    */

    //testBVH(rbs);
    testPhysics(rbs);
    for (auto* r : rbs) {
        delete r;
    }
#pragma region cleanUp

    s->unbind();
    for(Mesh* m : meshes)
        delete m;

    delete crosshair;
    delete s;
    delete c;

    glfwTerminate();

    return 0;
#pragma endregion
}
