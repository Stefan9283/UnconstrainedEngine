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
void testPhysics(std::vector<RigidBody*> rbs) {
    PhysicsWorld physicsWorld;

    physicsWorld.addConstraint(new RestingConstraint(rbs[0], rbs[2]));
    physicsWorld.addConstraint(new RestingConstraint(rbs[1], rbs[2]));
    physicsWorld.addConstraint(new SliderConstraint(rbs[0], rbs[1]));
//    physicsWorld.addConstraint(new BallSocketConstraint(rbs[0], rbs[1]));


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
                auto* contact = new RestingConstraint(r1, r2);
                physicsWorld.addConstraint(contact);
            } else break;


    bool runWithPhysics = false;

    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        s->setVec3("cameraPos", c->position);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        bool doOneStep = false;
        if (ImGui::BeginTabBar("Bar")) {
            if (ImGui::BeginTabItem("Physics")) {
                physicsWorld.gui(rbs);
                doOneStep = ImGui::Button("Do one simulation step");
                ImGui::Checkbox("run with physics", &runWithPhysics);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("RigidBodies")) {
                int index = 0;
                for (auto rb : rbs)
                    rb->gui(++index);
                doOneStep = ImGui::Button("Do one simulation step");
                ImGui::Checkbox("run with physics", &runWithPhysics);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        {
            Timer t(true, "physics step");
            if (runWithPhysics || doOneStep)
                physicsWorld.step(1 / 60.0f, rbs);
        }
        c->Move(window);

        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());

        for (auto* rb : rbs) {
            s->setVec3("color", glm::vec3(0.0, 0.5, 0.1));
            rb->collider->Draw(s);
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        resize(window, display_w, display_h);
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
        tree->root->asciiprint();
    }
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

        if (ImGui::Button("Rebuild")) {
            Timer t(true, "creating BVH");
            delete tree;
            tree = new BVH(rbs);
            for (size_t i = 0; i < 90; i++)
                tree->removeRigidBody(rbs[i]);
            break; // TODO remove
        }
        /*
        std::vector<CollisionPoint> points;
        {
            Timer t(true, "drawing everything");
            int i = 0;
            for each (auto rb in rbs) {
                s->setVec3("color", glm::vec3(0, 1, 0));
                rb->gui(i);
                i++;
                // std::vector<CollisionPoint> newpoints = tree->getCollisions(rb);
                //points.insert(points.begin(), newpoints.begin(), newpoints.end());
                ///std::cout << newpoints.size() << " ";
                //if (!newpoints.empty())
                //    s->setVec3("color", glm::vec3(1, 0, 0));
                rb->collider->Draw(s);
            }
        }
        s->setVec3("color", glm::vec3(1));
        */

        //if (tree)
        //    tree->root->Draw(s);

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
    // testRayMeshIntersection(Triangle);

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

    auto* cubePhy = new RigidBody(new AABB());
    auto* spherePhy = new RigidBody(new Sphere());
    spherePhy->movable = true;


    if (true) {
        //spherePhy->movable = false;
        //rbs.push_back(spherePhy);
    } else {
        cubePhy->movable = false;
        rbs.push_back(cubePhy);
    }

    {
        RigidBody* tmp;
        
        for (int i = 0; i < 10; i++)
            for (int j = 0; j < 10; j++) {
                tmp = new RigidBody(new Sphere(glm::vec3(i * 2, j * 2, 0), 1), 1); // TODO remove the last param after optimizing BVH
                if (j == 0)
                    tmp->movable = false;
                rbs.push_back(tmp);
            }
    }
/*
    for (int i = 3; i < 4; ++i) {
        RigidBody* tmp;
        switch (i % 2) {
        case 0:
            tmp = new RigidBody(new Sphere(), 1);
            break;
        case 1:
            tmp = new RigidBody(new Capsule(), 1);
            break;
        case 2:
            tmp = new RigidBody(new AABB(), 1);
            break;
        }
        tmp->position = glm::vec3(1, 5 * i, 0);
        rbs.push_back(tmp);
    }
*/
    testBVH(rbs);
    //testPhysics(rbs);
    for (auto* r : rbs)
        delete r;
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
