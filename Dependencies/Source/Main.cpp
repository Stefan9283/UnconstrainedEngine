#define STBI_IMAGE_IMPLEMENTATION

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <iostream>
#include <ctime>
#include <vector>

#include <vector>
#include <glm/gtc/type_ptr.hpp>
#include <iomanip>
#include "ObjLoad.h"
#include "Camera.h"
#include "BoundingVolumes.h"
#include "Octree.h"
#include "Mesh.h"

#include "imconfig.h"
#include "imgui.h"

#define USE_IMGUI_API

#include "ImGuizmo.h"
#include "ImSequencer.h"
#include "ImZoomSlider.h"
#include "ImCurveEdit.h"
#include "Debug.h"

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
GLFWwindow* window;
Shader* s;
Camera* c;
Ray* r;
Mesh* crosshair;
std::vector<Mesh*> meshes;

Mesh* generateBoxMesh(glm::vec3 min, glm::vec3 max) {
    std::vector<glm::vec3> verticesPos;
    verticesPos.emplace_back(glm::vec3(min.x, min.y, min.z));
    verticesPos.emplace_back(glm::vec3(min.x, min.y, max.z));
    verticesPos.emplace_back(glm::vec3(min.x, max.y, min.z));
    verticesPos.emplace_back(glm::vec3(min.x, max.y, max.z));
    verticesPos.emplace_back(glm::vec3(max.x, min.y, min.z));
    verticesPos.emplace_back(glm::vec3(max.x, min.y, max.z));
    verticesPos.emplace_back(glm::vec3(max.x, max.y, min.z));
    verticesPos.emplace_back(glm::vec3(max.x, max.y, max.z));

    Mesh* body = new Mesh();

    for (auto v : verticesPos)
        body->vertices.push_back(Vertex{v, v});

    body->indices.push_back(3);
    body->indices.push_back(2);
    body->indices.push_back(0);

    body->indices.push_back(0);
    body->indices.push_back(1);
    body->indices.push_back(3);

    body->indices.push_back(6);
    body->indices.push_back(7);
    body->indices.push_back(4);

    body->indices.push_back(4);
    body->indices.push_back(7);
    body->indices.push_back(5);

    body->indices.push_back(7);
    body->indices.push_back(3);
    body->indices.push_back(5);

    body->indices.push_back(5);
    body->indices.push_back(3);
    body->indices.push_back(1);

    body->indices.push_back(2);
    body->indices.push_back(6);
    body->indices.push_back(0);

    body->indices.push_back(0);
    body->indices.push_back(6);
    body->indices.push_back(4);

    body->indices.push_back(0);
    body->indices.push_back(4);
    body->indices.push_back(1);

    body->indices.push_back(4);
    body->indices.push_back(5);
    body->indices.push_back(1);

    body->indices.push_back(2);
    body->indices.push_back(3);
    body->indices.push_back(7);

    body->indices.push_back(7);
    body->indices.push_back(6);
    body->indices.push_back(2);
    body->prepare();
    return body;
}

void testBasicCollision() {
    std::vector<bool> collision;
    for (int i = 0; i < meshes.size() ; i++)
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
        if (r) r->body->Draw(s);




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

        ImGui::SliderFloat("cam speed", &c->speed, 0.001, 0.5);

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

            //hit_or_nah = wasMeshHit(meshes[0]->bv, r);


            for (int l = 0; l < collision.size(); ++l) {
                collision[l] = false;
            }
            for(int i=0; i<meshes.size(); i++) {
                Mesh* m = meshes[i];
                for (int j=0; j<meshes.size(); j++) {
                    Mesh* m2 = meshes[j];
                    if (m != m2)
                        if(m->bv->checkCollision(m2->bv).hasCollision) {
                            collision[i] = true;
                            collision[j] = true;
                        }
                }
            }
            if(r) {
                for (int i=0; i<meshes.size(); i++) {
                    Mesh* m = meshes[i];
                    if (m->bv->checkCollision(r).hasCollision) {
                        collision[i] = true;
                        collision[collision.size() - 1] = true;
                    }
                }
            }
        }

        Ray* tmp_r = Ray::generateRay(window, c);
        if (tmp_r) {
            if (r) delete r;
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
            for (int i = 0; i < meshes.size(); ++i) {
                Mesh* m = meshes[i];
                m->boundingBoxON = !m->boundingBoxON;
            }
        }
        if(ImGui::Button("Toggle solid all" )) {
            for (int i = 0; i < meshes.size(); ++i) {
                Mesh* m = meshes[i];
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

        if (r) r->body->Draw(s);

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glm::mat4 proj = *c->getprojmatrix();
        glm::mat4 model = glm::mat4(1);
        s->setMat4("proj", &proj);
        s->setMat4("view", &model);
        model = glm::translate(model, c->goFront);
        s->setMat4("model", &model);
        glViewport(0, 0, display_w, display_h);


        ImGui::SliderFloat("cam speed", &c->speed, 0.001, 0.5);

        s->setVec3("color", glm::vec3(0, 1, 0));

        s->setFloat("flatColorsON", 1);
        crosshair->Draw(s);

        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::Checkbox("Continuously checking for collisions", &continuouslyChecking);
        if(ImGui::Button("Check collisions" ) || continuouslyChecking) {
            if (hit_or_nah.size()) {
                delete hit_or_nah[0];
                delete hit_or_nah[1];
            }

            hit_or_nah = wasMeshHit(mesh->bv, r);
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
            if (r) delete r;
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


        if(ImGui::Button("Toggle bounding volumes all" )) {
            for (int i = 0; i < meshes.size(); ++i) {
                Mesh* m = meshes[i];
                m->boundingBoxON = !m->boundingBoxON;
            }
        }
        if(ImGui::Button("Toggle solid all" )) {
            for (int i = 0; i < meshes.size(); ++i) {
                Mesh* m = meshes[i];
                m->solidON = !m->solidON;
            }
        }


        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwSwapBuffers(window);

        glfwPollEvents();
    }
}
void testOctree(Mesh* mesh) {
    Octree* octree = nullptr;

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
            octree = new Octree(mesh, 7);
        }

        mesh->Draw(s);

        s->setVec3("color", glm::vec3(0,1,0));
        if (octree)
            octree->root->Draw(s);

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
int main() {
#pragma region prepare OGL
    if (!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);

    window = glfwCreateWindow(1040, 1040, "Window name", NULL, NULL);

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

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init( "#version 330");
    glEnable(GL_DEPTH_TEST);

#pragma endregion
    s = new Shader("Dependencies/Shaders/simpleVert.glsl", "Dependencies/Shaders/simpleFrag.glsl");
    s->bind();

    c = new Camera(window);
    glClearColor(0.1, 0.1, 0.3, 1);

#pragma region meshes

    //Mesh* cube = readObj("Box.obj");
    //cube->bv = new AABB(cube);
    ////cube->translation = glm::vec3(1.76, 1.76, 0);
    ////cube->rotation = glm::vec3 (0, -145, 0);
    //cube->solidON = false;
    //meshes.push_back(cube);

    //Mesh* sphere = readObj("Sphere.obj");
    ////sphere->translation += glm::vec3(0,-1,0);
    //sphere->bv = new BoundingSphere(sphere);
    //sphere->solidON = false;
    ////sphere->translation = glm::vec3(-1.76, 1.11, 0);
    //sphere->solidON = false;
    //meshes.push_back(sphere);

    //Mesh* Yen = readObj("Yen.obj");
    //Yen->bv = Capsule::generateCapsule(Yen);
    //meshes.push_back(Yen);

    Mesh* Mercy = readObj("Mercy2.obj");
    Mercy->bv = new TriangleMesh(Mercy);
    Mercy->solidON = false;
    meshes.push_back(Mercy);

    //Mesh* Triangle = readObj("Triangle.obj");
    //Triangle->bv = new TriangleMesh(Triangle);
    //Triangle->solidON = false;
    //Triangle->rotation = glm::vec3(0, 180, 0);
    //meshes.push_back(Triangle);

    r = nullptr; //new Ray(glm::vec3(-0.117, 1.522, 0.281), glm::vec3(0.143, -0.057, -0.988), 100, true); // nullptr;

    crosshair = new Mesh();
    crosshair->vertices.push_back(Vertex{glm::vec3(0,  1,  -25.0f)});
    crosshair->vertices.push_back(Vertex{glm::vec3(0, -1,  -25.0f)});
    crosshair->vertices.push_back(Vertex{glm::vec3(1,  0,  -25.0f)});
    crosshair->vertices.push_back(Vertex{glm::vec3(-1, 0,  -25.0f)});
    crosshair->indices.push_back(0);
    crosshair->indices.push_back(1);
    crosshair->indices.push_back(0);
    crosshair->indices.push_back(2);
    crosshair->indices.push_back(3);
    crosshair->indices.push_back(2);
    crosshair->wireframeON = true;
    crosshair->solidON = true;
    crosshair->prepare();
#pragma endregion

    //testOctree(meshes[0]);
    //testBasicCollision();
    //testRayMeshIntersection(meshes[0]);

    s->unbind();

    if (r) delete r;


    for(Mesh* m : meshes)
        delete m;
    delete crosshair;
    delete s;
    delete c;

    glfwTerminate();
    return 0;
}
