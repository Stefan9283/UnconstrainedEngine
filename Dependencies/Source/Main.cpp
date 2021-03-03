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
#include "ObjLoad.h"
#include "Camera.h"
#include "BoundingVolumes.h"
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

Shader* s;
extern ImGuizmo::OPERATION mCurrentGizmoOperation;

int main() {
#pragma region prepare OGL
    GLFWwindow* window;

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

    Camera* c = new Camera(window);
    glClearColor(0.1, 0.1, 0.3, 1);

#pragma region meshes
    std::vector<Mesh*> meshes;

    //Mesh* cube = readObj("Box.obj");
    //cube->bv = new AABB(cube);
    //cube->translation = glm::vec3(1.76, 1.76, 0);
    //cube->rotation = glm::vec3 (0, -145, 0);
    //cube->solidON = false;
    //meshes.push_back(cube);

    //Mesh* sphere = readObj("Sphere.obj");
    //sphere->translation += glm::vec3(0,-1,0);
    //sphere->bv = new BoundingSphere(sphere);
    //sphere->solidON = false;
    //sphere->translation = glm::vec3(-1.76, 1.11, 0);
    //sphere->solidON = false;
    //meshes.push_back(sphere);

    //////Mesh* Yen = readObj("Yen.obj");
    //////Yen->bv = Capsule::generateCapsule(Yen);
    //////meshes.push_back(Yen);

    //Mesh* triangle = readObj("Triangle.obj");
    //triangle->bv = new TriangleMesh(triangle);
    //triangle->rotation = glm::vec3(0, -180, 0);
    //triangle->translation = glm::vec3(0, 1.11, 0);
    //triangle->solidON = false;
    //meshes.push_back(triangle);

    //Mesh* triangle2 = readObj("Triangle.obj");
    //triangle2->bv = new TriangleMesh(triangle2);
    //triangle2->rotation = glm::vec3(0, -180, -49);
    //triangle2->translation = glm::vec3(0, 1.316, 0);
    //triangle2->solidON = false;
    //meshes.push_back(triangle2);

    Mesh* Mercy = readObj("Mercy.obj");
    Mercy->bv = new TriangleMesh(Mercy);
    meshes.push_back(Mercy);


    //Mesh* test = new Mesh();
    //test->vertices.push_back(Vertex{glm::vec3(0.5f, 1, 0), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //test->vertices.push_back(Vertex{glm::vec3(-1, 0, 0), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //test->vertices.push_back(Vertex{glm::vec3(1, 0, 0), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //test->indices.push_back(0);
    //test->indices.push_back(1);
    //test->indices.push_back(2);
    //test->bv = new TriangleMesh(test);
    //test->prepare();
    //meshes.push_back(test);

    //std::vector<Mesh*> mamalLor;

    //Mesh* m1 = new Mesh();
    //m1->vertices.push_back(Vertex{glm::vec3(0.013611, 1.564913, 0.044879), glm::vec3(0.741500, 0.415300, -0.527000)});
    //m1->vertices.push_back(Vertex{glm::vec3(0.014145, 1.565383, 0.046110), glm::vec3(0.741500, 0.415300, -0.527000)});
    //m1->vertices.push_back(Vertex{glm::vec3(0.013871, 1.564592, 0.045128), glm::vec3(0.741500, 0.415300, -0.527000)});
    //m1->indices.push_back(0);
    //m1->indices.push_back(1);
    //m1->indices.push_back(2);
    //m1->bv = new TriangleMesh(m1);
    //m1->prepare();
    //meshes.push_back(m1);

    //Mesh* m2 = new Mesh();
    //m2->vertices.push_back(Vertex{glm::vec3(0.013871, 1.564592, 0.045128), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //m2->vertices.push_back(Vertex{glm::vec3(0.014145, 1.565383, 0.046110), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //m2->vertices.push_back(Vertex{glm::vec3(0.014555, 1.565118, 0.046085), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
//
    //Mesh* m3 = new Mesh();
    //m3->vertices.push_back(Vertex{glm::vec3(-0.024256, 1.565797, 0.040848), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //m3->vertices.push_back(Vertex{glm::vec3(-0.024852, 1.566274, 0.040575), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //m3->vertices.push_back(Vertex{glm::vec3(-0.025058, 1.565326, 0.040927), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
//
    //Mesh* m4 = new Mesh();
    //m4->vertices.push_back(Vertex{glm::vec3(-0.024000, 1.566069, 0.040335), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //m4->vertices.push_back(Vertex{glm::vec3(-0.024256, 1.565797, 0.040848), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});
    //m4->vertices.push_back(Vertex{glm::vec3(-0.023475, 1.565964, 0.040644), glm::vec3(glm::cross(glm::vec3(0.5f, 1, 0), glm::vec3(-1, 0, 0)))});

    //mamalLor.push_back(m1);
    //mamalLor.push_back(m2);
    //mamalLor.push_back(m3);
    //mamalLor.push_back(m4);

    //for (int n = 0; n < mamalLor.size(); ++n) {
    //    mamalLor[n]->indices.push_back(0);
    //    mamalLor[n]->indices.push_back(1);
    //    mamalLor[n]->indices.push_back(2);
    //    mamalLor[n]->bv = new TriangleMesh(mamalLor[n]);
    //    mamalLor[n]->prepare();
    //}


    Ray* r = new Ray(glm::vec3(-0.117, 1.522, 0.281), glm::vec3(0.143, -0.057, -0.988), 100, true); // nullptr;



    Mesh* crosshair = new Mesh();
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

    std::vector<bool> collision;
    for (int i = 0; i < meshes.size() ; i++)
        collision.push_back(false);
    collision.push_back(false);

    bool continuouslyChecking = false;

    std::vector<Mesh*> hit_or_nah;

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

        float* camviewvec = (float*)glm::value_ptr(c->view);
        float* projvec = (float*)glm::value_ptr(c->proj);
        float* identity = (float*)glm::value_ptr(glm::mat4(1));
        //ImGuizmo::DrawGrid(camviewvec, projvec, identity, 100.f);

        s->setFloat("flatColorsON", 0);
        for (int i = 0; i < collision.size() - 1 ; ++i) {
            EditTransform(c, &c->view, &c->proj, meshes[i]);
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

            if (hit_or_nah.size()) {
                delete hit_or_nah[0];
                delete hit_or_nah[1];
            }

            hit_or_nah = wasMeshHit(meshes[0]->bv, r);

            /*
            for (int l = 0; l < collision.size(); ++l) {
                collision[l] = false;
            }
            for(int i=0; i<meshes.size(); i++) {
                Mesh* m = meshes[i];
                for (int j=0; j<meshes.size(); j++) {
                    Mesh* m2 = meshes[j];
                    if (m != m2)
                        if(m->bv->checkCollision(m2->bv)) {
                            collision[i] = true;
                            collision[j] = true;
                        }
                }
            }
            if(r) {
                for (int i=0; i<meshes.size(); i++) {
                    Mesh* m = meshes[i];
                    if (m->bv->checkCollision(r)) {
                        collision[i] = true;
                        collision[collision.size() - 1] = true;
                    }
                }
            }*/
        }


        if (hit_or_nah.size()) {
            //std::cout << "sup\n";
            s->setVec3("color", glm::vec3(1,0,0));
            hit_or_nah[0]->Draw(s);
            s->setVec3("color", glm::vec3(0,1,0));
            hit_or_nah[1]->Draw(s);
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

        /*
        s->setFloat("flatColorsON", 1);
        s->setVec3("color", glm::vec3(1,1,1));
        for (int n = 0; n < mamalLor.size(); ++n) {
            mamalLor[n]->Draw(s);
        }*/



        ImGui::Render();




        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwSwapBuffers(window);

        glfwPollEvents();
    }

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
