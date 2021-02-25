#define STBI_IMAGE_IMPLEMENTATION

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <stb_image.h>

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <iostream>
#include <ctime>
#include <vector>



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

#include <vector>
#include "ObjLoad.h"
#include "Camera.h"
#include "BoundingVolumes.h"
#include "Mesh.h"

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
    Shader* s = new Shader("Dependencies/Shaders/simpleVert.glsl", "Dependencies/Shaders/simpleFrag.glsl");
    s->bind();

    Camera* c = new Camera(window);

    std::vector<Mesh*> meshes;

    //Mesh* cube = readObj("Box.obj");
    //cube->bv = new AABB(cube);
    //meshes.push_back(cube);

    //Mesh* sphere = readObj("Sphere.obj");
    //sphere->translation += glm::vec3(0,-1,0);
    //sphere->bv = new BoundingSphere(sphere);
    //meshes.push_back(sphere);
    //sphere->solidON = false;

    //Mesh* Yen = readObj("Yen.obj");
    //Yen->bv = new TriangleMesh(Yen);
    //meshes.push_back(Yen);

    Mesh* Triangle = readObj("Triangle.obj");
    Triangle->bv = new TriangleMesh(Triangle);
    meshes.push_back(Triangle);

    //Mesh* Mercy = readObj("Mercy.obj");
    //Mercy->bv = new TriangleMesh(Mercy);
    //meshes.push_back(Mercy);

    //Mesh* Tube = readObj("Tube.obj");
    //meshes.push_back(Tube);
    //Mesh* halfSphere = readObj("halfSphere.obj");
    //meshes.push_back(halfSphere);
    glClearColor(0.1, 0.1, 0.3, 1);

    Ray* r = nullptr;

    std::vector<bool> collision;
    for (int i = 0; i < meshes.size() ; i++)
        collision.push_back(false);
    collision.push_back(false);
    bool continuouslyChecking = false;

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

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();



        s->setFloat("flatColorsON", 1);
        crosshair->Draw(s);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::Checkbox("Continuously checking for collisions", &continuouslyChecking);
        if(ImGui::Button("Check collisions" ) || continuouslyChecking) {
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
            }
        }


        s->setFloat("flatColorsON", 0);
        s->setMat4("proj", c->getprojmatrix());
        s->setMat4("view", c->getviewmatrix());
        s->setVec3("cameraPos", c->position);
        c->update_proj(window);
        c->Move(window);



        Ray* tmp_r = Ray::generateRay(window, c);
        if (tmp_r) {
            if (r) delete r;
            r = tmp_r;
            collision[collision.size() - 1] = false;
        }

        s->setFloat("flatColorsON", 0);
        for (int i = 0; i < collision.size() - 1 ; ++i) {
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


        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glm::mat4 proj = *c->getprojmatrix();
        glm::mat4 model = glm::mat4(1);
        s->setMat4("proj", &proj);
        s->setMat4("view", &model);
        //model = glm::scale(model, glm::vec3(0.0001f));
        model = glm::translate(model, c->goFront);
        s->setMat4("model", &model);


        ImGui::Render();

        glViewport(0, 0, display_w, display_h);
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
