#pragma once


#include "Vertex.h"
#include "Shader.h"
#include "BoundingVolumes.h"

#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

class Mesh
{
public:
    std::string name;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    glm::vec3 translation{};
    glm::vec3 rotation{};
    glm::vec3 scale{};

    Collider* bv = nullptr;

    bool wireframeON = false, solidON = true, boundingBoxON = true;
    Mesh();
    ~Mesh();

    glm::mat4 getTransform();


        unsigned int VAO{}, VBO{}, EBO{};

    void prepare();
    void Draw(Shader* shader);
    void gui(int outIndex);
};
