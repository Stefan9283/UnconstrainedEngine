#pragma once

#include "Common.h"

#include "Vertex.h"
#include "Shader.h"
#include "BoundingVolumes.h"
#include "PhysicsWorld.h"


class Mesh
{
public:
    std::string name;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    transform localTransform;

    RigidBody* rigidbody = nullptr;

    bool wireframeON = false, solidON = true, boundingBoxON = true;
    Mesh();
    ~Mesh();

    glm::mat4 getTransform();

    unsigned int VAO{}, VBO{}, EBO{};

    void prepare();
    void Draw(Shader* shader);
    void gui(int outIndex);
    void addBody(RigidBody* body);
    void setPosition(glm::vec3 pos);

    std::vector<Vertex> getTriangle(int index);
    int getTriangleCount();
};
