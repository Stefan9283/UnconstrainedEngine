//
// Created by Stefan on 22-Feb-21.
//

#include "Mesh.h"
#include "RigidBody.h"
#include "glm/gtc/matrix_transform.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

Mesh::Mesh() {
    rigidbody = nullptr;
    localTransform = transform{
        glm::vec3(0),
        glm::vec3(1),
        glm::quat(1, 0, 0, 0)};
}
Mesh::~Mesh() {
    if (rigidbody) delete rigidbody;
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);}


void Mesh::prepare() {
    //assert(vertices.size() != 0);

    if (vertices.size() == 0) return;

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

    glBindVertexArray(0);
}
void Mesh::Draw(Shader* shader) {

    shader->bind();

    if (rigidbody && boundingBoxON) {
        //rigidbody->setTransform(localTransform.tr, localTransform.rot, localTransform.sc);
        glDisable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        rigidbody->collider->Draw(shader);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    glm::mat4 model = getTransform();
    if (rigidbody) 
        model = model * rigidbody->getTransform();
    
    shader->setMat4("model", &model);

    glBindVertexArray(VAO);

    if (wireframeON) {
        shader->bind();
        glDisable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // draw mesh wireframe
        glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, nullptr);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    if (solidON) {
        shader->bind();
        glDrawElements(GL_TRIANGLES, (GLsizei) indices.size(), GL_UNSIGNED_INT, nullptr);
    }
    glBindVertexArray(0);
    shader->unbind();
}
void Mesh::gui(int index = 0) {
    std::string s;
    s = name + " " + std::to_string(index);
    if (ImGui::TreeNode(s.c_str())) {
        s = "wireframeON " + std::to_string(index);
        ImGui::Checkbox(s.c_str(), &wireframeON);
        ImGui::SameLine();
        s = "solidON " + std::to_string(index);
        ImGui::Checkbox(s.c_str(), &solidON);
        ImGui::SameLine();
        s = "boundingBoxON " + std::to_string(index);
        ImGui::Checkbox(s.c_str(), &boundingBoxON);

        float t[] = {localTransform.tr.x, localTransform.tr.y, localTransform.tr.z};
        s = "mesh offset (don't use)" + std::to_string(index);
        ImGui::SliderFloat3(s.c_str(), t, -10, 10);
        localTransform.tr = glm::vec3(t[0], t[1], t[2]);

        float r[] = { localTransform.rot.w, localTransform.rot.x, localTransform.rot.y, localTransform.rot.z };
        ImGui::SliderFloat4(("rotation " + std::to_string(index)).c_str(), r, -1, 1);
        glm::quat newRot = glm::quat(r[0], r[1], r[2], r[3]);
        if (newRot != localTransform.rot) {
            localTransform.rot = newRot;
            localTransform.rot = glm::normalize(localTransform.rot);
        }

        if (rigidbody) 
            rigidbody->gui(index);

        ImGui::TreePop();
    }
}

int Mesh::getTriangleCount() {
    return (int)indices.size()/3;
}
std::vector<Vertex> Mesh::getTriangle(int index) {
    std::vector<Vertex> v;
    for (int i = index * 3; i < index * 3 + 3; i++) {
        v.push_back(vertices[i]);
    }
    return v;
}

void Mesh::addBody(RigidBody* rigidBody) {
    rigidbody = rigidBody;
    //rigidbody->setTransform(glm::vec3(0), glm::vec3(0), glm::vec3(0));
    // rigidBody->mesh = this;
//    rigidbody->setTransform(localTransform.tr, localTransform.rot, localTransform.sc);
}
glm::mat4 Mesh::getTransform() {
    glm::mat4 T, R = glm::mat4(1), S;
    T = glm::translate(glm::mat4(1), localTransform.tr);
    R = glm::toMat4(localTransform.rot);
    S = glm::scale(glm::mat4(1), localTransform.sc);
    return T * R * S;
}
void Mesh::setPosition(glm::vec3 pos) {
    localTransform.tr = pos;
//    if (rigidbody)
//        rigidbody->setTransform(pos, glm::vec3(0), glm::vec3(0));
}
