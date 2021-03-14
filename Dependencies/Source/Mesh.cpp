//
// Created by Stefan on 22-Feb-21.
//

#include "Mesh.h"
#include "glm/gtc/matrix_transform.hpp"

Mesh::Mesh() {
    bv = nullptr;
    translation = glm::vec3(0);
    rotation = glm::vec3(0);
    scale = glm::vec3(1);
}
Mesh::~Mesh() {
    if (bv) delete bv;
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);}

glm::mat4 Mesh::getTransform() {
    glm::mat4 T, R = glm::mat4(1), S;
    T = glm::translate(glm::mat4(1), translation);
    R = glm::rotate(R, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
    R = glm::rotate(R, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
    R = glm::rotate(R, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
    S = glm::scale(glm::mat4(1), scale);
    return T * R * S;
}

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

    if (bv && boundingBoxON) {
        bv->setTransform(translation, rotation, scale);
        glDisable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        if (bv->body == this) {
            bool bound = boundingBoxON;
            bool wireframe = wireframeON;
            bool solid = solidON;

            boundingBoxON = false;
            wireframeON = true;
            solidON = false;

            bv->body->Draw(shader);

            solidON = solid;
            boundingBoxON = bound;
            wireframeON = wireframe;
        }
        else
            bv->body->Draw(shader);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    glm::mat4 model = getTransform();
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
void Mesh::gui(int outIndex = 0) {

    std::string s;
    s = name + " " + std::to_string(outIndex);
    if (ImGui::TreeNode(s.c_str())) {
        s = "wireframeON " + std::to_string(outIndex);
        ImGui::Checkbox(s.c_str(), &wireframeON);
        s = "solidON " + std::to_string(outIndex);
        ImGui::Checkbox(s.c_str(), &solidON);
        s = "boundingBoxON " + std::to_string(outIndex);
        ImGui::Checkbox(s.c_str(), &boundingBoxON);

        float t[4] = {translation.x, translation.y, translation.z, 1.0f};
        s = "position " + std::to_string(outIndex);
        ImGui::SliderFloat3(s.c_str(), t, -10, 10);
        translation = glm::vec3(t[0], t[1], t[2]);

        float r[4] = {rotation.x, rotation.y, rotation.z, 1.0f};
        s = "rotation " + std::to_string(outIndex);
        ImGui::SliderFloat3(s.c_str(), r, -180, 180);
        rotation = glm::vec3(r[0], r[1], r[2]);

        ImGui::TreePop();
    }

}

int Mesh::getTriangleCount() {
    return indices.size()/3;
}

std::vector<Vertex> Mesh::getTriangle(int index) {
    std::vector<Vertex> v;
    for (int i = index * 3; i < index * 3 + 3; i++) {
        v.push_back(vertices[i]);
    }
    return v;
}
