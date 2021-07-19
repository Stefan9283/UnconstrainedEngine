#include "RigidBody.h"

RigidBody::RigidBody(Collider* c, float m) {
    collider = c;
    // mesh = nullptr;
    c->parent = this;

    force = glm::vec3(0);
    torque = glm::vec3(0);
    velocity = glm::vec3(0);
    angularVel = glm::vec3(0);

    mass = m;
    position = glm::vec3(0);
}

glm::mat4 RigidBody::getTransform() {
    glm::mat4 T, R;
    T = glm::translate(glm::mat4(1), position);
    R = glm::toMat4(rotation);
    return T * R;
}

void RigidBody::gui(int index) {
    std::string name = "RigidBody " + std::to_string(index) + " Settings";
    if (ImGui::TreeNode(name.c_str())) {
        float t[4] = { position.x, position.y, position.z, 1.0f };
        name = "rb pos " + std::to_string(index);
        ImGui::SliderFloat3(name.c_str(), t, -10, 10);
        position = glm::vec3(t[0], t[1], t[2]);
        collider->gui(index);
        ImGui::TreePop();
    }
}
