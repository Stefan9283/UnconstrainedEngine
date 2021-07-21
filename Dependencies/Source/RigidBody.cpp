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
    return getTranslationMatrix() * getRotationMatrix();
}
glm::mat4 RigidBody::getTranslationMatrix() {
    return glm::toMat4(rotation);
}
glm::mat4 RigidBody::getRotationMatrix() {
    return glm::translate(glm::mat4(1), position);
}

void RigidBody::gui(int index) {
    std::string name = "RigidBody " + std::to_string(index) + " Settings";
    if (ImGui::TreeNode(name.c_str())) {
        float t[4] = { position.x, position.y, position.z, 1.0f };
        name = "rb pos " + std::to_string(index);
        ImGui::SliderFloat3(name.c_str(), t, -10000, 10000);
        position = glm::vec3(t[0], t[1], t[2]);

        float r[] = { rotation.w, rotation.x, rotation.y, rotation.z };
        ImGui::SliderFloat4(("rotation " + std::to_string(index)).c_str(), r, -1, 1);
        glm::quat newRot = glm::quat(r[0], r[1], r[2], r[3]);
        if (newRot != rotation) {
            rotation = newRot;
            rotation = glm::normalize(rotation);
        }

        collider->gui(index);

        ImGui::TreePop();
    }



}
