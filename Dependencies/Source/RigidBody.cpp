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
    
    rotation = glm::quat(1, 0, 0, 0);
    position = glm::vec3(0);
}

glm::mat4 RigidBody::getTransform() {
    return getTranslationMatrix() * getRotationMatrix();
}
glm::mat4 RigidBody::getTranslationMatrix() {
    return glm::translate(glm::mat4(1), position);
}
glm::mat4 RigidBody::getRotationMatrix() {
    return glm::toMat4(rotation);
}

void RigidBody::gui(int index) {
    std::string name = "RigidBody " + std::to_string(index) + " Settings";
    if (ImGui::TreeNode(name.c_str())) {
        ImGui::Checkbox("Movable", &movable);
        ImGui::SameLine();
        ImGui::Checkbox("Rotatable", &canBeRotated);

        ImGui::SliderFloat("mass", &mass, 0, 100000);

        float t[] = { position.x, position.y, position.z };
        ImGui::SliderFloat3("position ", t, -10000, 10000);
        position = glm::vec3(t[0], t[1], t[2]);

        float r[] = { rotation.w, rotation.x, rotation.y, rotation.z };
        ImGui::SliderFloat4("rotation ", r, -1, 1);
        glm::quat newRot = glm::quat(r[0], r[1], r[2], r[3]);
        if (newRot != rotation) {
            rotation = newRot;
            rotation = glm::normalize(rotation);
        }

        float v[] = { velocity.x, velocity.y, velocity.z};
        ImGui::SliderFloat3("velocity ", v, -10000, 10000);
        velocity = glm::vec3(v[0], v[1], v[2]);

        float ang[] = { angularVel.x, angularVel.y, angularVel.z };
        ImGui::SliderFloat3("angular velocity ", ang, -10000, 10000);
        angularVel = glm::vec3(ang[0], ang[1], ang[2]);

        if (ImGui::Button("Reset")) {
            velocity = {};
            rotation = glm::quat(1, 0, 0, 0);
            position = {};
            angularVel = {};
        }

        if (collider)
            collider->gui(index);

        ImGui::TreePop();
    }



}

glm::mat3 RigidBody::getInertiaTensor() {
    return inertia;
}
