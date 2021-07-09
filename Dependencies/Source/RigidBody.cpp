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
void RigidBody::setTransform(glm::vec3 translation, glm::quat rotation, glm::vec3 scale) {
    position += translation;
}


glm::mat4 RigidBody::getTransform() {
    glm::mat4 T = glm::mat4(1), R = glm::mat4(1), S = glm::mat4(1); //TODO
    T = glm::translate(glm::mat4(1), position);
//    std::cout << mesh << "\n";
    // if (mesh) {
    //     return T;
    //     return mesh->getTransform() * T;
    // }
    return T;
}

void RigidBody::moveObject(glm::vec3 pos) {
    if (movable)
        position += pos;
}
