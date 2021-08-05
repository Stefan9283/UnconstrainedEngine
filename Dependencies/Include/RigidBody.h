#ifndef TRIANGLE_RIGIDBODY_H
#define TRIANGLE_RIGIDBODY_H

#include "Colliders.h"

class RigidBody {
public:
    // Mesh* mesh;
    int id;
    Collider* collider;
    glm::vec3
            force{}, velocity{},
            torque{}, angularVel{},
            position{};
    glm::quat rotation = glm::quat();

    float restitution = 0.5;
    float mass = 1;

    bool movable = true;
    bool canBeRotated = true;
    bool sleep = false;

    glm::mat3 inertia = glm::mat3(1);

    float drag = 0.01f;
    RigidBody(Collider* c,  float m = 1);
    glm::mat3 getInertiaTensor();
    glm::mat4 getTransform();
    glm::mat4 getTranslationMatrix();
    glm::mat4 getRotationMatrix();
    void gui(int index);
    void makeImmovable();
};

#endif //TRIANGLE_RIGIDBODY_H
