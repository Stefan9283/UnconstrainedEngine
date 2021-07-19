#ifndef TRIANGLE_RIGIDBODY_H
#define TRIANGLE_RIGIDBODY_H

#include "Colliders.h"

class RigidBody {
public:
    // Mesh* mesh;
    Collider* collider;
    glm::vec3
            force{}, velocity{},
            torque{}, angularVel{},
            position{};
    glm::quat rotation = glm::quat();

    float restitution = 0.5;
    float mass = 1;
    bool movable = true;
    float drag = 0.01f;
    RigidBody(Collider* c,  float m = 1);
    glm::mat4 getTransform();
    void gui(int index);
};

#endif //TRIANGLE_RIGIDBODY_H
