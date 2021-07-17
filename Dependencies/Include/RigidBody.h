#ifndef TRIANGLE_RIGIDBODY_H
#define TRIANGLE_RIGIDBODY_H

#include "Colliders.h"

class RigidBody {
public:
    // Mesh* mesh;
    Collider* collider;
    glm::vec3
            force, velocity,
            torque, angularVel,
            position;
    float mass;
    bool movable = true;
    float drag = 0.01f;
    RigidBody(Collider* c,  float m = 1);
    void moveObject(glm::vec3 pos);
    void setTransform(glm::vec3 translation, glm::quat rotation, glm::vec3 scale);
    glm::mat4 getTransform();
    void gui(int index);
};

#endif //TRIANGLE_RIGIDBODY_H
