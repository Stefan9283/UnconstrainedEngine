//
// Created by Stefan on 22-Feb-21.
//

#ifndef TRIANGLE_BoundingVolume_H
#define TRIANGLE_BoundingVolume_H

#include "glm/glm.hpp"
#include "Camera.h"
#include <vector>
#include <GLFW/glfw3.h>

class Mesh;
#include "Vertex.h"
class Collider;
class AABB;
class BoundingSphere;
class Ray;
class TriangleMesh;
class Triangle;
class Capsule;

struct CollisionPoint {
    glm::vec3 A; // furthest point of A in collider B
    glm::vec3 B; // furthest point of B in collider A
    glm::vec3 normal; // normalized B - A
    float depth; // length B - A
    bool hasCollision;
};

class Collider {
public:
    Mesh* body = nullptr;
    glm::mat4 transform;
    virtual void setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) {
        glm::mat4 T, R = glm::mat4(1), S;
        T = glm::translate(glm::mat4(1), pos);
        R = glm::rotate(R, glm::radians(rot.x), glm::vec3(1.0f, 0.0f, 0.0f));
        R = glm::rotate(R, glm::radians(rot.y), glm::vec3(0.0f, 1.0f, 0.0f));
        R = glm::rotate(R, glm::radians(rot.z), glm::vec3(0.0f, 0.0f, 1.0f));
        S = glm::scale(glm::mat4(1), scale);
        transform = T * R * S;
    };
    CollisionPoint checkCollision(Collider* col);
    virtual CollisionPoint checkCollision(BoundingSphere* col) = 0;
    virtual CollisionPoint checkCollision(AABB* col) = 0;
    virtual CollisionPoint checkCollision(Triangle* t) = 0;
    virtual CollisionPoint checkCollision(TriangleMesh* col) = 0;
    virtual CollisionPoint checkCollision(Ray* col) = 0;
    virtual CollisionPoint checkCollision(Capsule* col) = 0;
    virtual void toString() = 0;

    virtual ~Collider();
};

class BoundingSphere : public Collider {
public:

    float radius;
    glm::vec3 pos{};

    void setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) override;

    CollisionPoint checkCollision(BoundingSphere* bv) override;
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    explicit BoundingSphere(Mesh* mesh);
    BoundingSphere(glm::vec3 pos, float radius);
    void toString() override;
    bool isInside(glm::vec3 point);

};

class AABB : public Collider{
public:
    glm::vec3 max{}, min{}, max0, min0, offset{};

    void updateMinMax(glm::mat4 transform);
    void setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) override;
    std::vector<Vertex> generateVerices(glm::vec3 min, glm::vec3 max);
    Mesh* generateNewMesh();
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(BoundingSphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    AABB(glm::vec3 min, glm::vec3 max);
    explicit AABB(Mesh* mesh);
    void toString() override;
    bool isInside(glm::vec3 point);
};

class TriangleMesh : public Collider {
public:
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(BoundingSphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    CollisionPoint checkCollisionByTriangle(Collider* col);

    TriangleMesh(Mesh* mesh);
    ~TriangleMesh() { body = nullptr; }
    void toString() override;
};

std::vector<Mesh*> wasMeshHit(Collider* mesh, Collider *col);

class Triangle : public Collider {
public:
    glm::vec3 vertices[3];
    glm::vec3 norm;
    bool twoway = false;

    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(BoundingSphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    bool isInside(glm::dvec3 point);

    Triangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 norm);
    void toString() override;
};

class Ray : public Collider {
public:
    float length;
    glm::vec3 origin, direction;

    CollisionPoint checkCollision(BoundingSphere* bv) override;
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    static Ray* generateRay(GLFWwindow* window, Camera* cam);
    Ray(glm::vec3 origin, glm::vec3 direction, float length, bool createMesh);
    void toString() override;
};

class Capsule : public Collider { // aka swept sphere
public:
    glm::vec3 start0{}, end0{};
    float radius;

    glm::vec3 start{}, end{};


    static Capsule* generateCapsule(Mesh* mesh);
    void setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) override;
    Capsule(glm::vec3 start, glm::vec3 end, float radius);
    CollisionPoint checkCollision(BoundingSphere* col) override;
    CollisionPoint checkCollision(AABB* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Ray* col) override;
    CollisionPoint checkCollision(Capsule* col) override;
    void toString() override;


};
#endif //TRIANGLE_BoundingVolume_H
