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

class Collider;
class AABB;
class BoundingSphere;
class Ray;
class TriangleMesh;
class Triangle;
class Capsule;

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
    bool checkCollision(Collider* col);
    virtual bool checkCollision(BoundingSphere* col) = 0;
    virtual bool checkCollision(AABB* col) = 0;
    virtual bool checkCollision(Triangle* t) = 0;
    virtual bool checkCollision(TriangleMesh* col) = 0;
    virtual bool checkCollision(Ray* col) = 0;
    virtual bool checkCollision(Capsule* col) = 0;

    virtual ~Collider();
};

class BoundingSphere : public Collider {
public:

    float radius;
    glm::vec3 pos{};

    void setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) override;

    bool checkCollision(BoundingSphere* bv) override;
    bool checkCollision(AABB* bv) override;
    bool checkCollision(Ray* r) override;
    bool checkCollision(TriangleMesh* col) override;
    bool checkCollision(Triangle* t) override;
    bool checkCollision(Capsule* col) override;

    explicit BoundingSphere(Mesh* mesh);
    BoundingSphere(glm::vec3 pos, float radius);

};

class AABB : public Collider{
public:
    glm::vec3 max{}, min{}, max0, min0, offset{};


    void updateMinMax(glm::mat4 transform);
    void setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) override;
    std::vector<glm::vec3> generateVerices(glm::vec3 min, glm::vec3 max);
    Mesh* generateNewMesh();
    bool checkCollision(AABB* bv) override;
    bool checkCollision(BoundingSphere* bv) override;
    bool checkCollision(Ray* r) override;
    bool checkCollision(TriangleMesh* col) override;
    bool checkCollision(Triangle* t) override;
    bool checkCollision(Capsule* col) override;

    AABB(glm::vec3 min, glm::vec3 max);
    explicit AABB(Mesh* mesh);
};

class TriangleMesh : public Collider {
public:
    bool checkCollision(TriangleMesh* col) override;
    bool checkCollision(AABB* bv) override;
    bool checkCollision(BoundingSphere* bv) override;
    bool checkCollision(Ray* r) override;
    bool checkCollision(Triangle* t) override;
    bool checkCollision(Capsule* col) override;

    bool checkCollisionByTriangle(Collider* col);

    TriangleMesh(Mesh* mesh);
    ~TriangleMesh() { body = nullptr; }
};

class Triangle : public Collider {
public:
    glm::vec3 vertices[3];
    glm::vec3 norm;

    bool checkCollision(AABB* bv) override;
    bool checkCollision(BoundingSphere* bv) override;
    bool checkCollision(Ray* r) override;
    bool checkCollision(TriangleMesh* col) override;
    bool checkCollision(Triangle* t) override;
    bool checkCollision(Capsule* col) override;

    bool isInside(glm::vec3 point);

    Triangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 norm);
};

class Ray : public Collider {
public:
    float length;
    glm::vec3 origin, direction;


    bool checkCollision(BoundingSphere* bv) override;
    bool checkCollision(AABB* bv) override;
    bool checkCollision(Ray* r) override;
    bool checkCollision(TriangleMesh* col) override;
    bool checkCollision(Triangle* t) override;
    bool checkCollision(Capsule* col) override;

    ///
    bool checkCollision(Triangle *t, glm::vec3 C, glm::vec3 D);
    /// 
    

    static Ray* generateRay(GLFWwindow* window, Camera* cam);
    Ray(glm::vec3 origin, glm::vec3 direction, float length);

};

class Capsule : public Collider { // aka swept sphere
public:
    glm::vec3 start0{}, end0{};
    float radius;

    glm::vec3 start{}, end{};


    static Capsule* generateCapsule(Mesh* mesh);
    void setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) override;
    Capsule(glm::vec3 start, glm::vec3 end, float radius);
    bool checkCollision(BoundingSphere* col) override;
    bool checkCollision(AABB* col) override;
    bool checkCollision(Triangle* t) override;
    bool checkCollision(TriangleMesh* col) override;
    bool checkCollision(Ray* col) override;
    bool checkCollision(Capsule* col) override;


};
#endif //TRIANGLE_BoundingVolume_H
