//
// Created by Stefan on 22-Feb-21.
//

#ifndef TRIANGLE_BoundingVolume_H
#define TRIANGLE_BoundingVolume_H

#include "Common.h"


#include "Camera.h"

class Mesh;
class Shader;

class Collider;
class AABB;
class BoundingSphere;
class Ray;
class TriangleMesh;
class Triangle;
class Capsule;

class RigidBody;

#include "Vertex.h"

class ColliderMesh {
public:
    std::string name;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    transform localTransform;

    bool wireframeON = true, solidON = false;
    ColliderMesh();
    ~ColliderMesh();

    unsigned int VAO{}, VBO{}, EBO{};

    void prepare();
    void gui(int outIndex);
    void Draw(glm::mat4 parentMatrix, Shader* shader);

    glm::mat4 getTransform();
};

class CollisionPoint {
public:
    glm::vec3 A{}; // furthest point of A in collider B
    glm::vec3 B{}; // furthest point of B in collider A
    glm::vec3 normal{}; // normalized B - A
    float depth; // length B - A
    bool hasCollision;
    CollisionPoint(glm::vec3 A, glm::vec3 B);
    CollisionPoint();
};

class Collider {
public:
    ColliderMesh* body = nullptr;
    glm::mat4 localTransform = glm::mat4(1);
    RigidBody* parent = nullptr;

    virtual void update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
        glm::mat4 T, R = glm::mat4(1), S;
        T = glm::translate(glm::mat4(1), pos);
        R = glm::toMat4(rot);
        S = glm::scale(glm::mat4(1), scale);
        localTransform = T * R * S;
    };
    virtual void Draw(Shader* shader);
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

    void update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) override;

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
    void update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) override;
    std::vector<Vertex> generateVerices(glm::vec3 min, glm::vec3 max);
    ColliderMesh *generateNewMesh();
    glm::vec3 closestPoint(glm::vec3 p);

    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(BoundingSphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    void AABB::Draw(Shader* shader) override;

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

    bool isInside(glm::vec3 point);

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
    Ray(glm::vec3 origin, glm::vec3 direction, float length, bool createMesh = false);
    void toString() override;
};

class Capsule : public Collider { // aka swept sphere
public:
    glm::vec3 start0{}, end0{};
    float radius;

    glm::vec3 start{}, end{};


    static Capsule* generateCapsule(Mesh* mesh);
    void update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) override;
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
