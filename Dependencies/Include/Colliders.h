#ifndef TRIANGLE_BoundingVolume_H
#define TRIANGLE_BoundingVolume_H

#include "Common.h"

#include "Camera.h"

class Mesh;
class Shader;

class Collider;
class AABB;
class Sphere;
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
    bool wasReversed{};
    CollisionPoint(glm::vec3 A, glm::vec3 B);
    CollisionPoint();
    std::string toString();
};

class Collider {
public:
    ColliderMesh* body = nullptr;
    transform localTransform = {
        glm::vec3(0), glm::vec3(1), glm::quat()
    };
    //glm::mat4 localTransform = glm::mat4(1);
    RigidBody* parent = nullptr;

    virtual void update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
        localTransform.tr = pos;
        localTransform.rot = rot;
        localTransform.sc = scale;
    };

    glm::mat4 getLocalTransform();
    glm::mat4 getTransform();

    virtual void Draw(Shader* shader);

    virtual std::string toString() = 0;

    CollisionPoint checkCollision(Collider* col);
    virtual CollisionPoint checkCollision(Sphere* col) = 0;
    virtual CollisionPoint checkCollision(AABB* col) = 0;
    virtual CollisionPoint checkCollision(Triangle* t) = 0;
    virtual CollisionPoint checkCollision(TriangleMesh* col) = 0;
    virtual CollisionPoint checkCollision(Ray* col) = 0;
    virtual CollisionPoint checkCollision(Capsule* col) = 0;
    virtual void gui(int index) {}

    virtual ~Collider();
};

class Sphere : public Collider {
public:

    float radius;
    glm::vec3 pos{};

    void update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) override;

    glm::vec3 getCurrentPosition();

    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    void gui(int index) override;

    Sphere(Mesh* mesh);
    Sphere(glm::vec3 pos, float radius);
    std::string toString() override;
    bool isInside(glm::vec3 point);
};


class AABB : public Collider{
public:
    glm::vec3 max{}, min{}, offset{};

    std::vector<Vertex> generateVerices(glm::vec3 min, glm::vec3 max);
    ColliderMesh *generateNewMesh();
    glm::vec3 closestPoint(glm::vec3 p);

    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    glm::vec3 getOffset();
    glm::vec3 getMin();
    glm::vec3 getMax();

    void gui(int index) override;

    AABB(glm::vec3 min, glm::vec3 max);
    explicit AABB(Mesh* mesh);
    std::string toString() override;
    bool isInside(glm::vec3 point);
};

class OBB : public AABB {
public:
    glm::vec3 max{}, min{}, offset{};
    
    glm::vec3 getMin();
    glm::vec3 getMax();
};


class TriangleMesh : public Collider {
public:
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    TriangleMesh(Mesh* mesh);
    ~TriangleMesh() { body = nullptr; }
    std::string toString() override;
};

std::vector<Mesh*> wasMeshHit(Collider* mesh, Collider *col);

class Triangle : public Collider {
public:
    glm::vec3 vertices[3];
    glm::vec3 norm;
    bool twoway = false;

    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    bool isInside(glm::vec3 point);

    Triangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 norm);
    std::string toString() override;
};

class Ray : public Collider {
public:
    float length;
    glm::vec3 origin, direction;

    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;

    static Ray* generateRay(GLFWwindow* window, Camera* cam);
    Ray(glm::vec3 origin, glm::vec3 direction, float length, bool createMesh = false);
    std::string toString() override;
};

class Capsule : public Collider { // aka swept sphere
public:
    glm::vec3 start0{}, end0{};
    float radius;

    glm::vec3 start{}, end{};

    glm::vec3 getEnd();
    glm::vec3 getStart();

    static Capsule* generateCapsule(Mesh* mesh);
    void update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) override;
    Capsule(glm::vec3 start, glm::vec3 end, float radius);
    CollisionPoint checkCollision(Sphere* col) override;
    CollisionPoint checkCollision(AABB* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Ray* col) override;
    CollisionPoint checkCollision(Capsule* col) override;
    std::string toString() override;


};
#endif //TRIANGLE_BoundingVolume_H
