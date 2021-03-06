#ifndef TRIANGLE_BoundingVolume_H
#define TRIANGLE_BoundingVolume_H

#include "Common.h"

#include "Camera.h"

class Mesh;
class Shader;

class Collider;
class AABB;
class OBB;
class Sphere;
class Ray;
class TriangleMesh;
class Triangle;
class Capsule;
class RigidBody;

#include "Vertex.h"
enum class colliderType
{
    sphere,
    aabb,
    obb,
    capsule,
    trianglemesh,
    triangle,
    ray,
    dontknow
};


class ColliderMesh {
public:
    std::string name;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    bool wireframeON = true, solidON = false;
    ~ColliderMesh();

    unsigned int VAO{}, VBO{}, EBO{};

    void prepare();
    void gui(int outIndex);
    void Draw(glm::mat4 parentMatrix, Shader* shader);
};

class CollisionPoint {
public:
    glm::vec3 A{}; // furthest point of A in collider B
    glm::vec3 B{}; // furthest point of B in collider A
    glm::vec3 normal{}; // normalized B - A
    float depth; // length B - A
    bool hasCollision;
    bool wasReversed{};
    CollisionPoint(glm::vec3 A, glm::vec3 B, Collider* c1, Collider* c2);
    CollisionPoint();
    CollisionPoint reversePoint();
    std::string toString();
    Collider* c1, * c2;
    bool operator<(const CollisionPoint& p) const;
};

class Collider {
public:
    transform localTransform = {
        glm::vec3(0), glm::vec3(1), glm::quat()
    };
    colliderType type;

    // contains data relevant for velocity/position resolution
    RigidBody* parent = nullptr;
    // used for rendering
    ColliderMesh* body = nullptr;

    virtual void update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
        localTransform.tr = pos;
        localTransform.rot = rot;
        localTransform.sc = scale;
    };

    virtual glm::mat4 getLocalTransform();
    virtual glm::mat4 getTransform();

    virtual void Draw(Shader* shader);

    virtual std::string toString() = 0;
    void setType(colliderType type);

    CollisionPoint checkCollision(Collider* col);
    virtual CollisionPoint checkCollision(Sphere* col) = 0;
    virtual CollisionPoint checkCollision(AABB* col) = 0;
    virtual CollisionPoint checkCollision(Triangle* t) = 0;
    virtual CollisionPoint checkCollision(TriangleMesh* col) = 0;
    virtual CollisionPoint checkCollision(Ray* col) = 0;
    virtual CollisionPoint checkCollision(Capsule* col) = 0;
    virtual CollisionPoint checkCollision(OBB* col) = 0;

    bool hasCollision(Collider* col);
    virtual bool hasCollision(Sphere* col) = 0;
    virtual bool hasCollision(AABB* col) = 0;
    virtual bool hasCollision(Triangle* t) = 0;
    virtual bool hasCollision(TriangleMesh* col) = 0;
    virtual bool hasCollision(Ray* col) = 0;
    virtual bool hasCollision(Capsule* col) = 0;
    virtual bool hasCollision(OBB* col) = 0;

    virtual void gui(int index) {}

    virtual ~Collider();
};

class Sphere : public Collider {
public:
    float radius;
    glm::vec3 pos{};

    glm::vec3 getCurrentPosition();

    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;
    CollisionPoint checkCollision(OBB* col) override;

    bool hasCollision(Sphere* col) override;
    bool hasCollision(AABB* col) override;
    bool hasCollision(Triangle* t) override;
    bool hasCollision(TriangleMesh* col) override;
    bool hasCollision(Ray* col) override;
    bool hasCollision(Capsule* col) override;
    bool hasCollision(OBB* col) override;

    void gui(int index) override;
    glm::mat4 getLocalTransform() override;

    Sphere(Mesh* mesh, bool createBody = true);
    Sphere(glm::vec3 pos = glm::vec3(0), float radius = 1, bool createBody = true);
    std::string toString() override;
    bool isInside(glm::vec3 point);
};

class AABB : public Collider{
public:
    glm::vec3 max{}, min{};

    glm::vec3 closestPoint(glm::vec3 p);

    static ColliderMesh* generateNewMesh();
    static std::vector<Vertex> getVerices(glm::vec3 min_, glm::vec3 max_);

    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;
    CollisionPoint checkCollision(OBB* col) override;

    bool hasCollision(Sphere* col) override;
    bool hasCollision(AABB* col) override;
    bool hasCollision(Triangle* t) override;
    bool hasCollision(TriangleMesh* col) override;
    bool hasCollision(Ray* col) override;
    bool hasCollision(Capsule* col) override;
    bool hasCollision(OBB* col) override;

    glm::vec3 getOffset();
    glm::vec3 getMin();
    glm::vec3 getMax();

    void refit(glm::vec3 min, glm::vec3 max);

    glm::mat4 getLocalTransform() override;
    glm::mat4 getTransform() override;

    void gui(int index) override;

    AABB(glm::vec3 min, glm::vec3 max, bool createBody = true);
    AABB(float height = 1, float width = 1, float length = 1, bool createBody = true);

    explicit AABB(Mesh* mesh, bool createBody = true);
    std::string toString() override;
    bool isInside(glm::vec3 point);
};

class OBB : public Collider {
public:
    glm::vec3 max{}, min{}, offset{};
    OBB(float height, float width, float length);

    //glm::vec3 getMin();
    //glm::vec3 getMax();

    std::vector<Vertex> getVerices();

    std::string toString() override;
    void gui(int index) override;

    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;
    CollisionPoint checkCollision(OBB* col) override;

    bool hasCollision(Sphere* col) override;
    bool hasCollision(AABB* col) override;
    bool hasCollision(Triangle* t) override;
    bool hasCollision(TriangleMesh* col) override;
    bool hasCollision(Ray* col) override;
    bool hasCollision(Capsule* col) override;
    bool hasCollision(OBB* col) override;
};

class TriangleMesh : public Collider {
public:
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(AABB* bv) override;
    CollisionPoint checkCollision(Sphere* bv) override;
    CollisionPoint checkCollision(Ray* r) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(Capsule* col) override;
    CollisionPoint checkCollision(OBB* col) override;

    bool hasCollision(Sphere* col) override;
    bool hasCollision(AABB* col) override;
    bool hasCollision(Triangle* t) override;
    bool hasCollision(TriangleMesh* col) override;
    bool hasCollision(Ray* col) override;
    bool hasCollision(Capsule* col) override;
    bool hasCollision(OBB* col) override;

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
    CollisionPoint checkCollision(OBB* col) override;

    bool hasCollision(Sphere* col) override;
    bool hasCollision(AABB* col) override;
    bool hasCollision(Triangle* t) override;
    bool hasCollision(TriangleMesh* col) override;
    bool hasCollision(Ray* col) override;
    bool hasCollision(Capsule* col) override;
    bool hasCollision(OBB* col) override;

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
    CollisionPoint checkCollision(OBB* col) override;

    bool hasCollision(Sphere* col) override;
    bool hasCollision(AABB* col) override;
    bool hasCollision(Triangle* t) override;
    bool hasCollision(TriangleMesh* col) override;
    bool hasCollision(Ray* col) override;
    bool hasCollision(Capsule* col) override;
    bool hasCollision(OBB* col) override;

    static Ray* generateRay(GLFWwindow* window, Camera* cam);
    Ray(glm::vec3 origin, glm::vec3 direction, float length, bool createMesh = false);
    std::string toString() override;
};

class Capsule : public Collider { // aka swept sphere
public:
    float radius;

    glm::vec3 start{}, end{};

    glm::vec3 getEnd();
    glm::vec3 getStart();

    void gui(int index) override;

    void createBody(glm::vec3 start, glm::vec3 end, float radius);
    Capsule (Mesh* mesh);
    Capsule(float length = 2, float radius = 0.5f);
    CollisionPoint checkCollision(Sphere* col) override;
    CollisionPoint checkCollision(AABB* col) override;
    CollisionPoint checkCollision(Triangle* t) override;
    CollisionPoint checkCollision(TriangleMesh* col) override;
    CollisionPoint checkCollision(Ray* col) override;
    CollisionPoint checkCollision(Capsule* col) override;
    CollisionPoint checkCollision(OBB* col) override;

    bool hasCollision(Sphere* col) override;
    bool hasCollision(AABB* col) override;
    bool hasCollision(Triangle* t) override;
    bool hasCollision(TriangleMesh* col) override;
    bool hasCollision(Ray* col) override;
    bool hasCollision(Capsule* col) override;
    bool hasCollision(OBB* col) override;

    std::string toString() override;


};
#endif //TRIANGLE_BoundingVolume_H
