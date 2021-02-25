//
// Created by Stefan on 22-Feb-21.
//

#include <ObjLoad.h>
#include "BoundingVolumes.h"
#include "Mesh.h"
#include "glm/gtx/string_cast.hpp"

#pragma region Functii Ovidiu

#define EPS 0.00001

float getEuclidianDistance(glm::vec3 p1, glm::vec3 p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
}

float getTriangleArea(float edge1, float edge2, float edge3) {
    float p = (edge1 + edge2 + edge3) / 2;

    return sqrt(p * (p - edge1) * (p - edge2) * (p - edge3));
}

struct Constants {
    float c1, c2, c3, c4;
};

Constants xIsNotConstant(glm::vec3 A, glm::vec3 B, BoundingSphere* bv) {
    Constants constants;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    constants.c1 = (y1 - y0) / (x1 - x0);
    constants.c2 = (x1 * y0 - x0 * y1 + x0 * bv->pos.y - x1 * bv->pos.y) / (x1 - x0);

    constants.c3 = (z1 - z0) / (x1 - x0);
    constants.c4 = (x1 * z0 - x0 * z1 + x0 * bv->pos.z - x1 * bv->pos.z) / (x1 - x0);

    return constants;
}

Constants yIsNotConstant(glm::vec3 A, glm::vec3 B, BoundingSphere* bv) {
    Constants constants;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    constants.c1 = (x1 - x0) / (y1 - y0);
    constants.c2 = (y1 * x0 - y0 * x1 + y0 * bv->pos.x - y1 * bv->pos.x) / (y1 - y0);

    constants.c3 = (z1 - z0) / (y1 - y0);
    constants.c4 = (y1 * z0 - y0 * z1 + y0 * bv->pos.z - y1 * bv->pos.z) / (y1 - y0);

    return constants;
}

Constants zIsNotConstant(glm::vec3 A, glm::vec3 B, BoundingSphere* bv) {
    Constants constants;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    constants.c1 = (x1 - x0) / (z1 - z0);
    constants.c2 = (z1 * x0 - z0 * x1 + z0 * bv->pos.x - z1 * bv->pos.x) / (z1 - z0);

    constants.c3 = (y1 - y0) / (z1 - z0);
    constants.c4 = (z1 * y0 - z0 * y1 + z0 * bv->pos.y - z1 * bv->pos.y) / (z1 - z0);

    return constants;
}

struct Solutions {
    float x, y, z;
};

Solutions xIsFixed(glm::vec3 A, glm::vec3 B, float x) {
    Solutions solutions;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solutions.x = x;
    solutions.y = (x - x0) * (y1 - y0) / (x1 - x0) + y0;
    solutions.z = (x - x0) * (z1 - z0) / (x1 - x0) + z0;

    return solutions;
}

Solutions yIsFixed(glm::vec3 A, glm::vec3 B, float y) {
    Solutions solutions;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solutions.x = (y - y0) * (x1 - x0) / (y1 - y0) + x0;
    solutions.y = y;
    solutions.z = (y - y0) * (z1 - z0) / (y1 - y0) + z0;

    return solutions;
}

Solutions zIsFixed(glm::vec3 A, glm::vec3 B, float z) {
    Solutions solutions;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solutions.x = (z - z0) * (x1 - x0) / (z1 - z0) + x0;
    solutions.y = (z - z0) * (y1 - y0) / (z1 - z0) + y0;
    solutions.z = z;

    return solutions;
}
#pragma endregion


bool Collider::checkCollision(Collider* col) {
    if (dynamic_cast<BoundingSphere*>(col)) {
        return checkCollision(dynamic_cast<BoundingSphere*>(col));
    }
    if (dynamic_cast<AABB*>(col)) {
        return checkCollision(dynamic_cast<AABB*>(col));
    }
    if (dynamic_cast<Ray*>(col)) {
        return checkCollision(dynamic_cast<Ray*>(col));
    }
    if (dynamic_cast<TriangleMesh*>(col)) {
        return checkCollision(dynamic_cast<TriangleMesh*>(col));
    }
    if (dynamic_cast<Triangle*>(col)) {
        return checkCollision(dynamic_cast<Triangle*>(col));
    }
    if (dynamic_cast<Capsule*>(col)) {
        return checkCollision(dynamic_cast<Capsule*>(col));
    }

    return false;
}

Collider::~Collider() {
    if (body)
        delete body;
}


BoundingSphere::BoundingSphere(Mesh *mesh) {
    pos = glm::vec3(0);

    for (Vertex v : mesh->vertices)
        pos = pos + v.Position;
    pos = pos / (float)mesh->vertices.size();
    for (auto v : mesh->vertices) {
        float currentRadius = glm::length(pos - v.Position);
        if (currentRadius > radius) radius = currentRadius;
    }
    body = readObj("Sphere.obj");
    for (int i=0; i<body->vertices.size(); i++)
        body->vertices[i].Position *= radius;
    body->prepare();
}
bool BoundingSphere::checkCollision(TriangleMesh *col) {
    return col->checkCollisionByTriangle(this);
}
bool BoundingSphere::checkCollision(AABB* bv) {
    glm::vec3 newMin = bv->min + bv->offset, newMax = bv->max + bv->offset;
    
    float x = std::max(newMin.x, std::min(this->pos.x, newMax.x));
    float y = std::max(newMin.y, std::min(this->pos.y, newMax.y));
    float z = std::max(newMin.z, std::min(this->pos.z, newMax.z));
    
    return (x - this->pos.x) * (x - this->pos.x) + (y - this->pos.y) * (y - this->pos.y) + (z - this->pos.z) * (z - this->pos.z) <= this->radius * this->radius;
}
bool BoundingSphere::checkCollision(BoundingSphere* bv) {
    float radius2radiusDistance = glm::length(pos - bv->pos);
    return radius2radiusDistance < radius || radius2radiusDistance < bv->radius;
}
bool BoundingSphere::checkCollision(Ray *r) {
    return r->checkCollision(this);
}
bool BoundingSphere::checkCollision(Triangle *t) {
    return t->checkCollision(this);
}
bool BoundingSphere::checkCollision(Capsule *col) {
    return false;
}
void BoundingSphere::setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) {
    //transform = glm::translate(glm::mat4(1), pos) * glm::scale(glm::mat4(1), scale);
    this->pos = pos;
    this->body->translation = pos;
}


AABB::AABB(Mesh* mesh) {
    min = mesh->vertices[0].Position;
    max = mesh->vertices[0].Position;
    for (auto v : mesh->vertices) {
        if (min.x > v.Position.x) min.x = v.Position.x;
        if (min.y > v.Position.y) min.y = v.Position.y;
        if (min.z > v.Position.z) min.z = v.Position.z;
        if (max.x < v.Position.x) max.x = v.Position.x;
        if (max.y < v.Position.y) max.y = v.Position.y;
        if (max.z < v.Position.z) max.z = v.Position.z;
    }
}
std::vector<glm::vec3> AABB::generateVerices(glm::mat4 transform) {
    std::vector<glm::vec3> vertices;
    glm::vec3 max, min;
    min = glm::vec3(transform * glm::vec4(this->min, 1.0f));
    max = glm::vec3(transform * glm::vec4(this->max, 1.0f));
    //std::cout << glm::to_string(min) << glm::to_string(max) << "\n";
    vertices.emplace_back(glm::vec3(min.x, min.y, min.z));
    vertices.emplace_back(glm::vec3(min.x, min.y, max.z));
    vertices.emplace_back(glm::vec3(min.x, max.y, min.z));
    vertices.emplace_back(glm::vec3(min.x, max.y, max.z));
    vertices.emplace_back(glm::vec3(max.x, min.y, min.z));
    vertices.emplace_back(glm::vec3(max.x, min.y, max.z));
    vertices.emplace_back(glm::vec3(max.x, max.y, min.z));
    vertices.emplace_back(glm::vec3(max.x, max.y, max.z));
    return vertices;
}
Mesh *AABB::generateNewMesh() {
    std::vector<glm::vec3> verticesPos = generateVerices(transform);
    body = new Mesh();
    for (auto v : verticesPos)
        body->vertices.push_back(Vertex{v, v});
    body->indices.push_back(3);
    body->indices.push_back(2);
    body->indices.push_back(0);

    body->indices.push_back(0);
    body->indices.push_back(1);
    body->indices.push_back(3);

    body->indices.push_back(6);
    body->indices.push_back(7);
    body->indices.push_back(4);

    body->indices.push_back(4);
    body->indices.push_back(7);
    body->indices.push_back(5);

    body->indices.push_back(7);
    body->indices.push_back(3);
    body->indices.push_back(5);

    body->indices.push_back(5);
    body->indices.push_back(3);
    body->indices.push_back(1);

    body->indices.push_back(2);
    body->indices.push_back(6);
    body->indices.push_back(0);

    body->indices.push_back(0);
    body->indices.push_back(6);
    body->indices.push_back(4);

    body->indices.push_back(0);
    body->indices.push_back(4);
    body->indices.push_back(1);

    body->indices.push_back(4);
    body->indices.push_back(5);
    body->indices.push_back(1);

    body->indices.push_back(2);
    body->indices.push_back(3);
    body->indices.push_back(7);

    body->indices.push_back(7);
    body->indices.push_back(6);
    body->indices.push_back(2);
    body->prepare();
    return body;
}
bool AABB::checkCollision(TriangleMesh *col) {
    return col->checkCollisionByTriangle(this);
}
bool AABB::checkCollision(AABB* bv) {
    glm::vec3 newMin1 = bv->min + bv->offset, newMax1 = bv->max + bv->offset;
    glm::vec3 newMin2 = this->min + this->offset, newMax2 = this->max + this->offset;

    if (newMin1.x > newMax2.x || newMax1.x < newMin2.x)
        return false;

    if (newMin1.y > newMax2.y || newMax1.y < newMin2.y)
        return false;

    if (newMin1.z > newMax2.z || newMax1.z < newMin2.z)
        return false;

    return true;
}
bool AABB::checkCollision(BoundingSphere* bv) {
    return bv->checkCollision(this);
}
bool AABB::checkCollision(Ray *r) {
    return r->checkCollision(this);
}
bool AABB::checkCollision(Triangle *t) {
    return t->checkCollision(this);
}
bool AABB::checkCollision(Capsule *col) {
    return col->checkCollision(this);
}
void AABB::setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) {
    Collider::setTransform(pos, rot, scale);
    if (body) delete body;
    body = generateNewMesh();
}


TriangleMesh::TriangleMesh(Mesh *mesh) {
    body = mesh;
}
bool TriangleMesh::checkCollision(TriangleMesh *col) {
    return checkCollisionByTriangle(col);
}
bool TriangleMesh::checkCollision(AABB *bv) {
    return checkCollisionByTriangle(bv);
}
bool TriangleMesh::checkCollision(BoundingSphere *bv) {
    return checkCollisionByTriangle(bv);
}
bool TriangleMesh::checkCollision(Ray *r) {
    return checkCollisionByTriangle(r);
}
bool TriangleMesh::checkCollision(Triangle *t) {
    return checkCollisionByTriangle(t);
}
bool TriangleMesh::checkCollisionByTriangle(Collider *col) {
    bool result = false;
    for (int i = 0; i < this->body->indices.size(); i+=3) {
        Triangle t(
                glm::vec3(this->transform * glm::vec4(body->vertices[i].Position, 1.0f)),
                glm::vec3(this->transform * glm::vec4(body->vertices[i+1].Position, 1.0f)),
                glm::vec3(this->transform * glm::vec4(body->vertices[i+2].Position, 1.0f)),
                body->vertices[i].Normal
                );
        result = col->checkCollision(&t);
        if (result) return true;
    }
    return false;
}
bool TriangleMesh::checkCollision(Capsule *col) {
    return checkCollisionByTriangle(col);
}


Ray* Ray::generateRay(GLFWwindow* window, Camera* cam) {
    static bool wasHeld = false;
     if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE && wasHeld) {
        int wi, he;
        glfwGetWindowSize(window, &wi, &he);
        float x = cam->lastx_mouse - (float) wi / 2.0f;
        float y = he - cam->lasty_mouse - (float) he / 2.0f;

        x /= (float) wi / 2.0f;
        y /= (float) he / 2.0f;

        float aspectRatio = (float) wi / (float) he;

        glm::vec3 origin = cam->position;
        /* (cam->goFront
                                            + glm::tan(glm::radians(cam->fovy) / 2.0f) * aspectRatio * x * glm::normalize(cam->goRight)
                                            + glm::tan(glm::radians(cam->fovy) / 2.0f) * y * glm::normalize(cam->goUp)
                                            );
        */
        wasHeld = false;
        return new Ray(origin, glm::normalize(cam->goFront), 100);
    }

    if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) { wasHeld = true; }

    return nullptr;
}
Ray::Ray(glm::vec3 origin, glm::vec3 direction, float length) {
        this->origin = origin;
        this->direction = direction;
        this->length = length;
        body = new Mesh();
        body->vertices.push_back(Vertex{origin});
        body->vertices.push_back(Vertex{origin + direction * length});
        body->indices.push_back(0);
        body->indices.push_back(1);
        body->indices.push_back(0);
        body->solidON = false;
        body->wireframeON = true;
        body->prepare();
}
bool Ray::checkCollision(TriangleMesh *col) {
    return col->checkCollisionByTriangle(this);
}
bool Ray::checkCollision(BoundingSphere* bv) {
    glm::vec3 A = this->origin, B = this->origin - this->direction;

    Constants constants;

    // Pick an axis which the line is not parallel to
    if (A.x != B.x)
        constants = xIsNotConstant(A, B, bv);
    else if (A.y != B.y)
        constants = yIsNotConstant(A, B, bv);
    else
        constants = zIsNotConstant(A, B, bv);

    float c1 = constants.c1, c2 = constants.c2;
    float c3 = constants.c3, c4 = constants.c4;

    // The points of intersection are the solutions of the quadratic equation
    float pinnedSphereOffset;

    if (A.x != B.x)
        pinnedSphereOffset = bv->pos.x;
    else if (A.y != B.y)
        pinnedSphereOffset = bv->pos.y;
    else
        pinnedSphereOffset = bv->pos.z;

    float a = 1 + c1 * c1 + c3 * c3;
    float b = -2 * (pinnedSphereOffset - c1 * c2 - c3 * c4);
    float c = pinnedSphereOffset * pinnedSphereOffset + c2 * c2 + c4 * c4 - bv->radius * bv->radius;

    float delta = b * b - 4 * a * c;

    if (delta < 0)
        return false;

    if (delta == 0) {
        Solutions solutions;

        if (A.x != B.x)
            solutions = xIsFixed(A, B, -b / (2 * a));
        else if (A.y != B.y)
            solutions = yIsFixed(A, B, -b / (2 * a));
        else
            solutions = zIsFixed(A, B, -b / (2 * a));

        // Check the distance between the ray's origin and the unique point
        if (getEuclidianDistance(glm::vec3(solutions.x, solutions.y, solutions.z), this->origin) <= this->length)
            return true;
    } else {
        Solutions solutions;

        if (A.x != B.x)
            solutions = xIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));
        else if (A.y != B.y)
            solutions = yIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));
        else
            solutions = zIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));

        // Check the distance between the ray's origin and the first point
        if (getEuclidianDistance(glm::vec3(solutions.x, solutions.y, solutions.z), this->origin) <= this->length)
            return true;

        if (A.x != B.x)
            solutions = xIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
        else if (A.y != B.y)
            solutions = yIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
        else
            solutions = zIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));

        // Check the distance between the ray's origin and the second point
        if (getEuclidianDistance(glm::vec3(solutions.x, solutions.y, solutions.z), this->origin) <= this->length)
            return true;
    }

    return false;
}
bool Ray::checkCollision(AABB* bv) {
    glm::vec3 newMin = bv->min + bv->offset, newMax = bv->max + bv->offset;
    glm::vec3 A = this->origin, B = this->origin - this->direction;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    float x, y, z;

    // Front face
    x = (newMax.z * x1 - newMax.z * x0 - x1 * z0 + x0 * z1) / (z1 - z0);
    y = (newMax.z * y1 - newMax.z * y0 - y1 * z0 + y0 * z1) / (z1 - z0);

    if (z0 != z1 && x >= newMin.x && x <= newMax.x && y >= newMin.y && y <= newMax.y)
        if (getEuclidianDistance(glm::vec3(x, y, newMax.z), this->origin) <= this->length)
            return true;

    // Back face
    x = (newMin.z * x1 - newMin.z * x0 - x1 * z0 + x0 * z1) / (z1 - z0);
    y = (newMin.z * y1 - newMin.z * y0 - y1 * z0 + y0 * z1) / (z1 - z0);

    if (z0 != z1 && x >= newMin.x && x <= newMax.x && y >= newMin.y && y <= newMax.y)
        if (getEuclidianDistance(glm::vec3(x, y, newMin.z), this->origin) <= this->length)
            return true;

    // Left face
    y = (newMin.x * y1 - newMin.x * y0 - y1 * x0 + y0 * x1) / (x1 - x0);
    z = (newMin.x * z1 - newMin.x * z0 - z1 * x0 + z0 * x1) / (x1 - x0);

    if (x0 != x1 && y >= newMin.y && y <= newMax.y && z >= newMin.z && z <= newMax.z)
        if (getEuclidianDistance(glm::vec3(newMin.x, y, z), this->origin) <= this->length)
            return true;

    // Right face
    y = (newMax.x * y1 - newMax.x * y0 - y1 * x0 + y0 * x1) / (x1 - x0);
    z = (newMax.x * z1 - newMax.x * z0 - z1 * x0 + z0 * x1) / (x1 - x0);

    if (x0 != x1 && y >= newMin.y && y <= newMax.y && z >= newMin.z && z <= newMax.z)
        if (getEuclidianDistance(glm::vec3(newMax.x, y, z), this->origin) <= this->length)
            return true;

    // Bottom face
    x = (newMin.y * x1 - newMin.y * x0 - x1 * y0 + x0 * y1) / (y1 - y0);
    z = (newMin.y * z1 - newMin.y * z0 - z1 * y0 + z0 * y1) / (y1 - y0);

    if (y0 != y1 && x >= newMin.x && x <= newMax.x && z >= newMin.z && z <= newMax.z)
        if (getEuclidianDistance(glm::vec3(x, newMin.y, z), this->origin) <= this->length)
            return true;

    // Up face
    x = (newMax.y * x1 - newMax.y * x0 - x1 * y0 + x0 * y1) / (y1 - y0);
    z = (newMax.y * z1 - newMax.y * z0 - z1 * y0 + z0 * y1) / (y1 - y0);

    if (y0 != y1 && x >= newMin.x && x <= newMax.x && z >= newMin.z && z <= newMax.z)
        if (getEuclidianDistance(glm::vec3(x, newMax.y, z), this->origin) <= this->length)
            return true;

    return false;
}
// TODO
bool Ray::checkCollision(Ray* r) {
    return false;
}
bool Ray::checkCollision(Triangle *t) {
    glm::vec3 A = this->origin, B = this->origin - this->direction;

    if (glm::dot(t->norm, B) < 0)
        return false;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    float a = t->norm.x, b = t->norm.y, c = t->norm.z;
    float d = t->norm.x * t->vertices[0].x + t->norm.y * t->vertices[0].y + t->norm.z * t->vertices[0].z;

    float c1, c2, c3, c4;
    Solutions solutions;

    // Pick an axis which the line is not parallel to
    if (A.x != B.x) {
        c1 = b * (y1 - y0) / (x1 - x0);
        c2 = b * (x1 * y0 - x0 * y1) / (x1 - x0);

        c3 = c * (z1 - z0) / (x1 - x0);
        c4 = c * (x1 * z0 - x0 * z1) / (x1 - x0);

        if (!(a + c1 + c3))
            return false;

        solutions = xIsFixed(A, B, (d - c2 - c4) / (a + c1 + c3));
    } else if (A.y != B.y) {
        c1 = a * (x1 - x0) / (y1 - y0);
        c2 = a * (y1 * x0 - y0 * x1) / (y1 - y0);

        c3 = c * (z1 - z0) / (y1 - y0);
        c4 = c * (y1 * z0 - y0 * z1) / (y1 - y0);

        if (!(a + c1 + c3))
            return false;

        solutions = yIsFixed(A, B, (d - c2 - c4) / (a + c1 + c3));
    } else {
        c1 = a * (x1 - x0) / (z1 - z0);
        c2 = a * (z1 * x0 - z0 * x1) / (z1 - z0);

        c3 = c * (y1 - y0) / (z1 - z0);
        c4 = c * (z1 * y0 - z0 * y1) / (z1 - z0);

        if (!(a + c1 + c3))
            return false;

        solutions = zIsFixed(A, B, (d - c2 - c4) / (a + c1 + c3));
    }

    if (getEuclidianDistance(glm::vec3(solutions.x, solutions.y, solutions.z), this->origin) > this->length)
        return false;

    // Idea based on barycentric coordinates
    float edge1 = getEuclidianDistance(t->vertices[0], t->vertices[1]);
    float edge2 = getEuclidianDistance(t->vertices[1], t->vertices[2]);
    float edge3 = getEuclidianDistance(t->vertices[0], t->vertices[2]);

    float edge4 = getEuclidianDistance(t->vertices[0], glm::vec3(solutions.x, solutions.y, solutions.z));
    float edge5 = getEuclidianDistance(t->vertices[1], glm::vec3(solutions.x, solutions.y, solutions.z));
    float edge6 = getEuclidianDistance(t->vertices[2], glm::vec3(solutions.x, solutions.y, solutions.z));

    float subarea1 = getTriangleArea(edge1, edge4, edge5);
    float subarea2 = getTriangleArea(edge2, edge5, edge6);
    float subarea3 = getTriangleArea(edge3, edge4, edge6);

    return fabs(subarea1 + subarea2 + subarea3 - getTriangleArea(edge1, edge2, edge3)) <= EPS;
}
bool Ray::checkCollision(Capsule *col) {
    return col->checkCollision(this);
}


Triangle::Triangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 norm) {
    this->vertices[0] = v0;
    this->vertices[1] = v1;
    this->vertices[2] = v2;
    this->norm = norm;
}
bool Triangle::checkCollision(TriangleMesh *col) {
    return col->checkCollisionByTriangle(this);
}
bool Triangle::checkCollision(Ray *r) {
    return r->checkCollision(this);
}
bool Triangle::checkCollision(Capsule *col) {
    return col->checkCollision(this);
}
// TODO
bool Triangle::checkCollision(AABB *bv) {
    float p0, p1, p2, r;
    // Compute box center and extents (if not already given in that format)
    glm::vec3 c = (bv->min + bv->max) * 0.5f;
    float e0 = (bv->max.x - bv->min.x) * 0.5f;
    float e1 = (bv->max.y - bv->min.y) * 0.5f;
    float e2 = (bv->max.z - bv->min.z) * 0.5f;

    glm::vec3 v0, v1, v2;
    v0 = vertices[0];
    v1 = vertices[1];
    v2 = vertices[2];

    v0 = v0 - c;
    v1 = v1 - c;
    v2 = v2 - c;

    return false;
}
// TODO
bool Triangle::checkCollision(BoundingSphere *bv) {
    return false;
}
// TODO
bool Triangle::checkCollision(Triangle *t) {
    return false;
}


Capsule::Capsule(glm::vec3 start, glm::vec3 end, float radius) {
    this->start = start;
    this->end = end;
    this->radius = radius;

    Mesh* Tube = readObj("Tube.obj");
    Mesh* halfSphere = readObj("halfSphere.obj");

    body = new Mesh();

    body->vertices = Tube->vertices;
    body->indices = Tube->indices;

    float diff = glm::length(end - start);

    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1), glm::vec3(radius));

    glm::mat4 modelEnd = glm::translate(glm::mat4(1), glm::vec3(0, diff/2, 0)) * scaleMatrix;
    glm::mat4 modelStart = modelEnd * glm::rotate(glm::mat4(1), glm::radians(180.0f), glm::vec3(1, 0, 0)) * scaleMatrix;

    size_t indicesCount = Tube->indices.size();
    for (int j = 0; j < halfSphere->indices.size(); ++j) {
        body->indices.push_back(halfSphere->indices[j] + indicesCount);
    }
    for (int i = 0; i < halfSphere->vertices.size(); ++i) {
        halfSphere->vertices[i].Position = glm::vec3(modelStart * glm::vec4(halfSphere->vertices[i].Position, 1.0f));
        body->vertices.push_back(halfSphere->vertices[i]);
    }
    for (int i = 0; i < halfSphere->vertices.size(); ++i) {
        halfSphere->vertices[i].Position = glm::vec3(modelEnd * glm::vec4(halfSphere->vertices[i].Position, 1.0f));
        body->vertices.push_back(halfSphere->vertices[i]);
    }
    indicesCount = body->indices.size();
    for (int j = 0; j < halfSphere->indices.size(); ++j) {
        body->indices.push_back(halfSphere->indices[j] + indicesCount);
    }
    delete Tube;
    delete halfSphere;

    body->prepare();
}
Capsule *Capsule::generateCapsule(Mesh *mesh) {
    glm::vec3 min, max;
    min = mesh->vertices[0].Position;
    max = mesh->vertices[0].Position;
    for (auto v : mesh->vertices) {
        if (min.x > v.Position.x) min.x = v.Position.x;
        if (min.y > v.Position.y) min.y = v.Position.y;
        if (min.z > v.Position.z) min.z = v.Position.z;
        if (max.x < v.Position.x) max.x = v.Position.x;
        if (max.y < v.Position.y) max.y = v.Position.y;
        if (max.z < v.Position.z) max.z = v.Position.z;
    }
    return new Capsule(min, max, 1);
}
// TODO
bool Capsule::checkCollision(TriangleMesh *col) {
    return false;
}
// TODO
bool Capsule::checkCollision(BoundingSphere *col) {
    return false;
}
// TODO
bool Capsule::checkCollision(AABB *col) {
    return false;
}
// TODO
bool Capsule::checkCollision(Triangle *t) {
    return false;
}
// TODO
bool Capsule::checkCollision(Ray *col) {
    return false;
}
// TODO
bool Capsule::checkCollision(Capsule *col) {
    return false;
}



