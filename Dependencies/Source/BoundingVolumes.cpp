//
// Created by Stefan on 22-Feb-21.
//

#include <ObjLoad.h>
#include "BoundingVolumes.h"
#include "Mesh.h"
#include "glm/gtx/string_cast.hpp"

#pragma region Functii Ovidiu

#define EPS 0.00001
#define INF 2000000000

float getEuclidianDistance(glm::vec3 p1, glm::vec3 p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
}

float getTriangleArea(float edge1, float edge2, float edge3) {
    float p = (edge1 + edge2 + edge3) / 2;

    return sqrt(p * (p - edge1) * (p - edge2) * (p - edge3));
}

struct Solution {
    float x, y, z;
};

Solution xIsFixed(glm::vec3 A, glm::vec3 B, float x) {
    Solution solution;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.x = x;
    solution.y = (x - x0) * (y1 - y0) / (x1 - x0) + y0;
    solution.z = (x - x0) * (z1 - z0) / (x1 - x0) + z0;

    return solution;
}

Solution yIsFixed(glm::vec3 A, glm::vec3 B, float y) {
    Solution solution;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.x = (y - y0) * (x1 - x0) / (y1 - y0) + x0;
    solution.y = y;
    solution.z = (y - y0) * (z1 - z0) / (y1 - y0) + z0;

    return solution;
}

Solution zIsFixed(glm::vec3 A, glm::vec3 B, float z) {
    Solution solution;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.x = (z - z0) * (x1 - x0) / (z1 - z0) + x0;
    solution.y = (z - z0) * (y1 - y0) / (z1 - z0) + y0;
    solution.z = z;

    return solution;
}

Solution getIntersectionPoint(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 D) {
    Solution solution;

    float c1, c2;

    // Pick an axis which both lines are not parallel to
    if (A.x != B.x && C.x != D.x) {
        c1 = (B.y - A.y) / (B.x - A.x);
        c2 = (A.y * B.x - A.x * B.y) / (B.x - A.x);

        float a = (C.x * D.y + c2 * D.x - c2 * C.x - C.y * D.x);
        float b = (D.y - C.y - c1 * D.x + c1 * C.x);

        if (!b) {
            if (a)
                solution.x = solution.y = solution.z = INF;
            else
                solution.x = solution.y = solution.z = -INF;

            return solution;
        }

        solution = xIsFixed(A, B, a / b);
    } else if (A.y != B.y && C.y != D.y) {
        c1 = (B.x - A.x) / (B.y - A.y);
        c2 = (A.x * B.y - A.y * B.x) / (B.y - A.y);

        float a = (C.y * D.x + c2 * D.y - c2 * C.y - C.x * D.y);
        float b = (D.x - C.x - c1 * D.y + c1 * C.y);

        if (!b) {
            if (a)
                solution.x = solution.y = solution.z = INF;
            else
                solution.x = solution.y = solution.z = -INF;

            return solution;
        }

        solution = yIsFixed(A, B, a / b);
    } else {
        c1 = (B.x - A.x) / (B.z - A.z);
        c2 = (A.x * B.z - A.z * B.x) / (B.z - A.z);

        float a = (C.z * D.x + c2 * D.z - c2 * C.z - C.x * D.z);
        float b = (D.x - C.x - c1 * D.z + c1 * C.z);

        if (!b) {
            if (a)
                solution.x = solution.y = solution.z = INF;
            else
                solution.x = solution.y = solution.z = -INF;

            return solution;
        }

        solution = zIsFixed(A, B, a / b);
    }

    return solution;
}
#pragma endregion

#pragma region Functii Stefan
glm::vec3 getTransformedVertex(glm::mat4 tr, glm::vec3 v) {
    return glm::vec3(tr * glm::vec4(v, 1.0f));
}
glm::vec3 ClosestPointOnLineSegment(glm::vec3 A, glm::vec3 B, glm::vec3 Point)
{
    glm::vec3 AB = B - A;
    float t = glm::dot(Point - A, AB) / glm::dot(AB, AB);
    return A + glm::min(glm::max(t, 0.0f), 1.0f) * AB; // saturate(t) can be written as: min((max(t, 0), 1)
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


BoundingSphere::BoundingSphere(glm::vec3 pos, float radius) {
    this->pos = pos;
    this->radius = radius;
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
    return radius2radiusDistance <= radius + bv->radius;
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
    min0 = mesh->vertices[0].Position;
    max0 = mesh->vertices[0].Position;



    for (auto v : mesh->vertices) {
        if (min0.x > v.Position.x) min0.x = v.Position.x;
        if (min0.y > v.Position.y) min0.y = v.Position.y;
        if (min0.z > v.Position.z) min0.z = v.Position.z;
        if (max0.x < v.Position.x) max0.x = v.Position.x;
        if (max0.y < v.Position.y) max0.y = v.Position.y;
        if (max0.z < v.Position.z) max0.z = v.Position.z;
    }

    min = min0;
    max = max0;
}
std::vector<glm::vec3> AABB::generateVerices(glm::vec3 min_, glm::vec3 max_) {
    std::vector<glm::vec3> vertices;
    vertices.emplace_back(glm::vec3(min_.x, min_.y, min_.z));
    vertices.emplace_back(glm::vec3(min_.x, min_.y, max_.z));
    vertices.emplace_back(glm::vec3(min_.x, max_.y, min_.z));
    vertices.emplace_back(glm::vec3(min_.x, max_.y, max_.z));
    vertices.emplace_back(glm::vec3(max_.x, min_.y, min_.z));
    vertices.emplace_back(glm::vec3(max_.x, min_.y, max_.z));
    vertices.emplace_back(glm::vec3(max_.x, max_.y, min_.z));
    vertices.emplace_back(glm::vec3(max_.x, max_.y, max_.z));
    return vertices;
}
Mesh *AABB::generateNewMesh() {
    std::vector<glm::vec3> verticesPos = generateVerices(min, max);

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
void AABB::setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) {
    Collider::setTransform(pos, rot, scale);
    if (body) delete body;
    updateMinMax(transform);
    body = generateNewMesh();
}
void AABB::updateMinMax(glm::mat4 transform) {
    std::vector<glm::vec3> verticesPos = generateVerices(min0, max0);
    for (int i = 0; i < verticesPos.size(); ++i)
        verticesPos[i] = getTransformedVertex(transform, verticesPos[i]);

    min = verticesPos[0];
    max = verticesPos[0];

    for (int i = 1; i < verticesPos.size(); ++i) {
        min = glm::min(min, verticesPos[i]);
        max = glm::max(max, verticesPos[i]);
    }
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
AABB::AABB(glm::vec3 min, glm::vec3 max) {
    min0 = min;
    max0 = max;
    offset = glm::vec3(0);
    transform = glm::mat4(1);
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
                getTransformedVertex(transform, body->vertices[i].Position),
                getTransformedVertex(transform, body->vertices[i+1].Position),
                getTransformedVertex(transform, body->vertices[i+2].Position),
                glm::mat3(transpose(inverse(transform))) * body->vertices[i].Normal
                );
        result = col->checkCollision(&t);
        if (result) return true;
    }
    return false;
}
bool TriangleMesh::checkCollision(Capsule *col) {
    return checkCollisionByTriangle(col);
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
bool Ray::checkCollision(TriangleMesh *col) {
    return col->checkCollisionByTriangle(this);
}
bool Ray::checkCollision(BoundingSphere* bv) {
    glm::vec3 A = this->origin, B = this->origin + this->direction;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    float c1, c2, c3, c4;

    // Pick an axis which the line is not parallel to
    if (A.x != B.x) {
        c1 = (y1 - y0) / (x1 - x0);
        c2 = (x1 * y0 - x0 * y1 + x0 * bv->pos.y - x1 * bv->pos.y) / (x1 - x0);

        c3 = (z1 - z0) / (x1 - x0);
        c4 = (x1 * z0 - x0 * z1 + x0 * bv->pos.z - x1 * bv->pos.z) / (x1 - x0);
    } else if (A.y != B.y) {
        c1 = (x1 - x0) / (y1 - y0);
        c2 = (y1 * x0 - y0 * x1 + y0 * bv->pos.x - y1 * bv->pos.x) / (y1 - y0);

        c3 = (z1 - z0) / (y1 - y0);
        c4 = (y1 * z0 - y0 * z1 + y0 * bv->pos.z - y1 * bv->pos.z) / (y1 - y0);
    } else {
        c1 = (x1 - x0) / (z1 - z0);
        c2 = (z1 * x0 - z0 * x1 + z0 * bv->pos.x - z1 * bv->pos.x) / (z1 - z0);

        c3 = (y1 - y0) / (z1 - z0);
        c4 = (z1 * y0 - z0 * y1 + z0 * bv->pos.y - z1 * bv->pos.y) / (z1 - z0);
    }

    // The points of intersection are the solution of the quadratic equation
    float pinnedSphereOffset;

    if (x0 != x1)
        pinnedSphereOffset = bv->pos.x;
    else if (y0 != y1)
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
        Solution solution;

        if (A.x != B.x)
            solution = xIsFixed(A, B, -b / (2 * a));
        else if (A.y != B.y)
            solution = yIsFixed(A, B, -b / (2 * a));
        else
            solution = zIsFixed(A, B, -b / (2 * a));

        // Check the distance between the ray's origin and the unique point
        float distance1 = getEuclidianDistance(A, glm::vec3(solution.x, solution.y, solution.z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(solution.x, solution.y, solution.z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;
    } else {
        Solution solution;

        if (x0 != x1)
            solution = xIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));
        else if (y0 != y1)
            solution = yIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));
        else
            solution = zIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));

        // Check the distance between the ray's origin and the first point
        float distance1 = getEuclidianDistance(A, glm::vec3(solution.x, solution.y, solution.z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(solution.x, solution.y, solution.z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;

        if (x0 != x1)
            solution = xIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
        else if (y0 != y1)
            solution = yIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
        else
            solution = zIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));

        // Check the distance between the ray's origin and the second point
        float distance1 = getEuclidianDistance(A, glm::vec3(solution.x, solution.y, solution.z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(solution.x, solution.y, solution.z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
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

    if (z0 != z1 && x >= newMin.x && x <= newMax.x && y >= newMin.y && y <= newMax.y) {
        float distance1 = getEuclidianDistance(A, glm::vec3(x, y, newMax.z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(x, y, newMax.z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;
    }

    // Back face
    x = (newMin.z * x1 - newMin.z * x0 - x1 * z0 + x0 * z1) / (z1 - z0);
    y = (newMin.z * y1 - newMin.z * y0 - y1 * z0 + y0 * z1) / (z1 - z0);

    if (z0 != z1 && x >= newMin.x && x <= newMax.x && y >= newMin.y && y <= newMax.y) {
        float distance1 = getEuclidianDistance(A, glm::vec3(x, y, newMin.z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(x, y, newMin.z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;
    }

    // Left face
    y = (newMin.x * y1 - newMin.x * y0 - y1 * x0 + y0 * x1) / (x1 - x0);
    z = (newMin.x * z1 - newMin.x * z0 - z1 * x0 + z0 * x1) / (x1 - x0);

    if (x0 != x1 && y >= newMin.y && y <= newMax.y && z >= newMin.z && z <= newMax.z) {
        float distance1 = getEuclidianDistance(A, glm::vec3(newMin.x, y, z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(newMin.x, y, z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;
    }

    // Right face
    y = (newMax.x * y1 - newMax.x * y0 - y1 * x0 + y0 * x1) / (x1 - x0);
    z = (newMax.x * z1 - newMax.x * z0 - z1 * x0 + z0 * x1) / (x1 - x0);

    if (x0 != x1 && y >= newMin.y && y <= newMax.y && z >= newMin.z && z <= newMax.z) {
        float distance1 = getEuclidianDistance(A, glm::vec3(newMax.x, y, z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(newMax.x, y, z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;
    }

    // Bottom face
    x = (newMin.y * x1 - newMin.y * x0 - x1 * y0 + x0 * y1) / (y1 - y0);
    z = (newMin.y * z1 - newMin.y * z0 - z1 * y0 + z0 * y1) / (y1 - y0);

    if (y0 != y1 && x >= newMin.x && x <= newMax.x && z >= newMin.z && z <= newMax.z) {
        float distance1 = getEuclidianDistance(A, glm::vec3(x, newMin.y, z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(x, newMin.y, z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;
    }

    // Up face
    x = (newMax.y * x1 - newMax.y * x0 - x1 * y0 + x0 * y1) / (y1 - y0);
    z = (newMax.y * z1 - newMax.y * z0 - z1 * y0 + z0 * y1) / (y1 - y0);

    if (y0 != y1 && x >= newMin.x && x <= newMax.x && z >= newMin.z && z <= newMax.z) {
        float distance1 = getEuclidianDistance(A, glm::vec3(x, newMax.y, z));
        float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(x, newMax.y, z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;
    }

    return false;
}

/// to do Ray - Ray
bool Ray::checkCollision(Triangle *t, glm::vec3 C, glm::vec3 D) {
    glm::vec3 A = this->origin, B = this->origin + this->direction;

    Solution solution = getIntersectionPoint(A, B, C, D);

    float distance1 = getEuclidianDistance(A, glm::vec3(solution.x, solution.y, solution.z));
    float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(solution.x, solution.y, solution.z));

    if (solution.x != -INF &&
        solution.x != INF && fabs(distance1 + distance2 - this->length) <= EPS &&
        t->isInside(glm::vec3(solution.x, solution.y, solution.z)))
            return true;

    return false;
}
///

// TODO
/*
 * O1 + t * (O1 - D1) = v
 * O2 + t * (O2 - D2) = v
 *  ==> O1 + t * (O1 - D1) = O2 + t * (O2 - D2)
 *  t = (O1 - O2)/((O2 - D2) - (O1 - D1))
 * */
bool Ray::checkCollision(Ray* r) {
    glm::dvec3 O1, O2, D1, D2;
    O1 = r->origin;
    D1 = r->origin - r->direction * r->length;

    O2 = origin;
    D2 = origin - direction * length;

    glm::dvec3 t =  (O1 - O2) / ((O2 - D2) - (O1 - D1));

    if (glm::isnan(t.z))
        return glm::pow(t.x - t.y, 2) < EPS && t.x <= 1 && t.x >=0;

    if (glm::isnan(t.y))
        return glm::pow(t.x - t.z, 2) < EPS && t.x <= 1 && t.x >=0;

    if (glm::isnan(t.x))
        return glm::pow(t.z - t.y, 2) < EPS && t.z <= 1 && t.z >=0;

    return t.x - t.y < EPS && t.x - t.z < EPS && t.x <= 1 && t.x >=0;
}
bool Ray::checkCollision(Triangle *t) {
    glm::vec3 A = this->origin, B = this->origin + this->direction;

    if (glm::dot(t->norm, B) < 0)
        return false;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;
    
    float a = t->norm.x, b = t->norm.y, c = t->norm.z;
    float d = glm::dot(t->norm, t->vertices[0]);
    
    float c1, c2, c3, c4;
    Solution solution;
    
    // Pick an axis which the line is not parallel to
    if (x0 != x1) {
        c1 = b * (y1 - y0) / (x1 - x0);
        c2 = b * (x1 * y0 - x0 * y1) / (x1 - x0);
    
        c3 = c * (z1 - z0) / (x1 - x0);
        c4 = c * (x1 * z0 - x0 * z1) / (x1 - x0);

        if (!(a + c1 + c3)) {
            // No intersection at all
            if (d - c2 - c4)
                return false;

            // The intersection is an infinite number on points
            if (t->isInside(A) || t->isInside(B))
                return true;

            return this->checkCollision(t, t->vertices[0], t->vertices[1]) ||
                   this->checkCollision(t, t->vertices[1], t->vertices[2]) ||
                   this->checkCollision(t, t->vertices[0], t->vertices[2]);
        }
           
        solution = xIsFixed(A, B, (d - c2 - c4) / (a + c1 + c3));
    } else if (y0 != y1) {
        c1 = a * (x1 - x0) / (y1 - y0);
        c2 = a * (y1 * x0 - y0 * x1) / (y1 - y0);
    
        c3 = c * (z1 - z0) / (y1 - y0);
        c4 = c * (y1 * z0 - y0 * z1) / (y1 - y0);
    
        if (!(a + c1 + c3)) {
            // No intersection at all
            if (d - c2 - c4)
                return false;

            // The intersection is an infinite number on points
            if (t->isInside(A) || t->isInside(B))
                return true;

            return this->checkCollision(t, t->vertices[0], t->vertices[1]) ||
                   this->checkCollision(t, t->vertices[1], t->vertices[2]) ||
                   this->checkCollision(t, t->vertices[0], t->vertices[2]);
        }

        solution = yIsFixed(A, B, (d - c2 - c4) / (a + c1 + c3));
    } else {
        c1 = a * (x1 - x0) / (z1 - z0);
        c2 = a * (z1 * x0 - z0 * x1) / (z1 - z0);
    
        c3 = c * (y1 - y0) / (z1 - z0);
        c4 = c * (z1 * y0 - z0 * y1) / (z1 - z0);
    
        if (!(a + c1 + c3)) {
            // No intersection at all
            if (d - c2 - c4)
                return false;

            // The intersection is an infinite number on points
            if (t->isInside(A) || t->isInside(B))
                return true;

            return this->checkCollision(t, t->vertices[0], t->vertices[1]) ||
                   this->checkCollision(t, t->vertices[1], t->vertices[2]) ||
                   this->checkCollision(t, t->vertices[0], t->vertices[2]);
        }

        solution = zIsFixed(A, B, (d - c2 - c4) / (a + c1 + c3));
    }

    float distance1 = getEuclidianDistance(A, glm::vec3(solution.x, solution.y, solution.z));
    float distance2 = getEuclidianDistance(A + this->direction * this->length, glm::vec3(solution.x, solution.y, solution.z));
    
    if (distance1 > this->length || distance2 > this->length)
        return false;
    
    return t->isInside(glm::vec3(solution.x, solution.y, solution.z));
    
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
// TODO https://gdbooks.gitbooks.io/3dcollisions/content/Chapter4/aabb-triangle.html
bool Triangle::checkCollision(AABB *bv) {
    return false;
}
// TODO https://wickedengine.net/2020/04/26/capsule-collision-detection/
bool Triangle::checkCollision(BoundingSphere *bv) {
    return false;
}
// TODO https://gdbooks.gitbooks.io/3dcollisions/content/Chapter4/triangle-triangle.html
bool Triangle::checkCollision(Triangle *t) {
    bool co_planar = true;

    float d2 = -glm::dot(t->norm, t->vertices[0]);
    float d1 = -glm::dot(t->norm, vertices[0]);

    float d1_[3];
    float d2_[3];


    {
        bool
                positiveDist = false,
                negativeDist = false;

        for (int i = 0; i < 3; ++i) {
            d1_[i] = glm::dot(t->norm, vertices[i]) + d2;
            d2_[i] = glm::dot(t->norm, t->vertices[i]) + d1;

            if (d1_[i] != 0) co_planar = false;
            if (d1_[i] < 0) negativeDist = true;
                else if (d1_[i] > 0) positiveDist = true;
        }

        if (!negativeDist || !positiveDist)
            return false;

    }

    float p1[3], p2[3];
    if (!co_planar) {
        glm::vec3 O = vertices[0];
        glm::vec3 D = glm::cross(norm, t->norm);

        glm::vec3 t1, t2, v0, v1;
        float d_0, d_1;

        if (d1_[0] * d2_[1] < 0) {
            v0 = vertices[0];
            v1 = vertices[1];
            d_0 = d1_[0];
            d_1 = d1_[1];
        } else {
            v0 = vertices[0];
            v1 = vertices[2];
            d_0 = d1_[0];
            d_1 = d1_[2];
        }

        t1 = D * ((v0 - O) + (v1 - v0) * d_0 / (d_0 - d_1));


        if (d1_[2] * d2_[1] < 0) {
            v0 = vertices[2];
            v1 = vertices[1];
            d_0 = d1_[2];
            d_1 = d1_[1];
        } else {
            v0 = vertices[2];
            v1 = vertices[1];
            d_0 = d1_[2];
            d_1 = d1_[1];
        }

        t2 = D * ((v0 - O) + (v1 - v0) * d_0 / (d_0 - d_1));


        //std::cout << glm::to_string(t1) << " " << glm::to_string(t2) << "\n";
        Ray r(t1, - glm::normalize(t2 - t1), glm::length(t2 - t1));

        return r.checkCollision(t);
    } else {

     }


    return false;
}

bool Triangle::isInside(glm::vec3 point) {
    // Idea based on barycentric coordinates
    float edge1 = getEuclidianDistance(vertices[0], vertices[1]);
    float edge2 = getEuclidianDistance(vertices[1], vertices[2]);
    float edge3 = getEuclidianDistance(vertices[0], vertices[2]);

    float edge4 = getEuclidianDistance(vertices[0], glm::vec3(point.x, point.y, point.z));
    float edge5 = getEuclidianDistance(vertices[1], glm::vec3(point.x, point.y, point.z));
    float edge6 = getEuclidianDistance(vertices[2], glm::vec3(point.x, point.y, point.z));

    float subarea1 = getTriangleArea(edge1, edge4, edge5);
    float subarea2 = getTriangleArea(edge2, edge5, edge6);
    float subarea3 = getTriangleArea(edge3, edge4, edge6);

    return fabs(subarea1 + subarea2 + subarea3 - getTriangleArea(edge1, edge2, edge3)) <= EPS;
}


Capsule::Capsule(glm::vec3 start, glm::vec3 end, float radius) {
    this->start0 = start;
    this->end0 = end;
    this->radius = radius;

    Mesh* Tube = readObj("Tube.obj");
    Mesh* halfSphere = readObj("halfSphere.obj");

    body = new Mesh();

    body->vertices = Tube->vertices;
    body->indices = Tube->indices;

    float diff = glm::length(end - start);

    glm::mat4 scaleMatrixTube = glm::scale(glm::mat4(1), glm::vec3(radius, diff/2, radius));

    for (int l = 0; l < body->vertices.size(); ++l)
        body->vertices[l].Position = getTransformedVertex(scaleMatrixTube, body->vertices[l].Position);

    glm::mat4 scaleMatrixHalfSphere = glm::scale(glm::mat4(1), glm::vec3(radius));

    glm::mat4 modelEnd = glm::translate(glm::mat4(1), glm::vec3(0, diff/2, 0));
    glm::mat4 modelStart = glm::translate(glm::mat4(1), glm::vec3(0, -diff/2, 0)) * glm::rotate(glm::mat4(1), glm::radians(180.0f), glm::vec3(1, 0, 0));

    modelStart = modelStart * scaleMatrixHalfSphere;
    modelEnd = modelEnd * scaleMatrixHalfSphere;




    size_t indicesCount = Tube->indices.size();

    for (int i = 0; i < halfSphere->vertices.size(); ++i) {
        Vertex v{ getTransformedVertex(modelEnd, halfSphere->vertices[i].Position), halfSphere->vertices[i].Normal};
        body->vertices.push_back(v);
    }
    for (int j = 0; j < halfSphere->indices.size(); ++j) {
        body->indices.push_back(halfSphere->indices[j] + indicesCount);
    }

    indicesCount = body->indices.size();

    for (int i = 0; i < halfSphere->vertices.size(); ++i) {
        Vertex v{ getTransformedVertex(modelStart, halfSphere->vertices[i].Position), halfSphere->vertices[i].Normal};
        body->vertices.push_back(v);
    }
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

    glm::vec3 center = glm::vec3(0);
    for (auto v : mesh->vertices) {
        center = center + v.Position;
        if (min.x > v.Position.x) min.x = v.Position.x;
        if (min.y > v.Position.y) min.y = v.Position.y;
        if (min.z > v.Position.z) min.z = v.Position.z;
        if (max.x < v.Position.x) max.x = v.Position.x;
        if (max.y < v.Position.y) max.y = v.Position.y;
        if (max.z < v.Position.z) max.z = v.Position.z;
    }
    center = center/(float)mesh->vertices.size();
    glm::vec3 r = glm::max(glm::abs(min - center), glm::abs(max - center));
    float radius = glm::sqrt(2 * glm::max(r.x, glm::max(r.y, r.z)));
    float max_y = glm::max(glm::abs(min.y) - radius, glm::abs(max.y) - radius);
    return new Capsule(glm::vec3(0, max_y, 0), - glm::vec3(0, max_y, 0), radius);
}
void Capsule::setTransform(glm::vec3 pos, glm::vec3 rot, glm::vec3 scale) {
    Collider::setTransform(pos, rot, scale);
    start = getTransformedVertex(transform, start0);
    end = getTransformedVertex(transform, end0);

    body->translation = pos;
    body->scale = scale;
    body->rotation = rot;
}
bool Capsule::checkCollision(TriangleMesh *col) {
    return col->checkCollision(this);
}
bool Capsule::checkCollision(BoundingSphere *col) {
    glm::vec3 bestPoint = ClosestPointOnLineSegment(start, end, col->pos);
    BoundingSphere s(bestPoint, radius);
    return s.checkCollision(col);
}
bool Capsule::checkCollision(AABB *col) {
    glm::vec3 bestPoint = ClosestPointOnLineSegment(start, end, (col->max + col->min)/2.0f);
    BoundingSphere s(bestPoint, radius);
    return s.checkCollision(col);
}
// TODO https://wickedengine.net/2020/04/26/capsule-collision-detection/
bool Capsule::checkCollision(Triangle *t) {
    return false;
}
bool Capsule::checkCollision(Ray *col) {
// capsule:
    glm::vec3 a_Normal = normalize(this->start - this->end);
    glm::vec3 a_LineEndOffset = a_Normal * this->radius;
    glm::vec3 a_A = this->start + a_LineEndOffset;
    glm::vec3 a_B = this->end - a_LineEndOffset;

// ray:
    glm::vec3 b_A = col->origin;
    glm::vec3 b_B = col->direction * col->length + col->origin;

// vectors between line endpoints
    glm::vec3 v0 = b_A - a_A;
    glm::vec3 v1 = b_B - a_A;
    glm::vec3 v2 = b_A - a_B;
    glm::vec3 v3 = b_B - a_B;

// squared distances:
    float d0 = glm::dot(v0, v0);
    float d1 = glm::dot(v1, v1);
    float d2 = glm::dot(v2, v2);
    float d3 = glm::dot(v3, v3);

// select best potential endpoint on capsule A:
    glm::vec3 bestA;
    if (d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1)
    {
        bestA = a_B;
    }
    else
    {
        bestA = a_A;
    }

// select point on line segment nearest to best potential endpoint on capsule:
    glm::vec3 bestB = ClosestPointOnLineSegment(b_A, b_B, bestA);

// now do the same for capsule A segment:
    bestA = ClosestPointOnLineSegment(this->start, this->end, bestB);

    BoundingSphere s(bestA, this->radius);
    return s.checkCollision(col);
}
// !source of inspiration: https://wickedengine.net/2020/04/26/capsule-collision-detection/
bool Capsule::checkCollision(Capsule *col) {
// capsule A:
    glm::vec3 a_Normal = normalize(this->start - this->end);
    glm::vec3 a_LineEndOffset = a_Normal * this->radius;
    glm::vec3 a_A = this->start + a_LineEndOffset;
    glm::vec3 a_B = this->end - a_LineEndOffset;

// capsule B:
    glm::vec3 b_Normal = normalize(col->start - col->end);
    glm::vec3 b_LineEndOffset = b_Normal * col->radius;
    glm::vec3 b_A = col->start + b_LineEndOffset;
    glm::vec3 b_B = col->end - b_LineEndOffset;

// vectors between line endpoints
    glm::vec3 v0 = b_A - a_A;
    glm::vec3 v1 = b_B - a_A;
    glm::vec3 v2 = b_A - a_B;
    glm::vec3 v3 = b_B - a_B;

// squared distances:
    float d0 = glm::dot(v0, v0);
    float d1 = glm::dot(v1, v1);
    float d2 = glm::dot(v2, v2);
    float d3 = glm::dot(v3, v3);

// select best potential endpoint on capsule A:
    glm::vec3 bestA;
    if (d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1)
    {
        bestA = a_B;
    }
    else
    {
        bestA = a_A;
    }

// select point on capsule B line segment nearest to best potential endpoint on A capsule:
    glm::vec3 bestB = ClosestPointOnLineSegment(col->start, col->end, bestA);

// now do the same for capsule A segment:
    bestA = ClosestPointOnLineSegment(this->start, this->end, bestB);

    BoundingSphere s1(bestA, this->radius);
    BoundingSphere s2(bestB, col->radius);

    return s1.checkCollision(&s2);
}


