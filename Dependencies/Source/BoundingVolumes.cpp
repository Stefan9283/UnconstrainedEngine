//
// Created by Stefan on 22-Feb-21.
//

#include <ObjLoad.h>
#include "BoundingVolumes.h"
#include "Mesh.h"
#include "glm/gtx/string_cast.hpp"

//#include "boost/multiprecision/cpp_bin_float.hpp"
//#include <boost/multiprecision/mpfr.hpp>  // Defines the Backend type that wraps MPFR.
//
//typedef    boost::multiprecision::number<boost::multiprecision::mpfr_float_backend<300>> bigfloat;


extern Shader *s;



#pragma region Functii Ovidiu

#define EPS 0.0000001
#define INF 2000000000

float getEuclidianDistance(glm::vec3 p1, glm::vec3 p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
}

float getTriangleArea(float edge1, float edge2, float edge3) {
    float p = (edge1 + edge2 + edge3) / 2;
    return sqrt(p * (p - edge1) * (p - edge2) * (p - edge3));
}
float getTriangleArea2(glm::dvec3 edge1, glm::dvec3 edge2) {
    glm::vec3 N = glm::cross(edge1, edge2);
    return 0.5f * glm::sqrt(glm::dot(N, N));
}
struct Solution {
    glm::vec3 point;
};

Solution xIsFixed(glm::vec3 A, glm::vec3 B, float x) {
    Solution solution;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = x;
    solution.point.y = (x - x0) * (y1 - y0) / (x1 - x0) + y0;
    solution.point.z = (x - x0) * (z1 - z0) / (x1 - x0) + z0;

    return solution;
}

Solution yIsFixed(glm::vec3 A, glm::vec3 B, float y) {
    Solution solution;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = (y - y0) * (x1 - x0) / (y1 - y0) + x0;
    solution.point.y = y;
    solution.point.z = (y - y0) * (z1 - z0) / (y1 - y0) + z0;

    return solution;
}

Solution zIsFixed(glm::vec3 A, glm::vec3 B, float z) {
    Solution solution;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = (z - z0) * (x1 - x0) / (z1 - z0) + x0;
    solution.point.y = (z - z0) * (y1 - y0) / (z1 - z0) + y0;
    solution.point.z = z;

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
                solution.point.x = solution.point.y = solution.point.z = INF;
            else
                solution.point.x = solution.point.y = solution.point.z = -INF;

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
                solution.point.x = solution.point.y = solution.point.z = INF;
            else
                solution.point.x = solution.point.y = solution.point.z = -INF;

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
                solution.point.x = solution.point.y = solution.point.z = INF;
            else
                solution.point.x = solution.point.y = solution.point.z = -INF;

            return solution;
        }

        solution = zIsFixed(A, B, a / b);
    }

    return solution;
}
#pragma endregion
#pragma region Functii Stefan
glm::dvec3 getTransformedVertex(glm::mat4 tr, glm::dvec3 v) {
    return glm::dvec3(tr * glm::dvec4(v, 1.0f));
}
glm::vec3 ClosestPointOnLineSegment(glm::vec3 A, glm::vec3 B, glm::vec3 Point)
{
    glm::vec3 AB = B - A;
    float t = glm::dot(Point - A, AB) / glm::dot(AB, AB);
    return A + glm::min(glm::max(t, 0.0f), 1.0f) * AB; // saturate(t) can be written as: min((max(t, 0), 1)
}
glm::vec3 getMidPoint(glm::vec3 A, glm::vec3 B, glm::vec3 C) {
    glm::vec3 result = A, min, max;

    min = glm::min(A, glm::min(B, C));
    max = glm::max(A, glm::max(B, C));

    if (A.x != B.x && B.x != C.x) {
        if (A.x != min.x && A.x != max.x) result.x = A.x;
        if (B.x != min.x && B.x != max.x) result.x = B.x;
        if (C.x != min.x && C.x != max.x) result.x = C.x;
    }
    if (A.y != B.y && B.y != C.y) {
        if (A.y != min.y && A.y != max.y) result.y = A.y;
        if (B.y != min.y && B.y != max.y) result.y = B.y;
        if (C.y != min.y && C.y != max.y) result.y = C.y;
    }
    if (A.z != B.z && B.z != C.z) {
        if (A.z != min.z && A.z != max.z) result.z = A.z;
        if (B.z != min.z && B.z != max.z) result.z = B.z;
        if (C.z != min.z && C.z != max.z) result.z = C.z;
    }
    return result;
}
#pragma endregion

#pragma region Collider
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
#pragma endregion

#pragma region BoundingSphere
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
void BoundingSphere::toString() {
    std::cout << "Sphere center: " << glm::to_string(this->pos) << " " << this->radius << "\n";
}
bool BoundingSphere::isInside(glm::vec3 point) {
    return glm::length(point - pos) < radius;
}

#pragma endregion
#pragma region AABB
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
void AABB::toString() {
    std::cout << "AABB:\n\tmin: " << glm::to_string(min) << "\n\tmax: "<< glm::to_string(max) << "\n";
}
bool AABB::isInside(glm::vec3 point) {
    glm::vec3 minValues = min, maxValues = max;

    if (max.x < minValues.x) minValues.x = max.x;
    if (max.y < minValues.y) minValues.y = max.y;
    if (max.z < minValues.z) minValues.z = max.z;
    if (min.x > maxValues.x) maxValues.x = min.x;
    if (min.y > maxValues.y) maxValues.y = min.y;
    if (min.z > maxValues.z) maxValues.z = min.z;

    return glm::clamp(point, minValues, maxValues) == point;
}

#pragma endregion
#pragma region Ray
Ray::Ray(glm::vec3 origin, glm::vec3 direction, float length, bool createMesh = false) {
    this->origin = origin;
    this->direction = direction;
    this->length = length;
    if (createMesh)
    {
        body = new Mesh();
        body->vertices.push_back(Vertex{ origin });
        body->vertices.push_back(Vertex{ origin + direction * length });
        body->indices.push_back(0);
        body->indices.push_back(1);
        body->indices.push_back(0);
        body->solidON = false;
        body->wireframeON = true;
        body->prepare();
    }
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
        return new Ray(origin, glm::normalize(cam->goFront), 100, true);
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
        float distance1 = getEuclidianDistance(A, solution.point);
        float distance2 = getEuclidianDistance(A + this->direction * this->length, solution.point);

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
        float distance1 = getEuclidianDistance(A, solution.point);
        float distance2 = getEuclidianDistance(A + this->direction * this->length, solution.point);

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return true;

        if (x0 != x1)
            solution = xIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
        else if (y0 != y1)
            solution = yIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
        else
            solution = zIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));

        // Check the distance between the ray's origin and the second point
        distance1 = getEuclidianDistance(A, solution.point);
        distance2 = getEuclidianDistance(A + this->direction * this->length, solution.point);

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
bool Ray::checkCollision(Ray* r) {
    glm::vec3 A = this->origin, B = this->origin + this->direction * length;
    glm::vec3 C = r->origin, D = r->origin + r->direction * r->length;

    Solution solution = getIntersectionPoint(A, B, C, D);

    float distance1 = getEuclidianDistance(A, solution.point);
    float distance2 = getEuclidianDistance(B, solution.point);

    float distance3 = getEuclidianDistance(C, solution.point);
    float distance4 = getEuclidianDistance(D, solution.point);

    bool res = solution.point.x != -INF &&
               solution.point.x != INF && fabs(distance1 + distance2 - this->length) <= EPS && (
                       ( C.x >= solution.point.x && solution.point.x >= D.x ) || ( C.x <= solution.point.x && solution.point.x <= D.x) &&
                                                                                 ( C.y <= solution.point.y && solution.point.y <= D.y ) || ( C.y >= solution.point.y && solution.point.y >= D.y) &&
                                                                                                                                           ( C.z <= solution.point.z && solution.point.z <= D.z ) || ( C.z >= solution.point.z && solution.point.z >= D.z)
               );

    return solution.point.x != -INF &&
           solution.point.x != INF &&
           fabs(distance1 + distance2 - this->length) <= EPS &&
           fabs(distance3 + distance4 - r->length) <= EPS;
}
bool Ray::checkCollision(Triangle *t) {
    glm::vec3 A = this->origin, B = this->origin + this->direction;

    if (!t->twoway && glm::dot(t->norm, B) < 0)
        return false;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    float a = t->norm.x, b = t->norm.y, c = t->norm.z;
    float d = glm::dot(t->norm, t->vertices[0]);

    float c1, c2, c3, c4;
    Solution solution;

    Ray r0(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[1]), glm::length(t->vertices[0] - t->vertices[1]));
    Ray r1(t->vertices[1], glm::normalize(- t->vertices[1] + t->vertices[2]), glm::length(t->vertices[1] - t->vertices[2]));
    Ray r2(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[2]), glm::length(t->vertices[0] - t->vertices[2]));

    // Pick an axis which the line is not parallel to
    if (x0 != x1) {
        c1 = b * (y1 - y0)/(x1 - x0);
        c2 = b * (x1 * y0 - x0 * y1)/(x1 - x0);

        c3 = c * (z1 - z0)/(x1 - x0);
        c4 = c * (x1 * z0 - x0 * z1)/(x1 - x0);

        if (!(a + (c1 + c3))) {
            // No intersection at all
            if (d - (c2 - c4) )
                return false;

            //std::cout << "hello\n";

            // The intersection is an infinite number on points
            if (t->isInside(A) || t->isInside(A + this->direction * this->length))
                return true;

            return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
        }

        solution = xIsFixed(A, B, (d - c2 - c4) / (a + c1 + c3));
    } else if (y0 != y1) {

        c1 = a * (x1 - x0) / (y1 - y0);
        c2 = a * (y1 * x0 - y0 * x1) / (y1 - y0);

        c3 = c * (z1 - z0) / (y1 - y0);
        c4 = c * (y1 * z0 - y0 * z1) / (y1 - y0);

        if (!(b + c1 + c3)) {
            // No intersection at all
            if (d - c2 - c4)
                return false;

            // The intersection is an infinite number on points
            if (t->isInside(A) || t->isInside(A + this->direction * this->length))
                return true;

            return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
        }

        solution = yIsFixed(A, B, (d - c2 - c4) / (b + c1 + c3));
    } else {

        c1 = a * (x1 - x0) / (z1 - z0);
        c2 = a * (z1 * x0 - z0 * x1) / (z1 - z0);

        c3 = c * (y1 - y0) / (z1 - z0);
        c4 = c * (z1 * y0 - z0 * y1) / (z1 - z0);

        if (!(c + c1 + c3)) {
            // No intersection at all
            if (d - c2 - c4)
                return false;

            // The intersection is an infinite number on points
            if (t->isInside(A) || t->isInside(A + this->direction * this->length))
                return true;

            return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
        }

        solution = zIsFixed(A, B, (d - c2 - c4) / (c + c1 + c3));
    }

    float distance1 = getEuclidianDistance(A, solution.point);
    float distance2 = getEuclidianDistance(A + this->direction * this->length, solution.point);

    if (distance1 > this->length || distance2 > this->length)
        return false;

    ////////////////////////
    const float EPSILON = 0.000000001;
    glm::vec3 vertex0 = t->vertices[0];
    glm::vec3 vertex1 = t->vertices[1];
    glm::vec3 vertex2 = t->vertices[2];
    glm::vec3 edge1, edge2, h, s, q, rayVector = origin + length * direction, P;
    float aa ,f,u,v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = glm::cross(rayVector, edge2);
    aa = glm::dot(edge1, h);
    f = 1.0/aa;
    s = origin - vertex0;
    u = f * glm::dot(s, h);
    if (u < 0.0 || u > 1.0)
        return false;
    q = glm::cross(s, edge1);
    v = f * glm::dot(rayVector, q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float tt = f * glm::dot(edge2, q);
    //std::cout << t << " t\n";
    //if (tt > EPSILON) // ray intersection
    {
        P = origin + rayVector * tt;
        //std::cout << glm::to_string(solution.point) << glm::to_string(P) << glm::length(solution.point - P) << "\n";
        return true;
    }
    ///////////////////////


    return t->isInside(solution.point);
}
bool Ray::checkCollision(Capsule *col) {
    return col->checkCollision(this);
}
void Ray::toString() {
    std::cout << "Ray:\n\torigin: "
              << glm::to_string(this->origin) << "\n\tdirection: "
              << glm::to_string(this->direction) << "\n\tlength: "
              << this->length << "\n\tend point: "
              << glm::to_string(origin + direction * length) << "\n";
}
#pragma endregion
#pragma region TriangleMesh

std::vector<Mesh*> wasMeshHit(Collider* mesh, Collider *col) {
    Mesh* hit = new Mesh(), *not_hit = new Mesh();
    std::vector<Mesh*> meshes;
    meshes.push_back(hit);
    meshes.push_back(not_hit);
    bool result = false;

    int total = 0;

    for (int i = 0; i < mesh->body->indices.size(); i+=3) {
        Triangle t(
                getTransformedVertex(mesh->transform, mesh->body->vertices[i].Position),
                getTransformedVertex(mesh->transform, mesh->body->vertices[i+1].Position),
                getTransformedVertex(mesh->transform, mesh->body->vertices[i+2].Position),
                glm::mat3(transpose(inverse(mesh->transform))) * mesh->body->vertices[i].Normal
        );
        result = col->checkCollision(&t);
        if (result) {
            hit->vertices.push_back(mesh->body->vertices[i]);
            hit->vertices.push_back(mesh->body->vertices[i+1]);
            hit->vertices.push_back(mesh->body->vertices[i+2]);
            hit->indices.push_back(hit->indices.size());
            hit->indices.push_back(hit->indices.size());
            hit->indices.push_back(hit->indices.size());
            total++;
        } else {

            not_hit->vertices.push_back(mesh->body->vertices[i]);
            not_hit->vertices.push_back(mesh->body->vertices[i+1]);
            not_hit->vertices.push_back(mesh->body->vertices[i+2]);
            not_hit->indices.push_back(hit->indices.size());
            not_hit->indices.push_back(hit->indices.size());
            not_hit->indices.push_back(hit->indices.size());
        }
    }

    std::cout << hit->vertices.size() << "  " << not_hit->vertices.size() << "\n";

    hit->prepare();
    not_hit->prepare();

    return meshes;
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

    Mesh* hit = new Mesh(), *not_hit = new Mesh();

    bool result = false;
    int total = 0;
    for (int i = 0; i < this->body->indices.size(); i+=3) {
        Triangle t(
                getTransformedVertex(transform, body->vertices[i].Position),
                getTransformedVertex(transform, body->vertices[i+1].Position),
                getTransformedVertex(transform, body->vertices[i+2].Position),
                glm::mat3(transpose(inverse(transform))) * body->vertices[i].Normal
                );
        std::cout << i << " ";
        result = col->checkCollision(&t);
        if (result) {
            hit->vertices.push_back(body->vertices[i]);
            hit->vertices.push_back(body->vertices[i+1]);
            hit->vertices.push_back(body->vertices[i+2]);
            total++;
        } else {
            not_hit->vertices.push_back(body->vertices[i]);
            not_hit->vertices.push_back(body->vertices[i+1]);
            not_hit->vertices.push_back(body->vertices[i+2]);
        }
    }
    //std::cout << body->indices.size()/3 << "\n";
    //std::cout << total << "\n";
    return total > 0;
}
bool TriangleMesh::checkCollision(Capsule *col) {
    return checkCollisionByTriangle(col);
}
void TriangleMesh::toString() {
    std::cout << "TriangleMesh:\n\tvertices count:" << body->vertices.size() << "\n\ttriangle count" << body->indices.size() << "\n";
}
#pragma endregion
#pragma region Triangle
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
// https://gdbooks.gitbooks.io/3dcollisions/content/Chapter4/aabb-triangle.html
bool Triangle::checkCollision(AABB *bv) {
    for (int i = 0; i < 3; ++i)
        if (isInside(vertices[i])) return true;
    Ray r0(vertices[0], glm::normalize(-vertices[0] + vertices[1]), glm::length(vertices[0] - vertices[1]));
    Ray r1(vertices[1], glm::normalize(-vertices[1] + vertices[2]), glm::length(vertices[1] - vertices[2]));
    Ray r2(vertices[0], glm::normalize(-vertices[0] + vertices[2]), glm::length(vertices[0] - vertices[2]));
    return r0.checkCollision(bv) || r1.checkCollision(bv) || r2.checkCollision(bv);
}
// https://wickedengine.net/2020/04/26/capsule-collision-detection/
bool Triangle::checkCollision(BoundingSphere *bv) {
    for (int i = 0; i < 3; ++i)
        if (isInside(vertices[i])) return true;
    Ray r0(vertices[0], glm::normalize(-vertices[0] + vertices[1]), glm::length(vertices[0] - vertices[1]));
    Ray r1(vertices[1], glm::normalize(-vertices[1] + vertices[2]), glm::length(vertices[1] - vertices[2]));
    Ray r2(vertices[0], glm::normalize(-vertices[0] + vertices[2]), glm::length(vertices[0] - vertices[2]));
    return r0.checkCollision(bv) || r1.checkCollision(bv) || r2.checkCollision(bv);
}
// http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
bool Triangle::checkCollision(Triangle *t) {
    bool twoway0 = twoway;

    twoway = true;

    Ray r0(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[1]), glm::length(t->vertices[0] - t->vertices[1]));
    Ray r1(t->vertices[1], glm::normalize(- t->vertices[1] + t->vertices[2]), glm::length(t->vertices[1] - t->vertices[2]));
    Ray r2(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[2]), glm::length(t->vertices[0] - t->vertices[2]));

    bool result = r0.checkCollision(this) || r1.checkCollision(this) || r2.checkCollision(this);

    twoway = twoway0;

    return result;
}
bool Triangle::isInside(glm::vec3 point) {
    float area = getTriangleArea2(vertices[0] - vertices[2], vertices[0] - vertices[1]);

    //if (area < 0.0001)
    //    std::cout << area << "\n";


    if (area < 0.0001) {
        std::cout << "small tri\n";
        glm::dmat4 scalingMat = glm::scale(glm::mat4(1), glm::vec3(1000));
        glm::vec3 v0, v1, v2, pt;
        v0 = getTransformedVertex(scalingMat, vertices[0]);
        v1 = getTransformedVertex(scalingMat, vertices[1]);
        v2 = getTransformedVertex(scalingMat, vertices[2]);
        pt = getTransformedVertex(scalingMat, point);

        float subarea1 = getTriangleArea2(v0 - v1, v1 - pt);
        float subarea2 = getTriangleArea2(v1 - v2, v2 - pt);
        float subarea3 = getTriangleArea2(v0 - v2, v2 - pt);

        area = getTriangleArea2(v0 - v2, v0 - v1);
        //if(fabs(subarea1 + subarea2 + subarea3 - area)/1000 < 0)
        //    std::cout << "\t" << fabs(subarea1 + subarea2 + subarea3 - area)/1000 << "\n";

        return fabs(subarea1 + subarea2 + subarea3 - area) <= EPS;
    } else {
        float subarea1 = getTriangleArea2(vertices[0] - vertices[1], vertices[1] - point);
        float subarea2 = getTriangleArea2(vertices[1] - vertices[2], vertices[2] - point);
        float subarea3 = getTriangleArea2(vertices[0] - vertices[2], vertices[2] - point);

        return fabs(subarea1 + subarea2 + subarea3 - getTriangleArea2(vertices[0] - vertices[2], vertices[0] - vertices[1]))/1000 <= EPS;
    }
}
void Triangle::toString() {
    std::cout << "Triangle:\n";
    for (int i = 0; i < 3; ++i) {
        std::cout << "\tv" << i  << " " << glm::to_string(vertices[i]) << "\n";
    }
    std::cout << "\tn " << glm::to_string(norm) << "\n";
}
#pragma endregion
#pragma region Capsule
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
// https://wickedengine.net/2020/04/26/capsule-collision-detection/
bool Capsule::checkCollision(Triangle *t) {
    // for triangle inside a capsule
    for (int i = 0; i < 3; ++i) {
        glm::vec3 mid = getMidPoint(start, end, t->vertices[i]);
         if(glm::length(mid - t->vertices[i]) < radius)
             return true;
    }

    bool twoway = t->twoway;
    t->twoway = true;
    Ray r0(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[1]), glm::length(t->vertices[0] - t->vertices[1]));
    Ray r1(t->vertices[1], glm::normalize(- t->vertices[1] + t->vertices[2]), glm::length(t->vertices[1] - t->vertices[2]));
    Ray r2(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[2]), glm::length(t->vertices[0] - t->vertices[2]));
    bool result = r0.checkCollision(this) || r1.checkCollision(this) || r2.checkCollision(this);
    t->twoway = twoway;
    return result;
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
void Capsule::toString() {
    std::cout << "Capsule:\nStart Point: " << glm::to_string(start) << "\n\tEnd Point: "<< glm::to_string(end) << "\n\tRadius: " << radius << "\n";
}
#pragma endregion

