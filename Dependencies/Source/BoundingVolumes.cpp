//
// Created by Stefan on 22-Feb-21.
//

#include <ObjLoad.h>
#include <iomanip>
#include "BoundingVolumes.h"
#include "Mesh.h"
#include "glm/gtx/string_cast.hpp"
#include <iostream>
//#include "boost/multiprecision/cpp_bin_float.hpp"
//#include <boost/multiprecision/mpfr.hpp>  // Defines the Backend type that wraps MPFR.
//
//typedef    boost::multiprecision::number<boost::multiprecision::mpfr_float_backend<300>> bigfloat;


extern Shader *s;

#pragma region Functii Ovidiu

#define EPS 0.00001
#define WEAK_EPS 0.0001
#define INF 2000000000

float getEuclidianDistance(glm::vec3 p1, glm::vec3 p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
}
long double getEuclidianDistance2(glm::dvec3 p1, glm::dvec3 p2) {
    return glm::length(p1 - p2);
}

float getTriangleArea(float edge1, float edge2, float edge3) {
    float p = (edge1 + edge2 + edge3) / 2;
    return sqrt(p * (p - edge1) * (p - edge2) * (p - edge3));
}
long double getTriangleArea2(glm::dvec3 edge1, glm::dvec3 edge2) {
    glm::highp_dvec3 N = glm::cross(edge1, edge2);
    return 0.5f * glm::sqrt(glm::dot(N, N));
}

struct Solution {
    glm::dvec3 point;
};

Solution xIsFixed(glm::dvec3 A, glm::dvec3 B, long double x) {
    Solution solution;

    long double x0 = A.x, y0 = A.y, z0 = A.z;
    long double x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = x;
    solution.point.y = (x - x0) * (y1 - y0) / (x1 - x0) + y0;
    solution.point.z = (x - x0) * (z1 - z0) / (x1 - x0) + z0;

    return solution;
}
Solution yIsFixed(glm::dvec3 A, glm::dvec3 B, long double y) {
    Solution solution;

    long double x0 = A.x, y0 = A.y, z0 = A.z;
    long double x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = (y - y0) * (x1 - x0) / (y1 - y0) + x0;
    solution.point.y = y;
    solution.point.z = (y - y0) * (z1 - z0) / (y1 - y0) + z0;

    return solution;
}
Solution zIsFixed(glm::dvec3 A, glm::dvec3 B, long double z) {
    Solution solution;

    long double x0 = A.x, y0 = A.y, z0 = A.z;
    long double x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = (z - z0) * (x1 - x0) / (z1 - z0) + x0;
    solution.point.y = (z - z0) * (y1 - y0) / (z1 - z0) + y0;
    solution.point.z = z;

    return solution;
}

Solution getIntersectionPoint(glm::dvec3 A, glm::dvec3 B, glm::dvec3 C, glm::dvec3 D) {
    Solution solution;

    long double c1, c2;

    // Pick an axis which both lines are not parallel to
    if (A.x != B.x && C.x != D.x) {
        c1 = (B.y - A.y) / (B.x - A.x);
        c2 = (A.y * B.x - A.x * B.y) / (B.x - A.x);

        long double a = (C.x * D.y + c2 * D.x - c2 * C.x - C.y * D.x);
        long double b = (D.y - C.y - c1 * D.x + c1 * C.x);

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

        long double a = (C.y * D.x + c2 * D.y - c2 * C.y - C.x * D.y);
        long double b = (D.x - C.x - c1 * D.y + c1 * C.y);

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

        long double a = (C.z * D.x + c2 * D.z - c2 * C.z - C.x * D.z);
        long double b = (D.x - C.x - c1 * D.z + c1 * C.z);

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
    double t = glm::dot(Point - A, AB) / glm::dot(AB, AB);
    return A + (float)glm::min(glm::max(t, 0.0), 1.0) * AB; // saturate(t) can be written as: min((max(t, 0), 1)
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
CollisionPoint reverseCollisionPoint(CollisionPoint p) {
    glm::vec3 B = p.B;
    p.B = p.A;
    p.A = B;
    return p;
}
ColliderMesh* convertMesh2ColliderMesh(Mesh* m) {
    ColliderMesh* mesh = new ColliderMesh;
    mesh->indices = m->indices;
    mesh->vertices = m->vertices;
    mesh->prepare();
    delete m;
    return mesh;
}
#pragma endregion

#pragma region Collider
CollisionPoint Collider::checkCollision(Collider* col) {
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

    assert("Collider type was not identified");
}
Collider::~Collider() {
    if (body)
        delete body;
}
void Collider::Draw(Shader *shader) {
    if (body) {
        if (parent)
            body->Draw(parent->getTransform(), shader);
        else
            body->Draw(glm::mat4(1), shader);
    }
}

#pragma endregion
#pragma region ColliderMesh
ColliderMesh::ColliderMesh() {
    localTransform = transform{
            glm::vec3(0),
            glm::vec3(1),
            glm::quat()};
}
ColliderMesh::~ColliderMesh() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}
void ColliderMesh::prepare() {
    //assert(vertices.size() != 0);

    if (vertices.size() == 0) return;

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

    glBindVertexArray(0);
}
void ColliderMesh::Draw(glm::mat4 parentMatrix, Shader *shader) {
    glm::mat4 model = getTransform();
    model = parentMatrix * model;
    shader->setMat4("model", &model);
    shader->bind();
    glBindVertexArray(VAO);
    if (wireframeON) {
        glDisable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // draw mesh wireframe
        glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, nullptr);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    if (solidON) {
        glDrawElements(GL_TRIANGLES, (GLsizei) indices.size(), GL_UNSIGNED_INT, nullptr);
    }
    glBindVertexArray(0);
    shader->unbind();
}
glm::mat4 ColliderMesh::getTransform() {
    glm::mat4 T, R = glm::mat4(1), S;
    T = glm::translate(glm::mat4(1), localTransform.tr);
    R = glm::toMat4(localTransform.rot);
    S = glm::scale(glm::mat4(1), localTransform.sc);
    return T * R * S;
}
void ColliderMesh::gui(int outIndex = 0) {

    std::string name;
    name = name + " " + std::to_string(outIndex);
    if (ImGui::TreeNode(name.c_str())) {
        name = "wireframeON " + std::to_string(outIndex);
        ImGui::Checkbox(name.c_str(), &wireframeON);
        name = "solidON " + std::to_string(outIndex);
        ImGui::Checkbox(name.c_str(), &solidON);

        float t[4] = {localTransform.tr.x, localTransform.tr.y, localTransform.tr.z, 1.0f};
        name = "position " + std::to_string(outIndex);
        ImGui::SliderFloat3(name.c_str(), t, -10, 10);
        localTransform.tr = glm::vec3(t[0], t[1], t[2]);

        ImGui::TreePop();
    }
}
#pragma endregion
#pragma region CollisionPoint
CollisionPoint::CollisionPoint(glm::vec3 A, glm::vec3 B) {
    if (A == B) {
        this->hasCollision = false;
        this->normal = glm::vec3 (0);
        this->B = glm::vec3 (0);
        this->A = glm::vec3 (0);
        this->depth = 0;
    } else {
        this->B = B;
        this->A = A;
        glm::vec3 AB = B - A;
        this->depth = glm::length(AB);
        this->normal = glm::normalize(AB);
        this->hasCollision = true;
    }
}
CollisionPoint::CollisionPoint() {
    this->hasCollision = false;
    this->normal = glm::vec3 (0);
    this->B = glm::vec3 (0);
    this->A = glm::vec3 (0);
    this->depth = 0;
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
    body = convertMesh2ColliderMesh(readObj("Sphere.obj"));
    for (int i=0; i<body->vertices.size(); i++)
        body->vertices[i].Position *= radius;
    body->prepare();
}
CollisionPoint BoundingSphere::checkCollision(TriangleMesh *col) {
    return reverseCollisionPoint(col->checkCollisionByTriangle(this));
}
CollisionPoint BoundingSphere::checkCollision(AABB* bv) {
    glm::vec3 newMin = bv->min + bv->offset, newMax = bv->max + bv->offset;

    float x = std::max(newMin.x, std::min(this->pos.x, newMax.x));
    float y = std::max(newMin.y, std::min(this->pos.y, newMax.y));
    float z = std::max(newMin.z, std::min(this->pos.z, newMax.z));

    if((x - this->pos.x) * (x - this->pos.x) + (y - this->pos.y) * (y - this->pos.y) + (z - this->pos.z) * (z - this->pos.z) <= this->radius * this->radius)
        return {glm::vec3(x, y, z), pos - glm::normalize(pos - glm::vec3(x, y, z)) * radius};
    else return {};
}
CollisionPoint BoundingSphere::checkCollision(BoundingSphere* bv) {
    glm::vec3 center2centerVec = pos - bv->pos;
    if(glm::length(center2centerVec) <= radius + bv->radius) {
        center2centerVec = glm::normalize(center2centerVec);
        return CollisionPoint(pos - center2centerVec * radius, bv->pos + center2centerVec * bv->radius);
    }
    return {};
}
CollisionPoint BoundingSphere::checkCollision(Ray *r) {
    return reverseCollisionPoint(r->checkCollision(this));
}
CollisionPoint BoundingSphere::checkCollision(Triangle *t) {
    return reverseCollisionPoint(t->checkCollision(this));
}
CollisionPoint BoundingSphere::checkCollision(Capsule *col) {
    return reverseCollisionPoint(col->checkCollision(this));
} // TODO return CollisionPoint
void BoundingSphere::setTransform(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
    this->pos += pos;
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
std::vector<Vertex> AABB::generateVerices(glm::vec3 min_, glm::vec3 max_) {
    std::vector<Vertex> vertices;
    vertices.push_back(Vertex{glm::vec3(min_.x, min_.y, min_.z),  glm::vec3(min_.x, min_.y, min_.z)});
    vertices.push_back(Vertex{glm::vec3(min_.x, min_.y, max_.z),  glm::vec3(min_.x, min_.y, max_.z)});
    vertices.push_back(Vertex{glm::vec3(min_.x, max_.y, min_.z),  glm::vec3(min_.x, max_.y, min_.z)});
    vertices.push_back(Vertex{glm::vec3(min_.x, max_.y, max_.z),  glm::vec3(min_.x, max_.y, max_.z)});
    vertices.push_back(Vertex{glm::vec3(max_.x, min_.y, min_.z),  glm::vec3(max_.x, min_.y, min_.z)});
    vertices.push_back(Vertex{glm::vec3(max_.x, min_.y, max_.z),  glm::vec3(max_.x, min_.y, max_.z)});
    vertices.push_back(Vertex{glm::vec3(max_.x, max_.y, min_.z),  glm::vec3(max_.x, max_.y, min_.z)});
    vertices.push_back(Vertex{glm::vec3(max_.x, max_.y, max_.z),  glm::vec3(max_.x, max_.y, max_.z)});
    return vertices;
}
ColliderMesh *AABB::generateNewMesh() {
    std::vector<Vertex> verticesPos = generateVerices(min, max);

    body = new ColliderMesh();

    for (auto v : verticesPos) {
        body->vertices.push_back(v);
    }

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
void AABB::setTransform(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
    Collider::setTransform((min + max)/2.0f + pos, rot, scale);
    if (body) delete body;
    updateMinMax(localTransform);
    body = generateNewMesh();
}
void AABB::updateMinMax(glm::mat4 transform) {
    std::vector<Vertex> verticesPos = generateVerices(min0, max0);
    for (int i = 0; i < verticesPos.size(); ++i)
        verticesPos[i].Position = getTransformedVertex(transform, verticesPos[i].Position);

    min = verticesPos[0].Position;
    max = verticesPos[0].Position;

    for (int i = 1; i < verticesPos.size(); ++i) {
        min = glm::min(min, verticesPos[i].Position);
        max = glm::max(max, verticesPos[i].Position);
    }
}
CollisionPoint AABB::checkCollision(TriangleMesh *col) {
    return reverseCollisionPoint(col->checkCollisionByTriangle(this));
}
CollisionPoint AABB::checkCollision(AABB* bv) {
    glm::vec3 newMin1 = bv->min + bv->offset, newMax1 = bv->max + bv->offset;
    glm::vec3 newMin2 = this->min + this->offset, newMax2 = this->max + this->offset;

    if (newMin1.x > newMax2.x || newMax1.x < newMin2.x)
        return {};

    if (newMin1.y > newMax2.y || newMax1.y < newMin2.y)
        return {};

    if (newMin1.z > newMax2.z || newMax1.z < newMin2.z)
        return {};

    glm::vec3 v0 = glm::vec3(   std::max(newMin1.x, std::min(newMin2.x, newMax1.x)),
                                std::max(newMin1.y, std::min(newMin2.y, newMax1.y)),
                                std::max(newMin1.z, std::min(newMin2.z, newMax1.z)));

    glm::vec3 v1 = glm::vec3(   std::max(newMin1.x, std::min(newMax2.x, newMax1.x)),
                                std::max(newMin1.y, std::min(newMax2.y, newMax1.y)),
                                std::max(newMin1.z, std::min(newMax2.z, newMax1.z)));

    glm::vec3 mean = (v0 + v1);///2.0f;

    return {bv->closestPoint(- mean), closestPoint(mean)};
}
CollisionPoint AABB::checkCollision(BoundingSphere* bv) {
    return reverseCollisionPoint(bv->checkCollision(this));
}
CollisionPoint AABB::checkCollision(Ray *r) {
    return reverseCollisionPoint(r->checkCollision(this));
}
CollisionPoint AABB::checkCollision(Triangle *t) {
    return reverseCollisionPoint(t->checkCollision(this));
}
CollisionPoint AABB::checkCollision(Capsule *col) {
    return reverseCollisionPoint(col->checkCollision(this));
}
AABB::AABB(glm::vec3 min, glm::vec3 max) {
    min0 = min;
    max0 = max;
    this->min = min0;
    this->max = max0;
    offset = glm::vec3(0);
    localTransform = glm::mat4(1);
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

glm::vec3 AABB::closestPoint(glm::vec3 point) {
    glm::vec3 result;

    glm::vec3 newMin = min + offset, newMax = max + offset;

    if(point.x > newMax.x)
        result.x = newMax.x;
    else if(point.x < newMin.x)
        result.x = newMin.x;
    else
        result.x = point.x;

    if(point.y > newMax.y)
        result.y = newMax.y;
    else if(point.y < newMin.y)
        result.y = newMin.y;
    else
        result.y = point.y;

    if(point.z > newMax.z)
        result.z = newMax.z;
    else if(point.z < newMin.z)
        result.z = newMin.z;
    else
        result.z = point.z;

    return result;
}

#pragma endregion
#pragma region Ray
Ray::Ray(glm::vec3 origin, glm::vec3 direction, float length, bool createMesh = false) {
    this->origin = origin;
    this->direction = direction;
    this->length = length;
    parent = nullptr;
    if (true)
    {
        body = new ColliderMesh;
        body->vertices.push_back(Vertex{ origin });
        body->vertices.push_back(Vertex{ origin + direction * length });
        body->indices.push_back(0);
        body->indices.push_back(1);
        body->indices.push_back(0);
        body->solidON = true;
        body->wireframeON = true;
        body->prepare();
    }
}
Ray* Ray::generateRay(GLFWwindow* window, Camera* cam) {
    static bool wasHeld = false;
    if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE && wasHeld) {
        int wi, he;
        glfwGetWindowSize(window, &wi, &he);
        double x = cam->lastx_mouse - (float) wi / 2.0f;
        double y = he - cam->lasty_mouse - (float) he / 2.0f;

        x /= (double) wi / 2.0f;
        y /= (double) he / 2.0f;

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
CollisionPoint Ray::checkCollision(TriangleMesh *col) {
    return reverseCollisionPoint(col->checkCollisionByTriangle(this));
}
CollisionPoint Ray::checkCollision(BoundingSphere* bv) {
    glm::vec3 A = this->origin, B = this->origin + this->direction * this->length;

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

    if (delta < 0) {
        return {};
    }
    if (delta == 0) {
        Solution solution;

        if (A.x != B.x)
            solution = xIsFixed(A, B, -b / (2 * a));
        else if (A.y != B.y)
            solution = yIsFixed(A, B, -b / (2 * a));
        else
            solution = zIsFixed(A, B, -b / (2 * a));

        // Check the distance between the ray's origin and the unique point
        float distance1 = getEuclidianDistance2(A, solution.point);
        float distance2 = getEuclidianDistance2(A + this->direction * this->length, solution.point);

        if (fabs(distance1 + distance2 - this->length) <= EPS) {
            std::cout << "brah0\n";
            return CollisionPoint(solution.point, solution.point); // return true;
        }
    } else {
        Solution solution1;

        if (x0 != x1)
            solution1 = xIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));
        else if (y0 != y1)
            solution1 = yIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));
        else
            solution1 = zIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));

        // Check the distance between the ray's origin and the first point
        long double distance11 = getEuclidianDistance2(A, solution1.point);
        long double distance12 = getEuclidianDistance2(A + this->direction * this->length, solution1.point);

        /*
        if (fabs(distance11 + distance12 - this->length) <= EPS) {
            return CollisionPoint(solution1.point, solution1.point);; //return true;
        }
        */

        Solution solution2;

        if (x0 != x1)
            solution2 = xIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
        else if (y0 != y1)
            solution2 = yIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
        else
            solution2 = zIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));

        long double distance21, distance22;
        // Check the distance between the ray's origin and the second point
        distance21 = getEuclidianDistance2(A, solution2.point);
        distance22 = getEuclidianDistance2(A + this->direction * this->length, solution2.point);

        /*
        if (fabs(distance21 + distance22 - this->length) <= EPS) {
            std::cout << "brah1\n";
            return CollisionPoint(solution2.point, solution2.point);; // return true;
        }
        */


        if ((fabs(distance11 + distance12 - this->length) > WEAK_EPS) && (fabs(distance21 + distance22 - this->length) > WEAK_EPS))
        {
            //std::cout << distance11 << " " << distance12 << " " << distance21 << " " << distance22 << "\n";
            //std::cout << distance11 + distance12 - (long double)this->length << " " << distance21 + distance22 - (long double)this->length << "\n";
            //std::cout << glm::to_string(solution1.point) << glm::to_string(solution2.point) << "2\n";
            //std::cout << glm::to_string(origin) << glm::to_string(direction) << "2\n";
            return {};
        }

        glm::vec3 start, end;

        if (bv->isInside(A))
            start = A;
        else if (distance11 < distance21)
            start = solution1.point;
        else start = solution2.point;

        if (bv->isInside(B))
            end = B;
        else if (distance12 < distance22)
            end = solution1.point;
        else end = solution2.point;

        return CollisionPoint(start, end);



    }

    return {};

}
CollisionPoint Ray::checkCollision(AABB* bv) {
    glm::vec3 newMin = bv->min + bv->offset, newMax = bv->max + bv->offset;
    glm::vec3 A = this->origin, B = this->origin - this->direction;

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    float x, y, z;

    // Front face
    x = (newMax.z * x1 - newMax.z * x0 - x1 * z0 + x0 * z1) / (z1 - z0);
    y = (newMax.z * y1 - newMax.z * y0 - y1 * z0 + y0 * z1) / (z1 - z0);

    if (z0 != z1 && x >= newMin.x && x <= newMax.x && y >= newMin.y && y <= newMax.y) {
        float distance1 = getEuclidianDistance2(A, glm::vec3(x, y, newMax.z));
        float distance2 = getEuclidianDistance2(A + this->direction * this->length, glm::vec3(x, y, newMax.z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return {}; // return true;
    }

    // Back face
    x = (newMin.z * x1 - newMin.z * x0 - x1 * z0 + x0 * z1) / (z1 - z0);
    y = (newMin.z * y1 - newMin.z * y0 - y1 * z0 + y0 * z1) / (z1 - z0);

    if (z0 != z1 && x >= newMin.x && x <= newMax.x && y >= newMin.y && y <= newMax.y) {
        float distance1 = getEuclidianDistance2(A, glm::vec3(x, y, newMin.z));
        float distance2 = getEuclidianDistance2(A + this->direction * this->length, glm::vec3(x, y, newMin.z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return {}; // return true;
    }

    // Left face
    y = (newMin.x * y1 - newMin.x * y0 - y1 * x0 + y0 * x1) / (x1 - x0);
    z = (newMin.x * z1 - newMin.x * z0 - z1 * x0 + z0 * x1) / (x1 - x0);

    if (x0 != x1 && y >= newMin.y && y <= newMax.y && z >= newMin.z && z <= newMax.z) {
        float distance1 = getEuclidianDistance2(A, glm::vec3(newMin.x, y, z));
        float distance2 = getEuclidianDistance2(A + this->direction * this->length, glm::vec3(newMin.x, y, z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return {}; // return true;
    }

    // Right face
    y = (newMax.x * y1 - newMax.x * y0 - y1 * x0 + y0 * x1) / (x1 - x0);
    z = (newMax.x * z1 - newMax.x * z0 - z1 * x0 + z0 * x1) / (x1 - x0);

    if (x0 != x1 && y >= newMin.y && y <= newMax.y && z >= newMin.z && z <= newMax.z) {
        float distance1 = getEuclidianDistance2(A, glm::vec3(newMax.x, y, z));
        float distance2 = getEuclidianDistance2(A + this->direction * this->length, glm::vec3(newMax.x, y, z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return {}; // return true;
    }

    // Bottom face
    x = (newMin.y * x1 - newMin.y * x0 - x1 * y0 + x0 * y1) / (y1 - y0);
    z = (newMin.y * z1 - newMin.y * z0 - z1 * y0 + z0 * y1) / (y1 - y0);

    if (y0 != y1 && x >= newMin.x && x <= newMax.x && z >= newMin.z && z <= newMax.z) {
        float distance1 = getEuclidianDistance2(A, glm::vec3(x, newMin.y, z));
        float distance2 = getEuclidianDistance2(A + this->direction * this->length, glm::vec3(x, newMin.y, z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return {}; // return true;
    }

    // Up face
    x = (newMax.y * x1 - newMax.y * x0 - x1 * y0 + x0 * y1) / (y1 - y0);
    z = (newMax.y * z1 - newMax.y * z0 - z1 * y0 + z0 * y1) / (y1 - y0);

    if (y0 != y1 && x >= newMin.x && x <= newMax.x && z >= newMin.z && z <= newMax.z) {
        float distance1 = getEuclidianDistance2(A, glm::vec3(x, newMax.y, z));
        float distance2 = getEuclidianDistance2(A + this->direction * this->length, glm::vec3(x, newMax.y, z));

        if (fabs(distance1 + distance2 - this->length) <= EPS)
            return {}; //  true;
    }

    return {};
} // TODO return CollisionPoint
CollisionPoint Ray::checkCollision(Ray* r) {
    glm::vec3 A = this->origin, B = this->origin + this->direction * length;
    glm::vec3 C = r->origin, D = r->origin + r->direction * r->length;

    Solution solution = getIntersectionPoint(A, B, C, D);

    long double distance1 = getEuclidianDistance2(A, solution.point);
    long double distance2 = getEuclidianDistance2(B, solution.point);

    long double distance3 = getEuclidianDistance2(C, solution.point);
    long double distance4 = getEuclidianDistance2(D, solution.point);

    bool res = solution.point.x != -INF &&
               solution.point.x != INF && fabs(distance1 + distance2 - this->length) <= EPS && (
                       ( C.x >= solution.point.x && solution.point.x >= D.x ) || ( C.x <= solution.point.x && solution.point.x <= D.x) &&
                                                                                 ( C.y <= solution.point.y && solution.point.y <= D.y ) || ( C.y >= solution.point.y && solution.point.y >= D.y) &&
                                                                                                                                           ( C.z <= solution.point.z && solution.point.z <= D.z ) || ( C.z >= solution.point.z && solution.point.z >= D.z)
               );

    return {};
    /*
    return solution.point.x != -INF &&
           solution.point.x != INF &&
           fabs(distance1 + distance2 - this->length) <= EPS &&
           fabs(distance3 + distance4 - r->length) <= EPS;
           */
} // TODO return CollisionPoint
CollisionPoint Ray::checkCollision(Triangle *t) {
    glm::dvec3 A = this->origin, B = this->origin + this->length * this->direction;

    if (!t->twoway && glm::dot((glm::dvec3)t->norm, (glm::dvec3)direction) > 0)
        return {};

    // The intersection is an infinite number on points
    if (glm::dot((glm::dvec3)t->norm, B - A) == 0) {
        if (t->isInside(A) || t->isInside(B))
            return {}; // return true;
    }

    long double x0 = A.x, y0 = A.y, z0 = A.z;
    long double x1 = B.x, y1 = B.y, z1 = B.z;

    long double a = t->norm.x, b = t->norm.y, c = t->norm.z;
    long double d = glm::dot(t->norm, t->vertices[0]);

    long double c1, c2, c3, c4;
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
            if (d - (c2 - c4))
                return {};

            return {}; // return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
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
                return {};

            return {}; // return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
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
                return {};

            return {}; //return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
        }

        solution = zIsFixed(A, B, (d - c2 - c4) / (c + c1 + c3));
    }

    long double distance1 = getEuclidianDistance2(A, solution.point);
    long double distance2 = getEuclidianDistance2(B, solution.point);

    if (distance1 > this->length || distance2 > this->length)
        return {};

    return {}; // return t->isInside(solution.point);
} // TODO return CollisionPoint
CollisionPoint Ray::checkCollision(Capsule *col) {
    return reverseCollisionPoint(col->checkCollision(this));
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
        glm::vec3 meanNormal = (mesh->body->vertices[mesh->body->indices[i]].Normal + mesh->body->vertices[mesh->body->indices[i + 1]].Normal + mesh->body->vertices[mesh->body->indices[i + 2]].Normal) / 3.0f;
        glm::vec3 faceNormal = glm::normalize(glm::cross(mesh->body->vertices[mesh->body->indices[i]].Position - mesh->body->vertices[mesh->body->indices[i + 2]].Position, mesh->body->vertices[mesh->body->indices[i]].Position - mesh->body->vertices[mesh->body->indices[i + 1]].Position));
        if (glm::dot(faceNormal, meanNormal) < 0)
            faceNormal = -1.0f * faceNormal;

        Triangle t(
                getTransformedVertex(mesh->localTransform, mesh->body->vertices[mesh->body->indices[i]].Position),
                getTransformedVertex(mesh->localTransform, mesh->body->vertices[mesh->body->indices[i + 1]].Position),
                getTransformedVertex(mesh->localTransform, mesh->body->vertices[mesh->body->indices[i + 2]].Position),
                glm::mat3(transpose(inverse(mesh->localTransform))) * faceNormal
        );

        result = col->checkCollision(&t).hasCollision;
        if (result) {
            hit->vertices.push_back(mesh->body->vertices[i]);
            hit->vertices.push_back(mesh->body->vertices[i+1]);
            hit->vertices.push_back(mesh->body->vertices[i+2]);
            hit->indices.push_back(hit->indices.size());
            hit->indices.push_back(hit->indices.size());
            hit->indices.push_back(hit->indices.size());
            total++;
        } else {

            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i]]);
            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i+1]]);
            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i+2]]);
            not_hit->indices.push_back(hit->indices.size());
            not_hit->indices.push_back(hit->indices.size());
            not_hit->indices.push_back(hit->indices.size());
        }
    }

    hit->prepare();
    not_hit->prepare();

    hit->localTransform.tr = mesh->body->localTransform.tr;
    hit->localTransform.rot = mesh->body->localTransform.rot;
    hit->localTransform.sc = mesh->body->localTransform.sc;

    not_hit->localTransform.tr = mesh->body->localTransform.tr;
    not_hit->localTransform.rot = mesh->body->localTransform.rot;
    not_hit->localTransform.sc = mesh->body->localTransform.sc;

    return meshes;
} // adapt for CollisionPoint

TriangleMesh::TriangleMesh(Mesh *mesh) {
    Mesh* m = new Mesh(*mesh);
    body = convertMesh2ColliderMesh(m);
}
CollisionPoint TriangleMesh::checkCollision(TriangleMesh *col) {
    return checkCollisionByTriangle(col);
}
CollisionPoint TriangleMesh::checkCollision(AABB *bv) {
    return checkCollisionByTriangle(bv);
}
CollisionPoint TriangleMesh::checkCollision(BoundingSphere *bv) {
    return checkCollisionByTriangle(bv);
}
CollisionPoint TriangleMesh::checkCollision(Ray *r) {
    return checkCollisionByTriangle(r);
}
CollisionPoint TriangleMesh::checkCollision(Triangle *t) {
    return checkCollisionByTriangle(t);
}
CollisionPoint TriangleMesh::checkCollisionByTriangle(Collider *col) {
    bool result = false;

    for (int i = 0; i < body->indices.size(); i += 3) {
        glm::vec3 meanNormal = body->vertices[i].Normal + body->vertices[i + 1].Normal + body->vertices[i + 2].Normal;
        glm::vec3 faceNormal = glm::normalize(glm::cross(body->vertices[i].Position - body->vertices[i + 2].Position, body->vertices[i].Position - body->vertices[i + 1].Position));
        if (glm::dot(faceNormal, meanNormal) < 0)
            faceNormal = -1.0f * faceNormal;
        Triangle t(
                getTransformedVertex(localTransform, body->vertices[i].Position),
                getTransformedVertex(localTransform, body->vertices[i + 1].Position),
                getTransformedVertex(localTransform, body->vertices[i + 2].Position),
                glm::mat3(transpose(inverse(localTransform))) * faceNormal
        );

        // [Stefan] Ma ocup eu aici
        //result = col->checkCollision(&t);
        //if (result)
        //    return result;
    }

    return {}; //return result;
} // TODO return CollisionPoint
CollisionPoint TriangleMesh::checkCollision(Capsule *col) {
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
CollisionPoint Triangle::checkCollision(TriangleMesh *col) {
    return reverseCollisionPoint(col->checkCollisionByTriangle(this));
}
CollisionPoint Triangle::checkCollision(Ray *r) {
    return reverseCollisionPoint(r->checkCollision(this));
}
CollisionPoint Triangle::checkCollision(Capsule *col) {
    return reverseCollisionPoint(col->checkCollision(this));
}
// https://gdbooks.gitbooks.io/3dcollisions/content/Chapter4/aabb-triangle.html
CollisionPoint Triangle::checkCollision(AABB *bv) {
    for (int i = 0; i < 3; ++i)
        if (bv->isInside(vertices[i])) return {}; // return true;
    Ray r0(vertices[0], glm::normalize(-vertices[0] + vertices[1]), glm::length(vertices[0] - vertices[1]));
    Ray r1(vertices[1], glm::normalize(-vertices[1] + vertices[2]), glm::length(vertices[1] - vertices[2]));
    Ray r2(vertices[0], glm::normalize(-vertices[0] + vertices[2]), glm::length(vertices[0] - vertices[2]));
    return {}; // return r0.checkCollision(bv) || r1.checkCollision(bv) || r2.checkCollision(bv);
} // TODO return CollisionPoint
// https://wickedengine.net/2020/04/26/capsule-collision-detection/
CollisionPoint Triangle::checkCollision(BoundingSphere *bv) {
    for (int i = 0; i < 3; ++i)
        if (bv->isInside(vertices[i])) return {}; // return true;
    Ray r0(vertices[0], glm::normalize(-vertices[0] + vertices[1]), glm::length(vertices[0] - vertices[1]));
    Ray r1(vertices[1], glm::normalize(-vertices[1] + vertices[2]), glm::length(vertices[1] - vertices[2]));
    Ray r2(vertices[0], glm::normalize(-vertices[0] + vertices[2]), glm::length(vertices[0] - vertices[2]));
    return {}; // return r0.checkCollision(bv) || r1.checkCollision(bv) || r2.checkCollision(bv);
} // TODO return CollisionPoint
// http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
CollisionPoint Triangle::checkCollision(Triangle *t) {
    bool twoway0 = twoway;

    twoway = true;

    Ray r0(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[1]), glm::length(t->vertices[0] - t->vertices[1]));
    Ray r1(t->vertices[1], glm::normalize(- t->vertices[1] + t->vertices[2]), glm::length(t->vertices[1] - t->vertices[2]));
    Ray r2(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[2]), glm::length(t->vertices[0] - t->vertices[2]));

    return {}; // bool result = r0.checkCollision(this) || r1.checkCollision(this) || r2.checkCollision(this);

    twoway = twoway0;

    return {}; // return result;
} // TODO return CollisionPoint
bool Triangle::isInside(glm::dvec3 point) {
    long double area = getTriangleArea2(vertices[0] - vertices[2], vertices[0] - vertices[1]);

    long double subarea1 = getTriangleArea2((glm::dvec3)vertices[0] - (glm::dvec3)vertices[1], (glm::dvec3)vertices[1] - point);
    long double subarea2 = getTriangleArea2((glm::dvec3)vertices[1] - (glm::dvec3)vertices[2], (glm::dvec3)vertices[2] - point);
    long double subarea3 = getTriangleArea2((glm::dvec3)vertices[0] - (glm::dvec3)vertices[2], (glm::dvec3)vertices[2] - point);

    //if (area < 0.0001)
    //std::cout << area << " area ";
    //std::cout << subarea1 + subarea2 + subarea3 << " sub ";
    //std::cout << subarea1 + subarea2 + subarea3 - area << " diff\n";
    //std::cout << fabs(subarea1 + subarea2 + subarea3 - getTriangleArea2((glm::dvec3)vertices[0] - (glm::dvec3)vertices[2], (glm::dvec3)vertices[0] - (glm::dvec3)vertices[1])) << "\n";
    //std::cout << (fabs(subarea1 + subarea2 + subarea3 - getTriangleArea2((glm::dvec3)vertices[0] - (glm::dvec3)vertices[2], (glm::dvec3)vertices[0] - (glm::dvec3)vertices[1])) <= EPS) << "\n";


    return fabs(subarea1 + subarea2 + subarea3 - getTriangleArea2((glm::dvec3)vertices[0] - (glm::dvec3)vertices[2], (glm::dvec3)vertices[0] - (glm::dvec3)vertices[1])) <= EPS;
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

    body = new ColliderMesh;

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
void Capsule::setTransform(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
    Collider::setTransform(pos, rot, scale);
    start = getTransformedVertex(localTransform, start0);
    end = getTransformedVertex(localTransform, end0);

    body->localTransform.tr = pos;
    body->localTransform.sc = scale;
    body->localTransform.rot = rot;
}
CollisionPoint Capsule::checkCollision(TriangleMesh *col) {
    return reverseCollisionPoint(col->checkCollision(this));
}
CollisionPoint Capsule::checkCollision(BoundingSphere *col) {
    glm::vec3 bestPoint = ClosestPointOnLineSegment(start, end, col->pos);
    BoundingSphere s(bestPoint, radius);
    return s.checkCollision(col);
}
CollisionPoint Capsule::checkCollision(AABB *col) {
    glm::vec3 bestPoint = ClosestPointOnLineSegment(start, end, (col->max + col->min)/2.0f);
    BoundingSphere s(bestPoint, radius);
    return s.checkCollision(col);
}
// https://wickedengine.net/2020/04/26/capsule-collision-detection/
CollisionPoint Capsule::checkCollision(Triangle *t) {
    // for triangle inside a capsule
    for (int i = 0; i < 3; ++i) {
        glm::vec3 mid = getMidPoint(start, end, t->vertices[i]);
         if(glm::length(mid - t->vertices[i]) < radius)
             return {}; // return true;
    }

    bool twoway = t->twoway;
    t->twoway = true;
    Ray r0(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[1]), glm::length(t->vertices[0] - t->vertices[1]));
    Ray r1(t->vertices[1], glm::normalize(- t->vertices[1] + t->vertices[2]), glm::length(t->vertices[1] - t->vertices[2]));
    Ray r2(t->vertices[0], glm::normalize(- t->vertices[0] + t->vertices[2]), glm::length(t->vertices[0] - t->vertices[2]));
    return {}; //bool result = r0.checkCollision(this) || r1.checkCollision(this) || r2.checkCollision(this);
    t->twoway = twoway;
    return {}; //return result;
} // TODO return CollisionPoint
CollisionPoint Capsule::checkCollision(Ray *col) {
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
    return {}; //return s.checkCollision(col);
}
// !source of inspiration: https://wickedengine.net/2020/04/26/capsule-collision-detection/
CollisionPoint Capsule::checkCollision(Capsule *col) {
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

    return {}; // return s1.checkCollision(&s2);
}
void Capsule::toString() {
    std::cout << "Capsule:\nStart Point: " << glm::to_string(start) << "\n\tEnd Point: "<< glm::to_string(end) << "\n\tRadius: " << radius << "\n";
}
#pragma endregion
