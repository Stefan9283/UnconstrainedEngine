//
// Created by Stefan on 22-Feb-21.
//

#include "Common.h"
#include <ObjLoad.h>
#include "Colliders.h"
#include "Mesh.h"
#include "CollisionALgos.h"
#include "RigidBody.h"

extern Shader *s;

ColliderMesh* convertMesh2ColliderMesh(Mesh* m) {
    ColliderMesh* mesh = new ColliderMesh;
    mesh->indices = m->indices;
    mesh->vertices = m->vertices;
    mesh->prepare();
    delete m;
    return mesh;
}

#pragma region Collider Specific Funcs

#pragma region Collider
Collider::~Collider() {
    if (body)
        delete body;
}
void Collider::Draw(Shader *shader) {
    if (body) {
        if (parent) {
            body->Draw(parent->getTransform() * getLocalTransform(), shader);
        }
        else {
            body->Draw(getLocalTransform(), shader);
        }
    }
}
glm::mat4 Collider::getLocalTransform() {
    return localTransform; 
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
        this->hasCollision = true; // TODO make this false
        this->normal = glm::vec3 (0);
        this->B = B;
        this->A = A;
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
std::string CollisionPoint::toString() {
    std::string s = "";
    s.append("A: ").append(glm::to_string(A)).append("\n");
    s.append("B: ").append(glm::to_string(B)).append("\n");
    s.append("depth: ").append(std::to_string(depth)).append("\n");
    s.append("normal: ").append(glm::to_string(normal)).append("\n");
    if (hasCollision)
        s.append("collision OK");
    else s.append("collision NO");
    return s;
}

#pragma endregion

#pragma region Sphere
Sphere::Sphere(glm::vec3 pos, float radius) {
    this->pos = pos;
    this->radius = radius;
}
Sphere::Sphere(Mesh *mesh) {
    pos = glm::vec3(0);

    for (Vertex v : mesh->vertices)
        pos = pos + v.Position;
    pos = pos / (float)mesh->vertices.size();
    for (auto v : mesh->vertices) {
        float currentRadius = glm::length(pos - v.Position);
        if (currentRadius > radius) radius = currentRadius;
    }
    body = convertMesh2ColliderMesh(readObj("3D/Sphere.obj"));
    for (auto & vertice : body->vertices)
        vertice.Position *= radius;
    body->prepare();
}

glm::vec3 Sphere::getCurrentPosition() {
    if (this->parent)
        return pos + parent->position;
    return pos;
}

void Sphere::update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
    this->pos += pos;
}
void Sphere::toString() {
    std::cout << "Sphere center: " << glm::to_string(this->pos) << " " << this->radius << "\n";
}
bool Sphere::isInside(glm::vec3 point) {
    return glm::length(point - pos) < radius;
}

#pragma endregion
#pragma region AABB
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
    
    body = generateNewMesh();
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
    std::vector<Vertex> verticesPos = generateVerices(glm::vec3(-1), glm::vec3(1));

    body = new ColliderMesh();

    for (auto v : verticesPos)
        body->vertices.push_back(v);

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


glm::vec3 AABB::getMin() {
    return this->min + getOffset();
}
glm::vec3 AABB::getMax() {
    return this->max + getOffset();
}

void AABB::Draw(Shader *shader) {
    if (body)
        body->Draw(getLocalTransform(), shader);
}
glm::vec3 AABB::getOffset() {
    glm::vec3 offs = this->offset;
    if (parent)
        return parent->position;
    return offs;
}

glm::mat4 AABB::getLocalTransform() {
    if (!parent)
        return localTransform;
    else 
        return parent->getTransform() * localTransform;
}


AABB::AABB(glm::vec3 min, glm::vec3 max) {
    this->min = min;
    this->max = max;
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
    glm::vec3 result{};

    glm::vec3 newMin = getMin(), newMax = getMax();

    /*
    // varianta 1
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
    
    // varianta 2
    if (glm::abs(point.x - newMax.x) < glm::abs(point.x - newMin.x))
        result.x = newMax.x;
    else result.x = newMin.x;

    if (glm::abs(point.y - newMax.y) < glm::abs(point.y - newMin.y))
        result.y = newMax.y;
    else result.y = newMin.y;

    if (glm::abs(point.z - newMax.z) < glm::abs(point.z - newMin.z))
        result.z = newMax.z;
    else result.z = newMin.z;
    */
    // TODO Ovidiu AABB point projection
    // Your code here
    
    
    // Your code here

    return result;
}

#pragma endregion
#pragma region Ray
Ray::Ray(glm::vec3 origin, glm::vec3 direction, float length, bool createMesh) {
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
        float x = (float)cam->lastx_mouse - (float) wi / 2.0f;
        float y = (float)he - (float)cam->lasty_mouse - (float) he / 2.0f;

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

void Ray::toString() {
    std::cout << "Ray:\n\torigin: "
              << glm::to_string(this->origin) << "\n\tdirection: "
              << glm::to_string(this->direction) << "\n\tlength: "
              << this->length << "\n\tend point: "
              << glm::to_string(origin + direction * length) << "\n";
}
#pragma endregion
#pragma region TriangleMesh


glm::vec3 getTransformedVertex(glm::mat4 tr, glm::vec3 v) {
    return glm::vec3(tr * glm::vec4(v, 1.0f));
}
float getTriangleArea2(glm::vec3 edge1, glm::vec3 edge2) {
    glm::highp_vec3 N = glm::cross(edge1, edge2);
    return 0.5f * glm::sqrt(glm::dot(N, N));
}


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
            hit->indices.push_back((uint32_t)hit->indices.size());
            hit->indices.push_back((uint32_t)hit->indices.size());
            hit->indices.push_back((uint32_t)hit->indices.size());
            total++;
        } else {

            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i]]);
            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i+1]]);
            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i+2]]);
            not_hit->indices.push_back((uint32_t)hit->indices.size());
            not_hit->indices.push_back((uint32_t)hit->indices.size());
            not_hit->indices.push_back((uint32_t)hit->indices.size());
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

bool Triangle::isInside(glm::vec3 point) {
    float area = getTriangleArea2(vertices[0] - vertices[2], vertices[0] - vertices[1]);

    float subarea1 = getTriangleArea2(vertices[0] - vertices[1], vertices[1] - point);
    float subarea2 = getTriangleArea2(vertices[1] - vertices[2], vertices[2] - point);
    float subarea3 = getTriangleArea2(vertices[0] - vertices[2], vertices[2] - point);

    //if (area < 0.0001)
    //std::cout << area << " area ";
    //std::cout << subarea1 + subarea2 + subarea3 << " sub ";
    //std::cout << subarea1 + subarea2 + subarea3 - area << " diff\n";
    //std::cout << fabs(subarea1 + subarea2 + subarea3 - getTriangleArea2(vertices[0] - vertices[2], vertices[0] - vertices[1])) << "\n";
    //std::cout << (fabs(subarea1 + subarea2 + subarea3 - getTriangleArea2(vertices[0] - vertices[2], vertices[0] - vertices[1])) <= EPS) << "\n";


    return fabs(subarea1 + subarea2 + subarea3 - getTriangleArea2(vertices[0] - vertices[2], vertices[0] - vertices[1])) <= EPS;
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
glm::vec3 Capsule::getEnd() {
    glm::vec4 res;
    if (this->parent) {
        res = parent->getTransform() * localTransform * glm::vec4(end, 1);
    }
    else {
        res = localTransform * glm::vec4(end, 1);
    }
    return glm::vec3(res);
}
glm::vec3 Capsule::getStart() {
    glm::vec4 res;
    if (this->parent) {
        res = parent->getTransform() * localTransform * glm::vec4(start, 1);
    }
    else {
        res = localTransform * glm::vec4(start, 1);
    }
    return glm::vec3(res);

}
Capsule::Capsule(glm::vec3 start, glm::vec3 end, float radius) {
    this->start = start;
    this->end = end;
    this->radius = radius;


    Mesh* Tube = readObj("3D/Tube.obj");
    Mesh* halfSphere = readObj("3D/halfSphere.obj");

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

    for (auto i = 0; i < halfSphere->vertices.size(); ++i) {
        Vertex v{ getTransformedVertex(modelEnd, halfSphere->vertices[i].Position), halfSphere->vertices[i].Normal};
        body->vertices.push_back(v);
    }
    for (auto j = 0; j < halfSphere->indices.size(); ++j) {
        body->indices.push_back((uint32_t)(halfSphere->indices[j] + indicesCount));
    }

    indicesCount = body->indices.size();

    for (auto i = 0; i < halfSphere->vertices.size(); ++i) {
        Vertex v{ getTransformedVertex(modelStart, halfSphere->vertices[i].Position), halfSphere->vertices[i].Normal};
        body->vertices.push_back(v);
    }
    for (auto j = 0; j < halfSphere->indices.size(); ++j) {
        body->indices.push_back((uint32_t)(halfSphere->indices[j] + indicesCount));
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
void Capsule::update(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
    Collider::update(pos, rot, scale);
    start = getTransformedVertex(localTransform, start0);
    end = getTransformedVertex(localTransform, end0);

    body->localTransform.tr = pos;
    body->localTransform.sc = scale;
    body->localTransform.rot = rot;
}

void Capsule::toString() {
    std::cout << "Capsule:\nStart Point: " << glm::to_string(start) << "\n\tEnd Point: "<< glm::to_string(end) << "\n\tRadius: " << radius << "\n";
}
#pragma endregion

#pragma endregion

#pragma region checkCollision
CollisionPoint reverseCollisionPoint(CollisionPoint p) {
    glm::vec3 B = p.B;
    p.B = p.A;
    p.A = B;
    return p;
}


CollisionPoint Collider::checkCollision(Collider* col) {
    if (dynamic_cast<Sphere*>(col)) {
        return checkCollision(dynamic_cast<Sphere*>(col));
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
    return {};
}

CollisionPoint Sphere::checkCollision(TriangleMesh *bv) {
    return collisionAlgos::checkCollision(bv, this);
}
CollisionPoint Sphere::checkCollision(AABB* bv) {
    return collisionAlgos::checkCollision(bv, this);
}
CollisionPoint Sphere::checkCollision(Sphere* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint Sphere::checkCollision(Ray *r) {
    return collisionAlgos::checkCollision(this, r);
}
CollisionPoint Sphere::checkCollision(Triangle *t) {
    return collisionAlgos::checkCollision(this, t);
}
CollisionPoint Sphere::checkCollision(Capsule *bv) {
    return collisionAlgos::checkCollision(this, bv);
} // TODO return CollisionPoint

CollisionPoint AABB::checkCollision(TriangleMesh *bv) {
    return collisionAlgos::checkCollision(bv, this);
}
CollisionPoint AABB::checkCollision(AABB* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint AABB::checkCollision(Sphere* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint AABB::checkCollision(Ray *r) {
    return collisionAlgos::checkCollision(this, r);
}
CollisionPoint AABB::checkCollision(Triangle *t) {
    return collisionAlgos::checkCollision(this, t);
}
CollisionPoint AABB::checkCollision(Capsule *bv) {
    return collisionAlgos::checkCollision(this, bv);
}

CollisionPoint Ray::checkCollision(TriangleMesh *col) {
    return collisionAlgos::checkCollision(col, this);
}
CollisionPoint Ray::checkCollision(Sphere* bv) {
    return collisionAlgos::checkCollision(bv, this);
}
CollisionPoint Ray::checkCollision(AABB* bv) {
    return collisionAlgos::checkCollision(bv, this);
}
CollisionPoint Ray::checkCollision(Ray* r) {
    return collisionAlgos::checkCollision(this, r);
}
CollisionPoint Ray::checkCollision(Triangle *t) {
    return collisionAlgos::checkCollision(t, this);
}
CollisionPoint Ray::checkCollision(Capsule *col) {
    return collisionAlgos::checkCollision(col, this);
}

CollisionPoint TriangleMesh::checkCollision(TriangleMesh *bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint TriangleMesh::checkCollision(AABB *bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint TriangleMesh::checkCollision(Sphere *bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint TriangleMesh::checkCollision(Ray *r) {
    return collisionAlgos::checkCollision(this, r);
}
CollisionPoint TriangleMesh::checkCollision(Triangle *t) {
    return collisionAlgos::checkCollision(this, t);
}
CollisionPoint TriangleMesh::checkCollision(Capsule *col) {
    return collisionAlgos::checkCollision(this, col);
}

CollisionPoint Triangle::checkCollision(TriangleMesh *col) {
    return collisionAlgos::checkCollision(col, this);
}
CollisionPoint Triangle::checkCollision(Ray *r) {
    return collisionAlgos::checkCollision(this, r);
}
CollisionPoint Triangle::checkCollision(Capsule *col) {
    return collisionAlgos::checkCollision(this, col);
}
// https://gdbooks.gitbooks.io/3dcollisions/content/Chapter4/aabb-triangle.html
CollisionPoint Triangle::checkCollision(AABB *bv) {
    return collisionAlgos::checkCollision(bv, this);
}
// https://wickedengine.net/2020/04/26/capsule-collision-detection/
CollisionPoint Triangle::checkCollision(Sphere *bv) {
    return collisionAlgos::checkCollision(bv, this);
}
// http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
CollisionPoint Triangle::checkCollision(Triangle *t) {
    return collisionAlgos::checkCollision(this, t);
} // TODO return CollisionPoint

CollisionPoint Capsule::checkCollision(TriangleMesh *col) {
    return collisionAlgos::checkCollision(col, this);
}
CollisionPoint Capsule::checkCollision(Sphere *col) {
    return collisionAlgos::checkCollision(col, this);
}
CollisionPoint Capsule::checkCollision(AABB *bv) {
    return collisionAlgos::checkCollision(bv, this);
}
// https://wickedengine.net/2020/04/26/capsule-collision-detection/
CollisionPoint Capsule::checkCollision(Triangle *t) {
    return collisionAlgos::checkCollision(t, this);
}
CollisionPoint Capsule::checkCollision(Ray *bv) {
    return collisionAlgos::checkCollision(this, bv);
}
// !source of inspiration: https://wickedengine.net/2020/04/26/capsule-collision-detection/
CollisionPoint Capsule::checkCollision(Capsule *bv) {
    return collisionAlgos::checkCollision(this, bv);
}
#pragma endregion
