#include "Common.h"
#include <ObjLoad.h>
#include "Colliders.h"
#include "CollisionAlgos.h"
#include "Mesh.h"
#include "RigidBody.h"

extern Shader *s;

#pragma region utilities
glm::vec2 getPointOnRectangle(glm::vec2 leftDownCorner, glm::vec2 rightUpCorner, glm::vec2 point) {
    // Corners
    if (point.x < leftDownCorner.x && point.y < leftDownCorner.y)
        return leftDownCorner;

    if (point.x < leftDownCorner.x && point.y > rightUpCorner.y)
        return {leftDownCorner.x, rightUpCorner.y};

    if (point.y < leftDownCorner.y && point.x > rightUpCorner.x)
        return {rightUpCorner.x, leftDownCorner.y};

    if (point.x > rightUpCorner.x && point.y > rightUpCorner.y)
        return rightUpCorner;

    // Points on edges
    if (point.x < leftDownCorner.x && point.y >= leftDownCorner.y && point.y <= rightUpCorner.y)
        return {leftDownCorner.x, point.y};

    if (point.x > rightUpCorner.x && point.y >= leftDownCorner.y && point.y <= rightUpCorner.y)
        return {rightUpCorner.x, point.y};

    if (point.y < leftDownCorner.y && point.x >= leftDownCorner.x && point.x <= rightUpCorner.x)
        return {point.x, leftDownCorner.y};

    return {point.x, rightUpCorner.y};
}

ColliderMesh* convertMesh2ColliderMesh(Mesh* m) {
    auto* mesh = new ColliderMesh;
    mesh->indices = m->indices;
    mesh->vertices = m->vertices;
    mesh->prepare();
    delete m;
    return mesh;
}
glm::vec3 getTransformedVertex(glm::mat4 tr, glm::vec3 v) {
    return glm::vec3(tr * glm::vec4(v, 1.0f));
}
float getTriangleArea2(glm::vec3 edge1, glm::vec3 edge2) {
    glm::highp_vec3 N = glm::cross(edge1, edge2);
    return 0.5f * glm::sqrt(glm::dot(N, N));
}
std::vector<Mesh*> wasMeshHit(Collider* mesh, Collider* col) {
    Mesh* hit = new Mesh(), * not_hit = new Mesh();
    std::vector<Mesh*> meshes;
    meshes.push_back(hit);
    meshes.push_back(not_hit);
    bool result = false;

    int total = 0;

    for (size_t i = 0; i < mesh->body->indices.size(); i += 3) {
        glm::vec3 meanNormal = (mesh->body->vertices[mesh->body->indices[i]].Normal +
            mesh->body->vertices[mesh->body->indices[i + 1]].Normal +
            mesh->body->vertices[mesh->body->indices[i + 2]].Normal)
            / 3.0f;
        glm::vec3 faceNormal = glm::normalize(glm::cross(mesh->body->vertices[mesh->body->indices[i]].Position - mesh->body->vertices[mesh->body->indices[i + 2]].Position,
            mesh->body->vertices[mesh->body->indices[i]].Position - mesh->body->vertices[mesh->body->indices[i + 1]].Position));
        if (glm::dot(faceNormal, meanNormal) < 0)
            faceNormal = -1.0f * faceNormal;

        Triangle t(
            getTransformedVertex(mesh->getTransform(), mesh->body->vertices[mesh->body->indices[i]].Position),
            getTransformedVertex(mesh->getTransform(), mesh->body->vertices[mesh->body->indices[i + 1]].Position),
            getTransformedVertex(mesh->getTransform(), mesh->body->vertices[mesh->body->indices[i + 2]].Position),
            glm::mat3(transpose(inverse(mesh->getTransform()))) * faceNormal
        );

        result = col->checkCollision(&t).hasCollision;
        std::cout << result << "\n";
        if (result) {
            hit->vertices.push_back(mesh->body->vertices[i]);
            hit->vertices.push_back(mesh->body->vertices[i + 1]);
            hit->vertices.push_back(mesh->body->vertices[i + 2]);
            hit->indices.push_back((uint32_t)hit->indices.size());
            hit->indices.push_back((uint32_t)hit->indices.size());
            hit->indices.push_back((uint32_t)hit->indices.size());
            total++;
        }
        else {
            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i]]);
            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i + 1]]);
            not_hit->vertices.push_back(mesh->body->vertices[mesh->body->indices[i + 2]]);
            not_hit->indices.push_back((uint32_t)hit->indices.size());
            not_hit->indices.push_back((uint32_t)hit->indices.size());
            not_hit->indices.push_back((uint32_t)hit->indices.size());
        }
    }

    hit->prepare();
    not_hit->prepare();

    hit->localTransform.tr = mesh->localTransform.tr;
    hit->localTransform.rot = mesh->localTransform.rot;
    hit->localTransform.sc = mesh->localTransform.sc;


    not_hit->localTransform.tr = mesh->localTransform.tr;
    not_hit->localTransform.rot = mesh->localTransform.rot;
    not_hit->localTransform.sc = mesh->localTransform.sc;

    return meshes;
} // adapt for CollisionPoint
#pragma endregion

#pragma region Collider Specific Funcs

#pragma region Collider
void Collider::setType(colliderType type) {
    this->type = type;
}
Collider::~Collider() {
    delete body;
}
void Collider::Draw(Shader *shader) {
    if (body) 
        body->Draw(getTransform(), shader);
}


glm::mat4 Collider::getLocalTransform() {
    glm::mat4 T, R, S;
    T = glm::translate(glm::mat4(1), localTransform.tr);
    R = glm::toMat4(localTransform.rot);
    S = glm::scale(glm::mat4(1), localTransform.sc);
    return T * R * S;
}
glm::mat4 Collider::getTransform() {
    if (parent)
        return parent->getTransform() * getLocalTransform();
    else return getLocalTransform();
}


#pragma endregion
#pragma region ColliderMesh
ColliderMesh::~ColliderMesh() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

void ColliderMesh::prepare() {
    //assert(vertices.size() != 0);

    if (vertices.empty()) return;

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)nullptr);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

    glBindVertexArray(0);
}
void ColliderMesh::Draw(glm::mat4 parentMatrix, Shader *shader) {
    shader->setMat4("model", &parentMatrix);
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
void ColliderMesh::gui(int outIndex = 0) {

    if (ImGui::TreeNode((name + " " + std::to_string(outIndex)).c_str())) {
        ImGui::Checkbox(("wireframeON " + std::to_string(outIndex)).c_str(), &wireframeON);
        ImGui::SameLine();
        ImGui::Checkbox(("solidON " + std::to_string(outIndex)).c_str(), &solidON);

        ImGui::TreePop();
    }
}
#pragma endregion
#pragma region CollisionPoint
CollisionPoint::CollisionPoint(glm::vec3 A, glm::vec3 B) {
    if (A == B) {
        this->hasCollision = false;
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
    s.append("reversed: ").append(std::to_string(wasReversed)).append("\n");
    if (hasCollision)
        s.append("collision OK");
    else s.append("collision NO");
    return s;
}

#pragma endregion

#pragma region Sphere
glm::mat4 Sphere::getLocalTransform() {
    glm::mat4 T, S;
    T = glm::translate(glm::mat4(1), this->pos);
    S = glm::scale(glm::mat4(1), glm::vec3(radius));
    return T * S;
}

Sphere::Sphere(glm::vec3 pos, float radius, bool createBody) {
    setType(colliderType::sphere);
    this->pos = pos;
    this->radius = radius;
    if (createBody) {
        body = convertMesh2ColliderMesh(readObj("3D/Sphere.obj"));
        body->prepare();
    }
}
Sphere::Sphere(Mesh *mesh, bool createBody) {
    setType(colliderType::sphere);

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

std::string Sphere::toString() {
    return "Sphere:\n\tcenter: " + glm::to_string(getCurrentPosition()) + "\n\tradius: " + std::to_string(this->radius) + "\n";
}
bool Sphere::isInside(glm::vec3 point) {
    return glm::length(point - pos) < radius;
}
void Sphere::gui(int index) {
    ImGui::Text("Sphere Collider Settings");
    std::string s = "radius" + std::to_string(index);
    ImGui::SliderFloat(s.c_str(), &this->radius, 0, 100);
    radius = glm::max(0.1f, radius);
}

#pragma endregion
#pragma region AABB
glm::mat4 AABB::getLocalTransform() {
    glm::mat4 T, S;
    T = glm::translate(glm::mat4(1), localTransform.tr);
    S = glm::scale(glm::mat4(1), max - glm::vec3(0));
    return T * S;
}
glm::mat4 AABB::getTransform() {
    if (parent)
        return parent->getTranslationMatrix() * getLocalTransform();
    else return getLocalTransform();
}

AABB::AABB(float height, float width, float length, bool createBody) {
    setType(colliderType::aabb);
    height /= 2.0f;
    width /= 2.0f;
    length /= 2.0f;
    max = glm::vec3(width, height, length);
    min = - max;
    if (createBody)
        this->body = generateNewMesh();
}
AABB::AABB(glm::vec3 min, glm::vec3 max, bool createBody) {
    setType(colliderType::aabb);
    localTransform.tr = (max + min) / 2.0f;
    this->max = glm::vec3(max - localTransform.tr);
    this->min = -this->max;

    if (createBody)
        this->body = generateNewMesh();
}
AABB::AABB(Mesh* mesh, bool createBody) {
    setType(colliderType::aabb);
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

    if (createBody)
        body = generateNewMesh();
}

glm::vec3 AABB::getMin() {
    return this->min + getOffset();
}
glm::vec3 AABB::getMax() {
    return this->max + getOffset();
}
glm::vec3 AABB::getOffset() {
    glm::vec3 offs = localTransform.tr;
    if (parent)
        return offs + parent->position;
    return offs;
}

bool AABB::isInside(glm::vec3 point) {
    glm::vec3 minValues, maxValues;

    minValues = glm::min(getMin(), getMax());
    maxValues = glm::max(getMin(), getMax());

    return glm::clamp(point, minValues, maxValues) == point;
}
glm::vec3 AABB::closestPoint(glm::vec3 point) {
    glm::vec2 result{};
    // glm::vec3 newMin = getMin(), newMax = getMax();
    // varianta 1
    /*if(point.x > newMax.x)
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
        result.z = point.z;*/
    // varianta 2
    /*if (glm::abs(point.x - newMax.x) < glm::abs(point.x - newMin.x))
        result.x = newMax.x;
    else result.x = newMin.x;

    if (glm::abs(point.y - newMax.y) < glm::abs(point.y - newMin.y))
        result.y = newMax.y;
    else result.y = newMin.y;

    if (glm::abs(point.z - newMax.z) < glm::abs(point.z - newMin.z))
        result.z = newMax.z;
    else result.z = newMin.z;
    */

    glm::vec3 min = getMin(), max = getMax();

    // Point is inside
    if (isInside(point)) {
        double dist1 = abs(point.x - min.x);
        double dist2 = abs(point.x - max.x);

        double dist3 = abs(point.y - min.y);
        double dist4 = abs(point.y - max.y);

        double dist5 = abs(point.z - min.z);
        double dist6 = abs(point.z - max.z);

        double minDist = glm::min(glm::min(glm::min(dist1, dist2), glm::min(dist3, dist4)), glm::min(dist5, dist6));

        if (minDist == dist1)
            return {min.x, point.y, point.z};

        if (minDist == dist2)
            return {max.x, point.y, point.z};

        if (minDist == dist3)
            return {point.x, min.y, point.z};

        if (minDist == dist4)
            return {point.x, max.y, point.z};

        if (minDist == dist5)
            return {point.x, point.y, min.z};

        return {point.x, point.y, max.z};
    }

    // Point is outside, minimum distance is a perpendicular
    if (point.x < min.x && point.y >= min.y && point.y <= max.y && point.z >= min.z && point.z <= max.z)
        return {min.x, point.y, point.z};

    if (point.x > max.x && point.y >= min.y && point.y <= max.y && point.z >= min.z && point.z <= max.z)
        return {max.x, point.y, point.z};

    if (point.y < min.y && point.x >= min.x && point.x <= max.x && point.z >= min.z && point.z <= max.z)
        return {point.x, min.y, point.z};

    if (point.y > max.y && point.x >= min.x && point.x <= max.x && point.z >= min.z && point.z <= max.z)
        return {point.x, max.y, point.z};

    if (point.z < min.z && point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y)
        return {point.x, point.y, min.z};

    if (point.z > max.z && point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y)
        return {point.x, point.y, max.z};

    // Point is outside, minimum distance is an oblique line
    if (point.x < min.x)
        return {min.x, getPointOnRectangle({min.y, min.z}, {max.y, max.z}, {point.y, point.z})};

    if (point.x > max.x)
        return {max.x, getPointOnRectangle({min.y, min.z}, {max.y, max.z}, {point.y, point.z})};

    if (point.y < min.y) {
        result = getPointOnRectangle({min.x, min.z}, {max.x, max.z}, {point.x, point.z});
        return {result.x, min.y, result.y};
    }

    if (point.y > max.y) {
        result = getPointOnRectangle({min.x, min.z}, {max.x, max.z}, {point.x, point.z});
        return {result.x, max.y, result.y};
    }

    if (point.z < min.z)
        return {getPointOnRectangle({min.x, min.y}, {max.x, max.y}, {point.x, point.y}), min.z};

    return {getPointOnRectangle({min.x, min.y}, {max.x, max.y}, {point.x, point.y}), max.z};
}

ColliderMesh* AABB::generateNewMesh() {
    std::vector<Vertex> verticesPos = getVerices(glm::vec3(-1), glm::vec3(1));

    auto body = new ColliderMesh();

    for (auto v : verticesPos)
        body->vertices.push_back(v);

    body->indices = {
            3, 2, 0,
            0, 1, 3,
            6, 7, 4,
            4, 7, 5,
            7, 3, 5,
            5, 3, 1,
            2, 6, 0,
            0, 6, 4,
            0, 4, 1,
            4, 5, 1,
            2, 3, 7,
            7, 6, 2 };

    body->prepare();

    return body;
}
std::vector<Vertex> AABB::getVerices(glm::vec3 min_, glm::vec3 max_) {
    std::vector<Vertex> vertices;
    vertices.push_back(Vertex{ glm::vec3(min_.x, min_.y, min_.z),  glm::vec3(min_.x, min_.y, min_.z) });
    vertices.push_back(Vertex{ glm::vec3(min_.x, min_.y, max_.z),  glm::vec3(min_.x, min_.y, max_.z) });
    vertices.push_back(Vertex{ glm::vec3(min_.x, max_.y, min_.z),  glm::vec3(min_.x, max_.y, min_.z) });
    vertices.push_back(Vertex{ glm::vec3(min_.x, max_.y, max_.z),  glm::vec3(min_.x, max_.y, max_.z) });
    vertices.push_back(Vertex{ glm::vec3(max_.x, min_.y, min_.z),  glm::vec3(max_.x, min_.y, min_.z) });
    vertices.push_back(Vertex{ glm::vec3(max_.x, min_.y, max_.z),  glm::vec3(max_.x, min_.y, max_.z) });
    vertices.push_back(Vertex{ glm::vec3(max_.x, max_.y, min_.z),  glm::vec3(max_.x, max_.y, min_.z) });
    vertices.push_back(Vertex{ glm::vec3(max_.x, max_.y, max_.z),  glm::vec3(max_.x, max_.y, max_.z) });
    return vertices;
}

void AABB::gui(int index) {
    ImGui::Text("AABB Collider Settings");

    glm::vec3 size = max - min;

    ImGui::InputFloat("H ", &size.y);
    ImGui::InputFloat("L ", &size.x);
    ImGui::InputFloat("W ", &size.z);

    size = glm::max(glm::vec3(0.1f), size / 2.0f);
    max = size;
    min = - size;
}
std::string AABB::toString() {
    return "AABB:\n\tmin: " + glm::to_string(getMin()) + "\n\tmax: "+ glm::to_string(getMax()) + "\n";
}
#pragma endregion

#pragma region OBB
/*
glm::vec3 OBB::getMin() {
    return glm::vec3(getTransform() * glm::vec4(min + offset, 1.0f));
}
glm::vec3 OBB::getMax() {
    return glm::vec3(getTransform() * glm::vec4(max + offset, 1.0f));
}
*/
OBB::OBB(float height, float width, float length) {
    setType(colliderType::obb);
    height /= 2.0f;
    width /= 2.0f;
    length /= 2.0f;
    max = glm::vec3(width, height, length);
    min = -max;
    this->body = AABB::generateNewMesh();
}
void OBB::gui(int index) {
    ImGui::Text("OBB Collider Settings");

    glm::vec3 size = max - min;

    ImGui::InputFloat("H ", &size.y);
    ImGui::InputFloat("L ", &size.x);
    ImGui::InputFloat("W ", &size.z);

    size = glm::max(glm::vec3(0.1f), size / 2.0f);
    max = size;
    min = -size;
}
std::vector<Vertex> OBB::getVerices() {
    std::vector<Vertex> v = AABB::getVerices(this->min, this->max);
    for (size_t i = 0; i < v.size(); i++) {
        v[i].Position = getTransformedVertex(getTransform(), v[i].Position);
    }
    return v;
}
std::string OBB::toString() {
    std::string s = "OBB:\n";
    glm::vec3 sizes = max - min;

    s += "\tL: " + std::to_string(sizes.x);
    s += "\tH: " + std::to_string(sizes.y);
    s += "\tW: " + std::to_string(sizes.z);

    s += "\tVertices";
    for each (auto v in getVerices())
        s += "\n\t" + glm::to_string(v.Position);
    s += "\n";
    return s;
}

#pragma endregion

#pragma region Ray
Ray::Ray(glm::vec3 origin, glm::vec3 direction, float length, bool createMesh) {
    setType(colliderType::ray);
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
std::string Ray::toString() {
    return "Ray:\n\torigin: "
              + glm::to_string(this->origin) + "\n\tdirection: "
              + glm::to_string(this->direction) + "\n\tlength: "
              + std::to_string(this->length) + "\n\tend point: "
              + glm::to_string(origin + direction * length) + "\n";
}

#pragma endregion
#pragma region TriangleMesh

TriangleMesh::TriangleMesh(Mesh *mesh) {
    setType(colliderType::trianglemesh);
    Mesh* m = new Mesh(*mesh);
    body = convertMesh2ColliderMesh(m);
}

std::string TriangleMesh::toString() {
    return "TriangleMesh:\n\tvertices count:" + std::to_string(body->vertices.size()) + "\n\ttriangle count"
                    + std::to_string(body->indices.size()) + "\n";
}
#pragma endregion
#pragma region Triangle
Triangle::Triangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 norm) {
    setType(colliderType::triangle);
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

    return fabs(subarea1 + subarea2 + subarea3 - getTriangleArea2(vertices[0] - vertices[2], vertices[0] - vertices[1])) <= EPS;
}
std::string Triangle::toString() {
    std::string s = "Triangle:\n";
    for (int i = 0; i < 3; ++i) {
        s += "\tv" + std::to_string(i) + " " + glm::to_string(vertices[i]) + "\n";
    }
    s += "\tn " + glm::to_string(norm) + "\n";
    return s;
}
#pragma endregion
#pragma region Capsule
glm::vec3 Capsule::getEnd() {
    return getTransformedVertex(getTransform(), end);
}
glm::vec3 Capsule::getStart() {
    return getTransformedVertex(getTransform(), start);
}

void Capsule::createBody(glm::vec3 start, glm::vec3 end, float radius) {
    Mesh* Tube = readObj("3D/Tube.obj");
    Mesh* halfSphere = readObj("3D/halfSphere.obj");

    body = new ColliderMesh;

    body->vertices = Tube->vertices;
    body->indices = Tube->indices;

    float diff = glm::length(end - start);

    glm::mat4 scaleMatrixTube = glm::scale(glm::mat4(1), glm::vec3(radius, diff/2, radius));

    for (auto & vertice : body->vertices)
        vertice.Position = getTransformedVertex(scaleMatrixTube, vertice.Position);

    glm::mat4 scaleMatrixHalfSphere = glm::scale(glm::mat4(1), glm::vec3(radius));

    glm::mat4 modelEnd = glm::translate(glm::mat4(1), glm::vec3(0, diff/2, 0));
    glm::mat4 modelStart = glm::translate(glm::mat4(1), glm::vec3(0, -diff/2, 0)) * glm::rotate(glm::mat4(1), glm::radians(180.0f), glm::vec3(1, 0, 0));

    modelStart = modelStart * scaleMatrixHalfSphere;
    modelEnd = modelEnd * scaleMatrixHalfSphere;

    size_t indicesCount = Tube->indices.size();

    for (auto & vertice : halfSphere->vertices) {
        Vertex v{ getTransformedVertex(modelEnd, vertice.Position), vertice.Normal};
        body->vertices.push_back(v);
    }
    for (unsigned int indice : halfSphere->indices) {
        body->indices.push_back((uint32_t)(indice + indicesCount));
    }

    indicesCount = body->indices.size();

    for (auto & vertice : halfSphere->vertices) {
        Vertex v{ getTransformedVertex(modelStart, vertice.Position), vertice.Normal};
        body->vertices.push_back(v);
    }
    for (unsigned int indice : halfSphere->indices) {
        body->indices.push_back((uint32_t)(indice + indicesCount));
    }

    delete Tube;
    delete halfSphere;

    body->prepare();
}
Capsule::Capsule(float length, float radius) {
    setType(colliderType::capsule);
    this->start = glm::vec3(0, length / 2, 0);
    this->end = glm::vec3(0, - length / 2, 0);
    this->radius = radius;
    createBody(start, end, radius);
}
Capsule::Capsule(Mesh *mesh) {
    setType(colliderType::capsule);
    glm::vec3 min, max;
    min = mesh->vertices[0].Position;
    max = mesh->vertices[0].Position;

    auto center = glm::vec3(0);
    for (auto v : mesh->vertices) {
        center = center + v.Position;
        min = glm::min(v.Position, min);
        max = glm::max(v.Position, max);
    }

    center = center / (float)mesh->vertices.size();
    glm::vec3 r = glm::max(glm::abs(min - center), glm::abs(max - center));
    this->radius = glm::sqrt(2 * glm::max(r.x, glm::max(r.y, r.z)));
    float max_y = glm::max(glm::abs(min.y) - radius, glm::abs(max.y) - radius);
    this->start = glm::vec3(0, max_y, 0);
    this->end = - glm::vec3(0, max_y, 0);

    createBody(start, end, radius);
}

void Capsule::gui(int index) {
    ImGui::Text("Capsule Collider Settings");

    float length = glm::distance(start, end);
    float length2 = length;
    float radius2 = radius;

    ImGui::SliderFloat(("Radius" + std::to_string(index)).c_str(), &radius2, 0.1f, 100);
    ImGui::SliderFloat(("Length" + std::to_string(index)).c_str(), &length2, 0.1f, 100);

    if (length != length2 || radius != radius2) {
        length2 /= 2.0f;

        start = glm::vec3(0, length2, 0);
        end = glm::vec3(0, -length2, 0);
        radius = radius2;

        this->createBody(start, end, radius);
    }

}

std::string Capsule::toString() {
    return "Capsule:\nStart Point: " + glm::to_string(getStart())
            + "\n\tEnd Point: "+ glm::to_string(getEnd())
            + "\n\tRadius: " + std::to_string(radius) + "\n";
}
#pragma endregion

#pragma endregion

#pragma region checkCollision
CollisionPoint reverseCollisionPoint(CollisionPoint p) {
    glm::vec3 B = p.B;
    p.B = p.A;
    p.A = B;
    p.normal = - p.normal;
    p.wasReversed = true;
    return p;
}
CollisionPoint Collider::checkCollision(Collider* col) {
    if (col->type == colliderType::sphere) {
        return checkCollision((Sphere*)col);
    }
    if (col->type == colliderType::aabb) {
        return checkCollision((AABB*)col);
    }
    if (col->type == colliderType::ray) {
        return checkCollision((Ray*)col);
    }
    if (col->type == colliderType::trianglemesh) {
        return checkCollision((TriangleMesh*)col);
    }
    if (col->type == colliderType::triangle) {
        return checkCollision((Triangle*)col);
    }
    if (col->type == colliderType::capsule) {
        return checkCollision((Capsule*)col);
    }
    if (col->type == colliderType::obb) {
        return checkCollision((OBB*)col);
    }
    assert("Collider type was not identified");
    return {};
}

#pragma region Sphere
CollisionPoint Sphere::checkCollision(TriangleMesh *bv) {
    return collisionAlgos::checkCollision(bv, this);
}
CollisionPoint Sphere::checkCollision(AABB* bv) {
    return reverseCollisionPoint(collisionAlgos::checkCollision(bv, this));
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
    return reverseCollisionPoint(collisionAlgos::checkCollision(this, bv));
}
CollisionPoint Sphere::checkCollision(OBB* col)
{
    return CollisionPoint();
}
#pragma endregion
#pragma region AABB
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
    return reverseCollisionPoint(collisionAlgos::checkCollision(this, bv));
}
CollisionPoint AABB::checkCollision(OBB* col)
{
    return CollisionPoint();
}
#pragma endregion 
#pragma region OBB
CollisionPoint OBB::checkCollision(AABB* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint  OBB::checkCollision(Sphere* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint OBB::checkCollision(Ray* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint OBB::checkCollision(TriangleMesh* bv) {
    return collisionAlgos::checkCollision(bv, this);
}
CollisionPoint OBB::checkCollision(Triangle* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint  OBB::checkCollision(Capsule* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
CollisionPoint  OBB::checkCollision(OBB* bv) {
    return collisionAlgos::checkCollision(this, bv);
}
#pragma endregion
#pragma region Ray
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
CollisionPoint Ray::checkCollision(OBB* col) {
    return collisionAlgos::checkCollision(col, this);
}
#pragma endregion
#pragma region TriangleMesh
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
CollisionPoint TriangleMesh::checkCollision(OBB* col) {
    return collisionAlgos::checkCollision(this, col); // TODO ????
}
#pragma endregion
#pragma region Triangle
CollisionPoint Triangle::checkCollision(TriangleMesh *col) {
    return collisionAlgos::checkCollision(col, this);
}
CollisionPoint Triangle::checkCollision(Ray *r) {
    return collisionAlgos::checkCollision(this, r);
}
CollisionPoint Triangle::checkCollision(Capsule *col) {
    return collisionAlgos::checkCollision(this, col);
}
CollisionPoint Triangle::checkCollision(OBB* col) {
    return collisionAlgos::checkCollision(col, this);
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
} 
#pragma endregion
#pragma region Capsule
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
CollisionPoint Capsule::checkCollision(OBB* col) {
    return collisionAlgos::checkCollision(col, this);
}
#pragma endregion
#pragma endregion
