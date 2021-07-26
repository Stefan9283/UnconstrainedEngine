#include "Octree.h"
#include "Mesh.h"
#include "Shader.h"
#include <glm/gtx/string_cast.hpp>
#include "Debug.h"


Octree::Octree(Mesh *mesh, int level) {
    std::vector<Triangle> triangles;
    for (int i = 0; i < mesh->getTriangleCount(); ++i) {
        std::vector<Vertex> vertices = mesh->getTriangle(i);
        Triangle t(vertices[0].Position, vertices[1].Position, vertices[2].Position, glm::cross(vertices[0].Position - vertices[1].Position, vertices[1].Position - vertices[2].Position));
        triangles.push_back(t);
    }
    root = new OctreeNode(triangles, level);
}
void Octree::Draw(int maxdepth, Shader* s) {
    root->Draw(s, maxdepth);
}


OctreeNode::~OctreeNode() {
    for (OctreeNode* oct : children)
        delete oct;
    delete box;
}
OctreeNode::OctreeNode(std::vector<Triangle>& remainingTriangles, int level) {
    glm::vec3 min = remainingTriangles[0].vertices[0],
              max = remainingTriangles[0].vertices[0];

    for (int i = 0; i < remainingTriangles.size(); i++) {
        for (int j = 0; j < 3; ++j) {
            min = glm::min(min, remainingTriangles[i].vertices[j]);
            max = glm::max(max, remainingTriangles[i].vertices[j]);
        }
    }

    box = new AABB(min, max);
    box->body = AABB::generateNewMesh();
    box->body->solidON = false;
    box->body->wireframeON = true;
    box->parent = nullptr;

    if (level >= 1) {
        level--;
        divide(remainingTriangles, level);
    }
}
/*  z0      < z1          < z2
 * plane 1  plane 2 (mid) plane 3
 * 1 2 3    10 11 12      19 20 21 (max)
 * 4 5 6    13 14 15      22 23 24
 * 7 8 9    16 17 18      25 26 27
 * ^
 * min
 *
 * AABBs
 * low    high
 * 7 14   4 11
 * 16 23  13 20
 * 8 15   5 12
 * 17 24  14 21
 */
void OctreeNode::divide(std::vector<Triangle>& remainingTriangles, int level) {
    // divide box in 8 sectors if more than 0 triangles are in vec and add the new nodes
    glm::vec3 min = box->min;
    glm::vec3 max = box->max;
    glm::vec3 mid = (min + max) / 2.0f;

    std::vector<std::pair<AABB, std::vector<Triangle>>> boxes;

    // bottom
    boxes.push_back(std::make_pair(AABB(min, mid), std::vector<Triangle>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(mid.x, min.y, min.z), glm::vec3(max.x, mid.y, mid.z)), std::vector<Triangle>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(min.x, min.y, mid.z), glm::vec3(mid.x, mid.y, max.z)), std::vector<Triangle>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(mid.x, min.y, mid.z), glm::vec3(max.x, mid.y, max.z)), std::vector<Triangle>()));

    // top
    boxes.push_back(std::make_pair(AABB(mid, max), std::vector<Triangle>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(min.x, mid.y, min.z), glm::vec3(mid.x, max.y, mid.z)), std::vector<Triangle>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(mid.x, mid.y, min.z), glm::vec3(max.x, max.y, mid.z)), std::vector<Triangle>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(min.x, mid.y, mid.z), glm::vec3(mid.x, max.y, max.z)), std::vector<Triangle>()));

    for (Triangle& triangle : remainingTriangles) {
        for (int i = 0; i < boxes.size(); ++i) {
            if (boxes[i].first.checkCollision(&triangle).hasCollision) {
                boxes[i].second.push_back(triangle);
            }
        }
    }

    // divide some more
    if (level >= 0) {
        for (int j = 0; j < boxes.size(); ++j) {
            if (!boxes[j].second.empty()) {
                children.push_back(new OctreeNode(boxes[j].second, level));
            }
        }
    }
}

void OctreeNode::Draw(Shader* s, int level) {
    if (level <= 0) {
        box->Draw(s);
    } else {
        level--;
        for (auto &i : children)
            i->Draw(s, level);
    }
}
