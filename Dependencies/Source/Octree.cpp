#include "Octree.h"
#include "Mesh.h"
#include "Shader.h"
#include <glm/gtx/string_cast.hpp>

Octree::Octree(Mesh *mesh, int level) {
    std::vector<std::pair<int, std::vector<Vertex>>> triangles;
    for (int i = 0; i < mesh->getTriangleCount(); ++i)
        triangles.push_back(std::make_pair(i, mesh->getTriangle(i)));
    root = new OctreeNode(triangles, level);
}

OctreeNode::~OctreeNode() {
    for (OctreeNode* oct : children)
        delete oct;
    delete box;
}
OctreeNode::OctreeNode(std::vector<std::pair<int, std::vector<Vertex>>>& remainingTriangles, int level) {
    glm::vec3 min = remainingTriangles[0].second[0].Position,
              max = remainingTriangles[0].second[0].Position;

    for (int i = 0; i < remainingTriangles.size(); i++) {
        triangleIndices.push_back(remainingTriangles[i].first);
        for (int j = 0; j < remainingTriangles[i].second.size(); ++j) {
            min = glm::min(min, remainingTriangles[i].second[j].Position);
            max = glm::max(max, remainingTriangles[i].second[j].Position);
        }
    }

    box = new AABB(min, max);
    box->generateNewMesh();
    box->body->solidON = false;
    box->body->wireframeON = true;

    level--;

    if (level > 1)
        divide(remainingTriangles, level);
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
void OctreeNode::divide(std::vector<std::pair<int, std::vector<Vertex>>>& remainingTriangles, int level) {
    // divide box in 8 sectors if more than 0 triangles are in vec and add the new nodes

    glm::vec3 min = box->min;
    glm::vec3 max = box->max;

    glm::vec3 mid = (min + max) / 2.0f;

    std::vector<std::pair<AABB, std::vector<std::pair<int, std::vector<Vertex>>>>> boxes;

    // bottom
    boxes.push_back(std::make_pair(AABB(min, mid), std::vector<std::pair<int, std::vector<Vertex>>>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(mid.x, min.y, min.z), glm::vec3(max.x, mid.y, mid.z)), std::vector<std::pair<int, std::vector<Vertex>>>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(min.x, min.y, mid.z), glm::vec3(mid.x, mid.y, max.z)), std::vector<std::pair<int, std::vector<Vertex>>>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(mid.x, min.y, mid.z), glm::vec3(max.x, mid.y, max.z)), std::vector<std::pair<int, std::vector<Vertex>>>()));

    // top
    boxes.push_back(std::make_pair(AABB(mid, max), std::vector<std::pair<int, std::vector<Vertex>>>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(min.x, mid.y, min.z), glm::vec3(mid.x, max.y, mid.z)), std::vector<std::pair<int, std::vector<Vertex>>>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(mid.x, mid.y, min.z), glm::vec3(max.x, max.y, mid.z)), std::vector<std::pair<int, std::vector<Vertex>>>()));
    boxes.push_back(std::make_pair(AABB(glm::vec3(min.x, mid.y, mid.z), glm::vec3(mid.x, max.y, max.z)), std::vector<std::pair<int, std::vector<Vertex>>>()));

    for (std::pair<int, std::vector<Vertex>> triangle : remainingTriangles) {
        this->triangleIndices.push_back(triangle.first);
        Triangle t(triangle.second[0].Position, triangle.second[1].Position, triangle.second[2].Position, glm::vec3(0));
        for (int i = 0; i < 8; ++i) {
            if (boxes[i].first.checkCollision(&t).hasCollision)
                boxes[i].second.push_back(triangle);
        }
    }

    if (level > 0) {
        for (int j = 0; j < boxes.size(); ++j) {
            if (!boxes[j].second.empty())
                children.push_back(new OctreeNode(boxes[j].second, level));
        }
    }
}

void OctreeNode::Draw(Shader* s) {
    if (!children.size())
        box->body->Draw(s);
    else {
        for (auto &i : children)
            i->Draw(s);
    }
}
