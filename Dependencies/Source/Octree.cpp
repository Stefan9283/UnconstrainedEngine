
#include "Octree.h"
#include "Mesh.h"

Octree::Octree(Mesh *mesh) {
    std::vector<std::pair<int, std::vector<Vertex>>> triangles;
    for (int i = 0; i < mesh->getTriangleCount(); ++i)
        triangles.push_back(std::make_pair(i, mesh->getTriangle(i)));
    root = new OctreeNode(triangles);
}

OctreeNode::~OctreeNode() {
    for (OctreeNode* oct : children)
        delete oct;
    delete box;
}
OctreeNode::OctreeNode(std::vector<std::pair<int, std::vector<Vertex>>> remainingTriangles) {
    glm::vec3 min = remainingTriangles[0].second[0].Position,
              max = remainingTriangles[0].second[0].Position;

    for (int i = 0; i < remainingTriangles.size(); ++i) {
        triangleIndices.push_back(remainingTriangles[i].first);
        for (int j = 0; j < remainingTriangles[i].second.size(); ++j) {
            min = glm::min(min, remainingTriangles[i].second[j].Position);
            max = glm::max(min, remainingTriangles[i].second[j].Position);
        }
    }

    box = new AABB(min, max);

    divide(remainingTriangles);
}
void OctreeNode::divide(std::vector<std::pair<int, std::vector<Vertex>>> remainingTriangles) {
    // divide box in 8 sectors if more than 0 triangles are in vec and add the new nodes
} // TODO

