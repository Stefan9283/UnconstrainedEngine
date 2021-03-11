
/*
 *      ▒█████   ▄████▄  ▄▄▄█████▓ ██▀███  ▓█████ ▓█████
 *     ▒██▒  ██▒▒██▀ ▀█  ▓  ██▒ ▓▒▓██ ▒ ██▒▓█   ▀ ▓█   ▀
 *     ▒██░  ██▒▒▓█    ▄ ▒ ▓██░ ▒░▓██ ░▄█ ▒▒███   ▒███
 *     ▒██   ██░▒▓▓▄ ▄██▒░ ▓██▓ ░ ▒██▀▀█▄  ▒▓█  ▄ ▒▓█  ▄
 *     ░ ████▓▒░▒ ▓███▀ ░  ▒██▒ ░ ░██▓ ▒██▒░▒████▒░▒████▒
 *     ░ ▒░▒░▒░ ░ ░▒ ▒  ░  ▒ ░░   ░ ▒▓ ░▒▓░░░ ▒░ ░░░ ▒░ ░
 *       ░ ▒ ▒░   ░  ▒       ░      ░▒ ░ ▒░ ░ ░  ░ ░ ░  ░
 * */

#ifndef TRIANGLE_OCTREE_H
#define TRIANGLE_OCTREE_H

#include "BoundingVolumes.h"
#include "Vertex.h"

class Mesh;
class OctreeNode;

class Octree {
public:
    OctreeNode* root;

    Octree(Mesh* mesh);
};

class OctreeNode {
public:
    AABB* box;
    std::vector<OctreeNode*> children;
    std::vector<uint32_t> triangleIndices;

    OctreeNode(std::vector<std::pair<int, std::vector<Vertex>>> remainingTriangles);
    ~OctreeNode();
    void divide(std::vector<std::pair<int, std::vector<Vertex>>> remainingTriangles);
};

#endif //TRIANGLE_OCTREE_H
