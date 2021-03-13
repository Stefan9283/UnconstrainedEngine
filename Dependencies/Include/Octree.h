
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

class Shader;
class Mesh;
class OctreeNode;

class Octree {
public:
    OctreeNode* root = nullptr;
    Octree(Mesh* mesh, int level = 1);
};

class OctreeNode {
public:
    AABB* box = nullptr;
    std::vector<OctreeNode*> children;
    std::vector<uint32_t> triangleIndices;

    void Draw(Shader* s);

    OctreeNode(std::vector<std::pair<int, std::vector<Vertex>>> remainingTriangles, int level);
    void divide(std::vector<std::pair<int, std::vector<Vertex>>> remainingTriangles, int level);
    ~OctreeNode();
};

#endif //TRIANGLE_OCTREE_H
