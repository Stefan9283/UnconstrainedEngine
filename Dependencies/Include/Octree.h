
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

#include "Colliders.h"
#include "Vertex.h"

class Shader;
class Mesh;
class OctreeNode;

class Octree {
public:
    OctreeNode* root = nullptr;
    Octree(Mesh* mesh, int level = 1);
    void Draw(int maxdepth, Shader* s);
};

class OctreeNode {
public:
    AABB* box = nullptr;
    std::vector<OctreeNode*> children;
    std::vector<uint32_t> triangleIndices;

    void Draw(Shader* s, int level = 0);

    OctreeNode(std::vector<Triangle>& remainingTriangles, int level);
    void divide(std::vector<Triangle>& remainingTriangles, int level);
    ~OctreeNode();
};

#endif //TRIANGLE_OCTREE_H
