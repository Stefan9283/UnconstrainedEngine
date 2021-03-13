
#ifndef TRIANGLE_BVH_H
#define TRIANGLE_BVH_H

#include <vector>

class Mesh;
class AABB;
class BVHNode;

class BVH {
public:
    BVHNode* root = nullptr;
    BVH(std::vector<Mesh*> meshes);
    ~BVH();
};

class BVHNode {
public:
    AABB* box;
    Mesh* mesh = nullptr;
    std::vector<BVHNode*> children;
    BVHNode(std::vector<Mesh*> meshes);
    ~BVHNode();
};

#endif //TRIANGLE_BVH_H
