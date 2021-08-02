
#ifndef PHYSICSENGINE_BVH_H
#define PHYSICSENGINE_BVH_H

#include "Common.h"

class Shader;
class Collider;
class CollisionPoint;
class RigidBody;
class BVHNode;


class BVHNode {
public:
    Collider* c = nullptr;
    RigidBody* rb = nullptr;
    BVHNode *parent = nullptr, * l = nullptr, * r = nullptr;
    glm::vec3 min{}, max{};

    void getCollisions(std::vector<CollisionPoint>& addHere, RigidBody* testWithMe);
    BVHNode(RigidBody* rb);
    BVHNode(BVHNode* n1, BVHNode* n2);
    glm::vec3 getMin();
    glm::vec3 getMax();
    void resizeParents();
    bool isLeaf();
    bool isRoot();
    bool isLeftChild();
    bool isRightChild();
    void clearTree();
    void asciiprint(int tabs = 0);
    void Draw(Shader* s);
};

class BVH {
public:
    BVHNode* root = nullptr;
    std::vector<BVHNode*> leafs;

    void* BVHNodeMaker;
    BVH(std::vector<RigidBody*>& rbs);
    ~BVH();
    std::vector<CollisionPoint> getCollisions(RigidBody* rb);
    void removeRigidBody(RigidBody* rb);
};




#endif //PHYSICSENGINE_BVH_H
