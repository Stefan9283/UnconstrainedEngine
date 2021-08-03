
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

    BVHNode(RigidBody* rb);
    BVHNode(BVHNode* n1, BVHNode* n2);
    ~BVHNode();

    void getCollisions(std::vector<CollisionPoint>* addHere, 
                        RigidBody* testWithMe);

    glm::vec3& getMin();
    glm::vec3& getMax();

    void resizeParents();
    bool isLeaf();
    bool isRoot();
    bool isLeftChild();
    bool isRightChild();
    void asciiprint(int tabs = 0);
    
    void Draw(Shader* s);
    void Draw(Shader* s, int desiredLevel);
};

class BVH {
public:
    BVHNode* root = nullptr;
    std::vector<BVHNode*> leafs;

    bool drawEverything = true;
    int levelDrawn = 0;
    bool drawNothing = false;

    void* BVHNodeMaker;
    BVH(std::vector<RigidBody*>& rbs);
    ~BVH();
    void getCollisions(std::vector<CollisionPoint>*  AddHere, RigidBody* rb);
    void removeRigidBody(RigidBody* rb);
    void insertRigidBody(RigidBody* rb);
    static float heuristic(BVHNode* n1, BVHNode* n2);
    void gui();
    void Draw(Shader* s);
};

#endif //PHYSICSENGINE_BVH_H
