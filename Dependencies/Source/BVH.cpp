//
// Created by Stefan on 31-Jul-21.
//
#include "BVH.h"
#include "Colliders.h"
#include "RigidBody.h"

#define createMeshes false

#pragma region BVH

BVH::BVH(std::vector<RigidBody*>& rbs) { // TODO use the smallest surface heuristic
    leafs.resize(rbs.size());

    std::deque<BVHNode*> tree;
    tree.resize(rbs.size());
    for (int i = 0; i < rbs.size(); i++) {
        tree[i] = new BVHNode(rbs[i]);
        leafs[i] = tree[i];
    }
    
    while (tree.size() != 1) {

        BVHNode* n1 = tree.back();
        tree.pop_back();

        uint32_t bestNeigh = -1;
        float bestScore = 0;

        for (size_t i = 0; i < tree.size(); i++) {
            BVHNode* node = tree[i];
            float currentScore = heuristic(node, n1);
            if (currentScore > bestScore) {
                bestScore = currentScore;
                bestNeigh = i;
            }
        }
        tree.push_front(new BVHNode(n1, tree[bestNeigh]));
        tree.erase(tree.begin() + bestNeigh + 1);

        /*
        size_t size = tree.size();
        for (size_t i = 0; i < size; i += 2) {
            BVHNode* n1, * n2;
            n1 = tree.back();
            
            n2 = tree.back();
            tree.pop_back();
            auto node = new BVHNode(n1, n2);
            tree.push_front(node);
        }*/
    }
    this->root = tree[0];
}
BVH::~BVH() {
    delete root;
}

BVHNode* BVH::removeRigidBody(RigidBody* rb) {
    for (size_t i = 0; i < leafs.size(); i++) {
        BVHNode* node = leafs[i];
        if (node->rb == rb) {
            if (node->isRoot()) {
                root = nullptr;
            } else {
                BVHNode* replacement = nullptr;

                bool wasLeft = false;

                if (node->isLeftChild()) {
                    wasLeft = true;
                    replacement = node->parent->r;
                } else {
                    replacement = node->parent->l;
                }

                if (node->parent->isRoot()) {
                    root = replacement;
                    root->parent = nullptr;
                } else {
                    if (node->parent->isLeftChild()) {
                        node->parent->parent->l = replacement;
                    } else {
                        node->parent->parent->r = replacement;
                    }
                    replacement->parent = node->parent->parent;
                }
                replacement->resizeParents();
                node->parent->r = nullptr;
                node->parent->l = nullptr;
                delete node->parent;
                node->parent = nullptr;
            }
            leafs.erase(leafs.begin() + i);
            return node;
        }
    }
}
void BVH::insertRigidBody(RigidBody* rb) { // TODO
    BVHNode* rbNode = new BVHNode(rb);
    insertRigidBody(rbNode);
}
void BVH::insertRigidBody(BVHNode* rbNode) {
    leafs.push_back(rbNode);
    if (!root) {
        root = rbNode;
        return;
    }

    BVHNode* n = root;

    while (!n->isLeaf()) {
        if (heuristic(n->l, rbNode) > heuristic(n->r, rbNode))
            n = n->l;
        else
            n = n->r;
    }

    if (n->isRoot()) {
        root = new BVHNode(n, rbNode);
        return;
    }

    BVHNode* parent = n->parent;
    BVHNode* newNode;

    if (n->isLeftChild()) {
        newNode = new BVHNode(n, rbNode);
        parent->l = newNode;
    }
    else {
        newNode = new BVHNode(n, rbNode);
        parent->r = newNode;
    }

    newNode->parent = parent;
    newNode->resizeParents();
}
float BVH::heuristic(BVHNode* n1, BVHNode* n2) {
    glm::vec3 
        min = glm::min(n1->getMin(), n2->getMin()),
        max = glm::max(n1->getMax(), n2->getMax());
    glm::vec3 res = max - min;
    float costInv = res.x + res.y + res.z;
    if (!costInv) return 1;
    return 1 / costInv;
}
void BVH::getCollisions(std::vector<CollisionPoint>* AddHere, RigidBody* rb) {
    std::stack<BVHNode*> nodes;
    if (root)
        nodes.push(root);
    while (!nodes.empty()) {
        BVHNode* n = nodes.top();
        nodes.pop();

        if (n->isLeaf() && n->rb->id >= rb->id)
            continue;

        if (rb->collider->hasCollision(n->c)) {
            if (n->isLeaf()) {
                if (n->rb != rb) {
                    CollisionPoint p = n->rb->collider->checkCollision(rb->collider);
                    if (p.hasCollision)
                        AddHere->push_back(p);
                }
            }
            else {
                nodes.push(n->l);
                nodes.push(n->r);
            }
        }
    }
}

void BVH::gui() {
    ImGui::Checkbox("Draw everything", &drawEverything);
    ImGui::Checkbox("Draw nothing", &drawNothing);
    if (drawNothing)
        drawEverything = false;
    if (drawEverything)
        drawNothing = false;
    ImGui::SliderInt("Level Drawn", &levelDrawn, 0, 16);
}
void BVH::Draw(Shader* s) {
    if (drawNothing) return;
    if (drawEverything) {
        root->Draw(s);
        return;
    }
    root->Draw(s, levelDrawn);
}

#pragma endregion

#pragma region BVHNode
BVHNode::BVHNode(RigidBody* rb) {
    this->rb = rb;
    this->c = rb->collider;
    c = new AABB(min, max, createMeshes);
    resizeSelf();
}
BVHNode::BVHNode(BVHNode* n1, BVHNode* n2) {
    n1->parent = this;
    n2->parent = this;
    min = glm::min(n1->getMin(), n2->getMin());
    max = glm::max(n1->getMax(), n2->getMax());
    c = new AABB(min, max, createMeshes);
    l = n1;
    r = n2;
}
BVHNode::~BVHNode() {
    delete c;
    if (!isLeaf()) {
        delete l;
        delete r;
    }
}

void BVHNode::resizeSelf() {
    if (isLeaf()) {
        if (rb->collider->type == colliderType::aabb) {
            min = ((AABB*)rb->collider)->getMin();
            max = ((AABB*)rb->collider)->getMax();
        }
        else if (rb->collider->type == colliderType::sphere) {
            max = ((Sphere*)rb->collider)->getCurrentPosition() + ((Sphere*)rb->collider)->radius;
            min = ((Sphere*)rb->collider)->getCurrentPosition() - ((Sphere*)rb->collider)->radius;
        }
        else if (rb->collider->type == colliderType::capsule) {
            glm::vec3 start, end;
            start = ((Capsule*)rb->collider)->getStart();
            end = ((Capsule*)rb->collider)->getEnd();
            float radius = ((Capsule*)rb->collider)->radius;
            min = glm::min(start, end) - glm::vec3(radius);
            max = glm::max(start, end) + glm::vec3(radius);
        }
        ((AABB*)c)->refit(min, max);
    } else {
        min = glm::min(l->getMin(), r->getMin());
        max = glm::max(l->getMax(), r->getMax());
        ((AABB*)c)->refit(min, max);
    }
}
void BVHNode::resizeParents() {
    BVHNode* n = parent;
    while (n) {
        n->resizeSelf();
        n = n->parent;
    }
}
glm::vec3& BVHNode::getMax() { return max; }
glm::vec3& BVHNode::getMin() { return min; }
bool BVHNode::isLeaf() {
    return rb != nullptr;
}
bool BVHNode::isRoot() {
    return parent == nullptr;
}
bool BVHNode::isLeftChild() {
    if (isRoot()) return true;
    return parent->l == this;
}
bool BVHNode::isRightChild() {
    if (isRoot()) return true;
    return parent->r == this;
}
void BVHNode::asciiprint(int tabs) {
    for (size_t i = 0; i < tabs; i++)
        std::cout << "\t";
    
    if (tabs == 0)
        std::cout << this << " " << parent << " ";

    if (isLeaf()) {
        std::cout << "Rb " << rb;
        //std::cout << glm::to_string(min) << glm::to_string(max);
        std::cout << "\n";
    } else {
        std::cout << "mid ";
        //std::cout << glm::to_string(min) << glm::to_string(max);
        std::cout << "\n";
        
        //std::cout << "l";
        std::cout << l << " " << l->parent << " L ";

        l->asciiprint(tabs + 1);
        //std::cout << "r";
        std::cout << r << " " << r->parent << " R ";
        r->asciiprint(tabs + 1);
    }
}
void BVHNode::Draw(Shader* s) {
    c->Draw(s);
    if (!isLeaf()) {
        l->Draw(s);
        r->Draw(s);
    }
}
void BVHNode::Draw(Shader* s, int desiredLevel) {
    if (desiredLevel == 0) {
        c->Draw(s);
        return;
    }
    if (!isLeaf()) {
        desiredLevel--;
        l->Draw(s, desiredLevel);
        r->Draw(s, desiredLevel);
    }
}
#pragma endregion
