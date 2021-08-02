//
// Created by Stefan on 31-Jul-21.
//
#include "BVH.h"
#include "Colliders.h"
#include "RigidBody.h"

#define createMeshes false

BVH::BVH(std::vector<RigidBody*>& rbs) {
    /*
    std::sort(rbs.begin(), rbs.end(), [](RigidBody* rb1, RigidBody* rb2) {
        return glm::dot(rb1->position, rb1->position) < glm::dot(rb2->position, rb2->position);
    });*/
    

    leafs.resize(rbs.size());

    std::deque<BVHNode*> tree;
    tree.resize(rbs.size());
    for (int i = 0; i < rbs.size(); i++) {
        tree[i] = new BVHNode(rbs[i]);
        leafs[i] = tree[i];
    }
    
    while (tree.size() != 1) {
        size_t size = tree.size();
        for (size_t i = 0; i < size; i += 2) {
            BVHNode* n1, * n2;
            n1 = tree.back();
            tree.pop_back();
            n2 = tree.back();
            tree.pop_back();
            auto node = new BVHNode(n1, n2);
            tree.push_front(node);
        }
    }
    this->root = tree[0];
}
BVH::~BVH() {
    if (root)
        root->clearTree();
    delete root;
}
void BVH::removeRigidBody(RigidBody* rb) {
    for (auto node : leafs)
        if (node->rb == rb) {
            if (node->isRoot()) {
                root = nullptr;
                node->clearTree();
            } else {
                BVHNode* replacement = nullptr;

                if (node->isLeftChild()) {
                    replacement = node->parent->r;
                    node->parent->r = nullptr;
                } else {
                    replacement = node->parent->l;
                    node->parent->l = nullptr;
                }

                if (node->parent->isRoot()) {
                    root = replacement;
                    root->parent = nullptr;
                } else {
                    if (node->parent->isLeftChild())
                        node->parent->parent->l = replacement;
                    else
                        node->parent->parent->r = replacement;
                    replacement->parent = node->parent->parent;
                }
                replacement->resizeParents();
                node->parent->clearTree();
            }
            break;
        }
}
std::vector<CollisionPoint> BVH::getCollisions(RigidBody* rb) {
    std::vector<CollisionPoint> points;
    if (rb)
        root->getCollisions(points, rb);

    return points;
}

void BVHNode::getCollisions(std::vector<CollisionPoint>& addHere, RigidBody* testWithMe) {
    CollisionPoint p = testWithMe->collider->checkCollision(c);

    if (p.hasCollision) {
        if (isLeaf()) {
            if (rb != testWithMe)
                addHere.push_back(p);
        } else {
            l->getCollisions(addHere, testWithMe);
            r->getCollisions(addHere, testWithMe);
        }
    }
}

BVHNode::BVHNode(RigidBody* rb) {
    this->rb = rb;
    this->c = rb->collider;

    if (rb->collider->type == colliderType::aabb) {
        min = ((AABB*)rb->collider)->getMin();
        max = ((AABB*)rb->collider)->getMax();
    } else if (rb->collider->type == colliderType::sphere) {
        max = ((Sphere*)rb->collider)->getCurrentPosition() + ((Sphere*)rb->collider)->radius;
        min = ((Sphere*)rb->collider)->getCurrentPosition() - ((Sphere*)rb->collider)->radius;
    } else if (rb->collider->type == colliderType::capsule) {
        glm::vec3 start, end;
        start = ((Capsule*)rb->collider)->getStart();
        end = ((Capsule*)rb->collider)->getEnd();
        float radius = ((Capsule*)rb->collider)->radius;
        min = glm::min(start, end) - glm::vec3(radius);
        max = glm::max(start, end) + glm::vec3(radius);
    }
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

glm::vec3 BVHNode::getMax() { return max; }
glm::vec3 BVHNode::getMin() { return min; }

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
void BVHNode::clearTree() {
    if (!isLeaf()) {
        if (l)
            l->clearTree();
        if (r)
            r->clearTree();
        delete c;
        delete l;
        delete r;
    }
}

void BVHNode::asciiprint(int tabs) {
    for (size_t i = 0; i < tabs; i++)
        std::cout << "\t";
    
    if (tabs == 0)
        std::cout << this << " " << parent << " ";

    if (isLeaf()) {
        std::cout << "Rb " << rb;
        std::cout << glm::to_string(min) << glm::to_string(max);
        std::cout << "\n";
    } else {
        std::cout << "mid ";
        std::cout << glm::to_string(min) << glm::to_string(max);
        std::cout << "\n";
        
        //std::cout << "l";
        std::cout << l << " " << l->parent << " L ";

        l->asciiprint(tabs + 1);
        //std::cout << "r";
        std::cout << r << " " << r->parent << " R ";
        r->asciiprint(tabs + 1);
    }
}
void BVHNode::resizeParents() {
    BVHNode* n = parent;
    while (n) {
        n->min = glm::min(n->l->getMin(), n->r->getMin());
        n->max = glm::max(n->l->getMax(), n->r->getMax());
        delete n->c;
        n->c = new AABB(min, max, createMeshes);
        n = n->parent;
    }
}

void BVHNode::Draw(Shader* s) {
    if (!isLeaf()) {
        c->Draw(s);
        l->Draw(s);
        r->Draw(s);
    }
}
