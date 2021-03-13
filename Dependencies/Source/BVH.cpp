
#include "BVH.h"
#include "Mesh.h"
#include "BVH.h"

BVH::BVH(std::vector<Mesh*> meshes) {

}

BVH::~BVH() {

}

BVHNode::BVHNode(std::vector<Mesh*> meshes) {

}

BVHNode::~BVHNode() {
    for (auto& c : children)
        delete c;
}
