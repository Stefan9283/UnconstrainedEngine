//
// Created by Stefan on 16-Mar-21.
//

#include "CollisionAlgos.h"
#include "BoundingVolumes.h"

namespace collisionAlgos {
    CollisionPoint checkCollision(BoundingSphere* bv1, BoundingSphere* bv2) { return {}; }
    CollisionPoint checkCollision(AABB* bv1, AABB* bv2) { return {}; }
    CollisionPoint checkCollision(Triangle* bv1, Triangle* bv2) { return {}; }
    CollisionPoint checkCollision(TriangleMesh* bv1, TriangleMesh* bv2) { return {}; }
    CollisionPoint checkCollision(Capsule* bv1, Capsule* bv2) { return {}; }
    CollisionPoint checkCollision(Ray* bv1, Ray* bv2) { return {}; }

    CollisionPoint checkCollision(AABB* bv1, BoundingSphere* bv2) { return {}; }
    CollisionPoint checkCollision(AABB* bv1, Triangle* bv2) { return {}; }
    CollisionPoint checkCollision(AABB* bv1, TriangleMesh* bv2) { return {}; }
    CollisionPoint checkCollision(AABB* bv1, Capsule* bv2) { return {}; }
    CollisionPoint checkCollision(AABB* bv1, Ray* bv2) { return {}; }

    CollisionPoint checkCollision(BoundingSphere* bv1, Triangle* bv2) { return {}; }
    CollisionPoint checkCollision(BoundingSphere* bv1, TriangleMesh* bv2) { return {}; }
    CollisionPoint checkCollision(BoundingSphere* bv1, Capsule* bv2) { return {}; }
    CollisionPoint checkCollision(BoundingSphere* bv1, Ray* bv2) { return {}; }

    CollisionPoint checkCollision(Triangle* bv1, TriangleMesh* bv2) { return {}; }
    CollisionPoint checkCollision(Triangle* bv1, Capsule* bv2) { return {}; }
    CollisionPoint checkCollision(Triangle* bv1, Ray* bv2) { return {}; }

    CollisionPoint checkCollision(TriangleMesh* bv1, Capsule* bv2) { return {}; }
    CollisionPoint checkCollision(TriangleMesh* bv1, Ray* bv2) { return {}; }

    CollisionPoint checkCollision(Capsule* bv1, Ray* bv2) { return {}; }
}
