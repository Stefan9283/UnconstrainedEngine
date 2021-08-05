#ifndef TRIANGLE_COLLISIONALGOS_H
#define TRIANGLE_COLLISIONALGOS_H


class Collider;
class AABB;
class OBB;
class Sphere;
class Ray;
class TriangleMesh;
class Triangle;
class Capsule;

class CollisionPoint;

namespace collisionAlgos {
    CollisionPoint checkCollision(Sphere* col1, Sphere* col);
    CollisionPoint checkCollision(AABB* col1, AABB* col);
    CollisionPoint checkCollision(Triangle* col1, Triangle* col);
    CollisionPoint checkCollision(TriangleMesh* col1, Collider* col);
    CollisionPoint checkCollision(Capsule* col1, Capsule* col);
    CollisionPoint checkCollision(Ray* col1, Ray* col);

    CollisionPoint checkCollision(AABB* col1, Sphere* col);
    CollisionPoint checkCollision(AABB* col1, Triangle* col);
    CollisionPoint checkCollision(AABB* col1, Capsule* col);
    CollisionPoint checkCollision(AABB* col1, Ray* col);

    CollisionPoint checkCollision(OBB* bv1, AABB* bv2);
    CollisionPoint checkCollision(OBB* bv1, Sphere* bv2);
    CollisionPoint checkCollision(OBB* bv1, OBB* bv2);
    CollisionPoint checkCollision(OBB* bv1, Ray* bv2);
    CollisionPoint checkCollision(OBB* bv1, Capsule* bv2);
    CollisionPoint checkCollision(OBB* bv1, Triangle* bv2);

    CollisionPoint checkCollision(Sphere* col1, Triangle* col);
    CollisionPoint checkCollision(Sphere* col1, Capsule* col);
    CollisionPoint checkCollision(Sphere* col1, Ray* col);

    CollisionPoint checkCollision(Triangle* col1, Capsule* col);
    CollisionPoint checkCollision(Triangle* col1, Ray* col);

    CollisionPoint checkCollision(Capsule* col1, Ray* col);
}



namespace collisionAlgos {
    bool hasCollision(Sphere* bv1, Sphere* bv2);
    bool hasCollision(AABB* bv1, AABB* bv2);
    bool hasCollision(Triangle* bv1, Triangle* bv2);
    bool hasCollision(TriangleMesh* bv1, Collider* bv2);
    bool hasCollision(Capsule* bv1, Capsule* col);
    bool hasCollision(Ray* bv1, Ray* bv2);

    bool hasCollision(AABB* bv1, Sphere* bv2);
    bool hasCollision(AABB* bv1, Triangle* bv2);
    bool hasCollision(AABB* bv1, Capsule* bv2);
    bool hasCollision(AABB* bv1, Ray* bv2);
    // TODO OBB 
    bool hasCollision(OBB* bv1, AABB* bv2);
    bool hasCollision(OBB* bv1, Sphere* bv2);
    bool hasCollision(OBB* bv1, OBB* bv2);
    bool hasCollision(OBB* bv1, Ray* bv2);
    bool hasCollision(OBB* bv1, Capsule* bv2);
    bool hasCollision(OBB* bv1, Triangle* bv2);

    bool hasCollision(Sphere* bv1, Triangle* bv2);
    bool hasCollision(Sphere* bv1, Capsule* bv2);
    bool hasCollision(Sphere* bv1, Ray* bv2);

    bool hasCollision(Triangle* bv1, Capsule* bv2);
    bool hasCollision(Triangle* bv1, Ray* bv2);
    bool hasCollision(Capsule* bv1, Ray* bv2);
}
#endif //TRIANGLE_COLLISIONALGOS_H
