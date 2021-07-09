//
// Created by Stefan on 16-Mar-21.
//

#ifndef TRIANGLE_COLLISIONALGOS_H
#define TRIANGLE_COLLISIONALGOS_H


class Collider;
class AABB;
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

    CollisionPoint checkCollision(Sphere* col1, Triangle* col);
    CollisionPoint checkCollision(Sphere* col1, Capsule* col);
    CollisionPoint checkCollision(Sphere* col1, Ray* col);

    CollisionPoint checkCollision(Triangle* col1, Capsule* col);
    CollisionPoint checkCollision(Triangle* col1, Ray* col);

    CollisionPoint checkCollision(Capsule* col1, Ray* col);
}

#endif //TRIANGLE_COLLISIONALGOS_H
