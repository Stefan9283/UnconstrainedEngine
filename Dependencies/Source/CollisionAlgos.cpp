//
// Created by Stefan on 16-Mar-21.
//

#include "CollisionAlgos.h"
#include "Colliders.h"
#include "PhysicsWorld.h"

struct Solution {
    glm::vec3 point;
};

#pragma region Functii Ovidiu


float getEuclidianDistance(glm::vec3 p1, glm::vec3 p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
}

/*
float getEuclidianDistance2(glm::vec3 p1, glm::vec3 p2) {
    return glm::length(p1 - p2);
}*/

float getTriangleArea(float edge1, float edge2, float edge3) {
    float p = (edge1 + edge2 + edge3) / 2;
    return sqrt(p * (p - edge1) * (p - edge2) * (p - edge3));
}

Solution xIsFixed(glm::vec3 A, glm::vec3 B, float x) {
    Solution solution{};

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = x;
    solution.point.y = (x - x0) * (y1 - y0) / (x1 - x0) + y0;
    solution.point.z = (x - x0) * (z1 - z0) / (x1 - x0) + z0;

    return solution;
}
Solution yIsFixed(glm::vec3 A, glm::vec3 B, float y) {
    Solution solution{};

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = (y - y0) * (x1 - x0) / (y1 - y0) + x0;
    solution.point.y = y;
    solution.point.z = (y - y0) * (z1 - z0) / (y1 - y0) + z0;

    return solution;
}
Solution zIsFixed(glm::vec3 A, glm::vec3 B, float z) {
    Solution solution{};

    float x0 = A.x, y0 = A.y, z0 = A.z;
    float x1 = B.x, y1 = B.y, z1 = B.z;

    solution.point.x = (z - z0) * (x1 - x0) / (z1 - z0) + x0;
    solution.point.y = (z - z0) * (y1 - y0) / (z1 - z0) + y0;
    solution.point.z = z;

    return solution;
}

Solution getIntersectionPoint(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 D) {
    Solution solution{};

    float c1, c2;

    // Pick an axis which both lines are not parallel to
    if (A.x != B.x && C.x != D.x) {
        c1 = (B.y - A.y) / (B.x - A.x);
        c2 = (A.y * B.x - A.x * B.y) / (B.x - A.x);

        float a = (C.x * D.y + c2 * D.x - c2 * C.x - C.y * D.x);
        float b = (D.y - C.y - c1 * D.x + c1 * C.x);

        if (!b) {
            if (a)
                solution.point.x = solution.point.y = solution.point.z = INF;
            else
                solution.point.x = solution.point.y = solution.point.z = -INF;

            return solution;
        }

        solution = xIsFixed(A, B, a / b);
    } else if (A.y != B.y && C.y != D.y) {
        c1 = (B.x - A.x) / (B.y - A.y);
        c2 = (A.x * B.y - A.y * B.x) / (B.y - A.y);

        float a = (C.y * D.x + c2 * D.y - c2 * C.y - C.x * D.y);
        float b = (D.x - C.x - c1 * D.y + c1 * C.y);

        if (!b) {
            if (a)
                solution.point.x = solution.point.y = solution.point.z = INF;
            else
                solution.point.x = solution.point.y = solution.point.z = -INF;

            return solution;
        }

        solution = yIsFixed(A, B, a / b);
    } else {
        c1 = (B.x - A.x) / (B.z - A.z);
        c2 = (A.x * B.z - A.z * B.x) / (B.z - A.z);

        float a = (C.z * D.x + c2 * D.z - c2 * C.z - C.x * D.z);
        float b = (D.x - C.x - c1 * D.z + c1 * C.z);

        if (!b) {
            if (a)
                solution.point.x = solution.point.y = solution.point.z = INF;
            else
                solution.point.x = solution.point.y = solution.point.z = -INF;

            return solution;
        }

        solution = zIsFixed(A, B, a / b);
    }

    return solution;
}
#pragma endregion
#pragma region Functii Stefan
glm::vec3 getTransformedVert(glm::mat4 tr, glm::vec3 v) { // copy of getTransformedVertex from Colliders.cpp
    return glm::vec3(tr * glm::vec4(v, 1.0f));
}
glm::vec3 ClosestPointOnLineSegment(glm::vec3 A, glm::vec3 B, glm::vec3 Point) {
   
    glm::vec3 AB = B - A;
    float t = glm::dot(Point - A, AB) / glm::dot(AB, AB);
    //std::cout << "\nClosest point\n";
    //std::cout << glm::to_string(Point);
    //std::cout << glm::to_string(A);
    //std::cout << glm::to_string(B);
    //std::cout <<  glm::to_string(AB);
    //std::cout << t << " ";
    //std::cout << (float)glm::min(glm::max(t, 0.0f), 1.0f) << "\n";
    // std::cout << "Closest point end\n";
    return A + (float)glm::min(glm::max(t, 0.0f), 1.0f) * AB; // saturate(t) can be written as: min((max(t, 0), 1)
}
glm::vec3 getMidPoint(glm::vec3 A, glm::vec3 B, glm::vec3 C) {
    glm::vec3 result = A, min, max;

    min = glm::min(A, glm::min(B, C));
    max = glm::max(A, glm::max(B, C));

    if (A.x != B.x && B.x != C.x) {
        if (A.x != min.x && A.x != max.x) result.x = A.x;
        if (B.x != min.x && B.x != max.x) result.x = B.x;
        if (C.x != min.x && C.x != max.x) result.x = C.x;
    }
    if (A.y != B.y && B.y != C.y) {
        if (A.y != min.y && A.y != max.y) result.y = A.y;
        if (B.y != min.y && B.y != max.y) result.y = B.y;
        if (C.y != min.y && C.y != max.y) result.y = C.y;
    }
    if (A.z != B.z && B.z != C.z) {
        if (A.z != min.z && A.z != max.z) result.z = A.z;
        if (B.z != min.z && B.z != max.z) result.z = B.z;
        if (C.z != min.z && C.z != max.z) result.z = C.z;
    }
    return result;
}
#pragma endregion


namespace collisionAlgos {
    CollisionPoint checkCollision(Sphere* bv1, Sphere* bv2) {
        glm::vec3 center2centerVec = bv1->getCurrentPosition() - bv2->getCurrentPosition();
        if(glm::length(center2centerVec) <= bv1->radius + bv2->radius) {
            center2centerVec = glm::normalize(center2centerVec);
            return CollisionPoint(bv1->getCurrentPosition() - center2centerVec * bv1->radius, bv2->getCurrentPosition() + center2centerVec * bv2->radius);
        }
        return {};
    }
    CollisionPoint checkCollision(AABB* bv1, AABB* bv2) {
        glm::vec3 newMin1 = bv1->getMin(), newMax1 = bv1->getMax();
        glm::vec3 newMin2 = bv2->getMin(), newMax2 = bv2->getMax();

        //std::cout << glm::to_string(bv1->getMin()) << glm::to_string(bv1->getMax()) << glm::to_string(bv1->getOffset()) << "\n";
        //std::cout << glm::to_string(bv2->getMin()) << glm::to_string(bv2->getMax()) << glm::to_string(bv2->getOffset()) << "\n";

        if (newMin1.x > newMax2.x || newMax1.x < newMin2.x) {
            //std::cout << newMin1.x << " " << newMax2.x << " " << newMax1.x << " " << newMin2.x << " ";
            //std::cout << "x\n";
            return {};
        }

        if (newMin1.y > newMax2.y || newMax1.y < newMin2.y) {
            //std::cout << newMin1.y << " " << newMax2.y << " " << newMax1.y << " " << newMin2.y << " ";
            //std::cout << "y\n";
            return {};
        }

        if (newMin1.z > newMax2.z || newMax1.z < newMin2.z) {
            //std::cout << newMin1.z << " " << newMax2.z << " " << newMax1.z << " " << newMin2.z << " ";
            //std::cout << "z\n";
            return {};
        }


        glm::vec3 v0 = glm::max(newMin1, glm::min(newMin2, newMax1));
        glm::vec3 v1 = glm::max(newMin1, glm::min(newMax2, newMax1));

//        glm::vec3 v0 = glm::vec3(   std::max(newMin1.x, std::min(newMin2.x, newMax1.x)),
//                                    std::max(newMin1.y, std::min(newMin2.y, newMax1.y)),
//                                    std::max(newMin1.z, std::min(newMin2.z, newMax1.z)));
//
//        glm::vec3 v1 = glm::vec3(   std::max(newMin1.x, std::min(newMax2.x, newMax1.x)),
//                                    std::max(newMin1.y, std::min(newMax2.y, newMax1.y)),
//                                    std::max(newMin1.z, std::min(newMax2.z, newMax1.z)));

        glm::vec3 mean = (v0 + v1) / 2.0f;

        //std::cout << "Coliziune\n";
        CollisionPoint p(bv1->closestPoint(mean), bv2->closestPoint(mean));
        //std::cout << p.toString() << "\n";
        return p;
    }
    CollisionPoint checkCollision(Triangle* bv1, Triangle* bv2) {
        bool twoway0 = bv1->twoway;
        bv1->twoway = true;

        Ray r0(bv2->vertices[0], glm::normalize(- bv2->vertices[0] + bv2->vertices[1]), glm::length(bv2->vertices[0] - bv2->vertices[1]));
        Ray r1(bv2->vertices[1], glm::normalize(- bv2->vertices[1] + bv2->vertices[2]), glm::length(bv2->vertices[1] - bv2->vertices[2]));
        Ray r2(bv2->vertices[0], glm::normalize(- bv2->vertices[0] + bv2->vertices[2]), glm::length(bv2->vertices[0] - bv2->vertices[2]));

        return {}; // bool result = r0.checkCollision(this) || r1.checkCollision(this) || r2.checkCollision(this);

        bv1->twoway = twoway0;

        return {}; // return result;
    }
    CollisionPoint checkCollision(TriangleMesh* bv1, Collider* bv2) {
        bool result = false;

        for (int i = 0; i < bv1->body->indices.size(); i += 3) {
            glm::vec3 meanNormal = bv1->body->vertices[i].Normal + bv1->body->vertices[i + 1].Normal + bv1->body->vertices[i + 2].Normal;
            glm::vec3 faceNormal = glm::normalize(glm::cross(bv1->body->vertices[i].Position - bv1->body->vertices[i + 2].Position, bv1->body->vertices[i].Position - bv1->body->vertices[i + 1].Position));
            if (glm::dot(faceNormal, meanNormal) < 0)
                faceNormal = -1.0f * faceNormal;
            Triangle t(
                    getTransformedVert(bv1->getLocalTransform(), bv1->body->vertices[i].Position),
                    getTransformedVert(bv1->getLocalTransform(), bv1->body->vertices[i + 1].Position),
                    getTransformedVert(bv1->getLocalTransform(), bv1->body->vertices[i + 2].Position),
                    glm::mat3(transpose(inverse(bv1->getLocalTransform()))) * faceNormal
            );

            // [Stefan] Ma ocup eu aici
            //result = col->checkCollision(&t);
            //if (result)
            //    return result;
        }

        return {}; //return result;
    } // TODO return CollisionPoint
    CollisionPoint checkCollision(Capsule* bv1, Capsule* col) {
        // capsule A:
        glm::vec3 a_Normal = normalize(bv1->getStart() - bv1->getEnd());
        glm::vec3 a_LineEndOffset = a_Normal * bv1->radius;
        glm::vec3 a_A = bv1->getStart() + a_LineEndOffset;
        glm::vec3 a_B = bv1->getEnd() - a_LineEndOffset;

        // capsule B:
        glm::vec3 b_Normal = normalize(col->getStart() - col->getEnd());
        glm::vec3 b_LineEndOffset = b_Normal * col->radius;
        glm::vec3 b_A = col->getStart() + b_LineEndOffset;
        glm::vec3 b_B = col->getEnd() - b_LineEndOffset;

        // vectors between line endpoints
        glm::vec3 v0 = b_A - a_A;
        glm::vec3 v1 = b_B - a_A;
        glm::vec3 v2 = b_A - a_B;
        glm::vec3 v3 = b_B - a_B;

        // squared distances:
        float d0 = glm::dot(v0, v0);
        float d1 = glm::dot(v1, v1);
        float d2 = glm::dot(v2, v2);
        float d3 = glm::dot(v3, v3);

        // select best potential endpoint on capsule A:
        glm::vec3 bestA;
        if (d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1)
        {
            bestA = a_B;
        }
        else
        {
            bestA = a_A;
        }

        // select point on capsule B line segment nearest to best potential endpoint on A capsule:
        glm::vec3 bestB = ClosestPointOnLineSegment(col->getStart(), col->getEnd(), bestA);

        // now do the same for capsule A segment:
        bestA = ClosestPointOnLineSegment(bv1->getStart(), bv1->getEnd(), bestB);

        Sphere s1(bestA, bv1->radius);
        Sphere s2(bestB, col->radius);

        return {}; // return s1.checkCollision(&s2);
    }
    CollisionPoint checkCollision(Ray* bv1, Ray* bv2) {
        glm::vec3 A = bv1->origin, B = bv1->origin + bv1->direction * bv1->length;
        glm::vec3 C = bv2->origin, D = bv2->origin + bv2->direction * bv2->length;

        Solution solution = getIntersectionPoint(A, B, C, D);

        float distance1 = glm::distance(A, solution.point);
        float distance2 = glm::distance(B, solution.point);

        float distance3 = glm::distance(C, solution.point);
        float distance4 = glm::distance(D, solution.point);

        bool res = solution.point.x != -INF &&
                   solution.point.x != INF && fabs(distance1 + distance2 - bv1->length) <= EPS && (
                            ( C.x >= solution.point.x && solution.point.x >= D.x ) || ( C.x <= solution.point.x && solution.point.x <= D.x) &&
                            ( C.y <= solution.point.y && solution.point.y <= D.y ) || ( C.y >= solution.point.y && solution.point.y >= D.y) &&
                            ( C.z <= solution.point.z && solution.point.z <= D.z ) || ( C.z >= solution.point.z && solution.point.z >= D.z)
                   );
        return {};
        /*
        return solution.point.x != -INF &&
               solution.point.x != INF &&
               fabs(distance1 + distance2 - bv1->length) <= EPS &&
               fabs(distance3 + distance4 - r->length) <= EPS;
               */
    }  // TODO return CollisionPoint

    CollisionPoint checkCollision(AABB* bv1, Sphere* bv2) {
        glm::vec3 newMin = bv1->getMin(), newMax = bv1->getMax();

        glm::vec3 O2 = bv2->getCurrentPosition();

        glm::vec3 v = glm::max(newMin, glm::min(O2, newMax));

        if (glm::distance(v, O2) <= glm::pow(bv2->radius, 2)) {
            if (O2 - v == glm::vec3(0)) {
                return {bv1->closestPoint(v), O2 - bv2->radius * glm::normalize(O2 - (newMin + newMax) / 2.0f)};
            } else {
                return {bv1->closestPoint(v), O2 - glm::normalize(O2 - v) * bv2->radius};
            }
        } else return {};
    }
    CollisionPoint checkCollision(AABB* bv1, Triangle* bv2) {
        CollisionPoint p;
        p.hasCollision = true;

        for (auto & vertex : bv2->vertices)
            if (bv1->isInside(vertex))
                return p; // return true;

        Ray r0(bv2->vertices[0], glm::normalize(-bv2->vertices[0] + bv2->vertices[1]), glm::length(bv2->vertices[0] - bv2->vertices[1]));
        Ray r1(bv2->vertices[1], glm::normalize(-bv2->vertices[1] + bv2->vertices[2]), glm::length(bv2->vertices[1] - bv2->vertices[2]));
        Ray r2(bv2->vertices[0], glm::normalize(-bv2->vertices[0] + bv2->vertices[2]), glm::length(bv2->vertices[0] - bv2->vertices[2]));

        {
            p = r0.checkCollision(bv1);
            if (p.hasCollision) return p;
            p = r1.checkCollision(bv1);
            if (p.hasCollision) return p;
            p = r2.checkCollision(bv1);
            if (p.hasCollision) return p;
        }
        return {};
    }
    CollisionPoint checkCollision(AABB* bv1, Capsule* bv2) {
        glm::vec3 bestPoint = ClosestPointOnLineSegment(bv2->getStart(), bv2->getEnd(), (bv1->getMin() + bv1->getMax())/2.0f);
        //std::cout << glm::to_string(bestPoint) << bv2->radius << "\n";
        Sphere s(bestPoint, bv2->radius);
        return s.checkCollision(bv1);
    }
    CollisionPoint checkCollision(AABB* bv1, Ray* bv2) {
        glm::vec3 newMin = bv1->getMin(), newMax = bv1->getMax();
        glm::vec3 A = bv2->origin, B = bv2->origin - bv2->direction;

        float x0 = A.x, y0 = A.y, z0 = A.z;
        float x1 = B.x, y1 = B.y, z1 = B.z;

        float x, y, z;

        CollisionPoint p;
        p.hasCollision = true;

        // Front face
        x = (newMax.z * x1 - newMax.z * x0 - x1 * z0 + x0 * z1) / (z1 - z0);
        y = (newMax.z * y1 - newMax.z * y0 - y1 * z0 + y0 * z1) / (z1 - z0);

        if (z0 != z1 && x >= newMin.x && x <= newMax.x && y >= newMin.y && y <= newMax.y) {
            float distance1 = glm::distance(A, glm::vec3(x, y, newMax.z));
            float distance2 = glm::distance(A + bv2->direction * bv2->length, glm::vec3(x, y, newMax.z));

            if (fabs(distance1 + distance2 - bv2->length) <= EPS)
                return p; // return true;
        }

        // Back face
        x = (newMin.z * x1 - newMin.z * x0 - x1 * z0 + x0 * z1) / (z1 - z0);
        y = (newMin.z * y1 - newMin.z * y0 - y1 * z0 + y0 * z1) / (z1 - z0);

        if (z0 != z1 && x >= newMin.x && x <= newMax.x && y >= newMin.y && y <= newMax.y) {
            float distance1 = glm::distance(A, glm::vec3(x, y, newMin.z));
            float distance2 = glm::distance(A + bv2->direction * bv2->length, glm::vec3(x, y, newMin.z));

            if (fabs(distance1 + distance2 - bv2->length) <= EPS)
                return p; // return true;
        }

        // Left face
        y = (newMin.x * y1 - newMin.x * y0 - y1 * x0 + y0 * x1) / (x1 - x0);
        z = (newMin.x * z1 - newMin.x * z0 - z1 * x0 + z0 * x1) / (x1 - x0);

        if (x0 != x1 && y >= newMin.y && y <= newMax.y && z >= newMin.z && z <= newMax.z) {
            float distance1 = glm::distance(A, glm::vec3(newMin.x, y, z));
            float distance2 = glm::distance(A + bv2->direction * bv2->length, glm::vec3(newMin.x, y, z));

            if (fabs(distance1 + distance2 - bv2->length) <= EPS)
                return p; // return true;
        }

        // Right face
        y = (newMax.x * y1 - newMax.x * y0 - y1 * x0 + y0 * x1) / (x1 - x0);
        z = (newMax.x * z1 - newMax.x * z0 - z1 * x0 + z0 * x1) / (x1 - x0);

        if (x0 != x1 && y >= newMin.y && y <= newMax.y && z >= newMin.z && z <= newMax.z) {
            float distance1 = glm::distance(A, glm::vec3(newMax.x, y, z));
            float distance2 = glm::distance(A + bv2->direction * bv2->length, glm::vec3(newMax.x, y, z));

            if (fabs(distance1 + distance2 - bv2->length) <= EPS)
                return p; // return true;
        }

        // Bottom face
        x = (newMin.y * x1 - newMin.y * x0 - x1 * y0 + x0 * y1) / (y1 - y0);
        z = (newMin.y * z1 - newMin.y * z0 - z1 * y0 + z0 * y1) / (y1 - y0);

        if (y0 != y1 && x >= newMin.x && x <= newMax.x && z >= newMin.z && z <= newMax.z) {
            float distance1 = glm::distance(A, glm::vec3(x, newMin.y, z));
            float distance2 = glm::distance(A + bv2->direction * bv2->length, glm::vec3(x, newMin.y, z));

            if (fabs(distance1 + distance2 - bv2->length) <= EPS)
                return p; // return true;
        }

        // Up face
        x = (newMax.y * x1 - newMax.y * x0 - x1 * y0 + x0 * y1) / (y1 - y0);
        z = (newMax.y * z1 - newMax.y * z0 - z1 * y0 + z0 * y1) / (y1 - y0);

        if (y0 != y1 && x >= newMin.x && x <= newMax.x && z >= newMin.z && z <= newMax.z) {
            float distance1 = glm::distance(A, glm::vec3(x, newMax.y, z));
            float distance2 = glm::distance(A + bv2->direction * bv2->length, glm::vec3(x, newMax.y, z));

            if (fabs(distance1 + distance2 - bv2->length) <= EPS)
                return p; //  true;
        }

        return {};
    }

    CollisionPoint checkCollision(Sphere* bv1, Triangle* bv2) {
        for (auto & vertex : bv2->vertices)
            if (bv1->isInside(vertex)) return {}; // return true;
        Ray r0(bv2->vertices[0], glm::normalize(-bv2->vertices[0] + bv2->vertices[1]), glm::length(bv2->vertices[0] - bv2->vertices[1]));
        Ray r1(bv2->vertices[1], glm::normalize(-bv2->vertices[1] + bv2->vertices[2]), glm::length(bv2->vertices[1] - bv2->vertices[2]));
        Ray r2(bv2->vertices[0], glm::normalize(-bv2->vertices[0] + bv2->vertices[2]), glm::length(bv2->vertices[0] - bv2->vertices[2]));
        return {}; // return r0.checkCollision(bv) || r1.checkCollision(bv) || r2.checkCollision(bv);
    }// TODO return CollisionPoint
    CollisionPoint checkCollision(Sphere* bv1, Capsule* bv2) {
        glm::vec3 bestPoint = ClosestPointOnLineSegment(bv2->getStart(), bv2->getEnd(), bv1->getCurrentPosition());
        Sphere s(bestPoint, bv2->radius);
        return s.checkCollision(bv1);
    }
    CollisionPoint checkCollision(Sphere* bv1, Ray* bv2) {
        glm::vec3 A = bv2->origin, B = bv2->origin + bv2->direction * bv2->length;

        float x0 = A.x, y0 = A.y, z0 = A.z;
        float x1 = B.x, y1 = B.y, z1 = B.z;

        float c1, c2, c3, c4;

        glm::vec3 O1 = bv1->getCurrentPosition();

        // Pick an axis which the line is not parallel to
        if (A.x != B.x) {
            c1 = (y1 - y0) / (x1 - x0);
            c2 = (x1 * y0 - x0 * y1 + x0 * O1.y - x1 * O1.y) / (x1 - x0);

            c3 = (z1 - z0) / (x1 - x0);
            c4 = (x1 * z0 - x0 * z1 + x0 * O1.z - x1 * O1.z) / (x1 - x0);
        } else if (A.y != B.y) {
            c1 = (x1 - x0) / (y1 - y0);
            c2 = (y1 * x0 - y0 * x1 + y0 * O1.x - y1 * O1.x) / (y1 - y0);

            c3 = (z1 - z0) / (y1 - y0);
            c4 = (y1 * z0 - y0 * z1 + y0 * O1.z - y1 * O1.z) / (y1 - y0);
        } else {
            c1 = (x1 - x0) / (z1 - z0);
            c2 = (z1 * x0 - z0 * x1 + z0 * O1.x - z1 * O1.x) / (z1 - z0);

            c3 = (y1 - y0) / (z1 - z0);
            c4 = (z1 * y0 - z0 * y1 + z0 * O1.y - z1 * O1.y) / (z1 - z0);
        }

        // The points of intersection are the solution of the quadratic equation
        float pinnedSphereOffset;

        if (x0 != x1)
            pinnedSphereOffset = O1.x;
        else if (y0 != y1)
            pinnedSphereOffset = O1.y;
        else
            pinnedSphereOffset = O1.z;

        float a = 1 + c1 * c1 + c3 * c3;
        float b = -2 * (pinnedSphereOffset - c1 * c2 - c3 * c4);
        float c = pinnedSphereOffset * pinnedSphereOffset + c2 * c2 + c4 * c4 - bv1->radius * bv1->radius;

        float delta = b * b - 4 * a * c;

        if (delta < 0) {
            return {};
        }
        if (delta == 0) {
            Solution solution{};

            if (A.x != B.x)
                solution = xIsFixed(A, B, -b / (2 * a));
            else if (A.y != B.y)
                solution = yIsFixed(A, B, -b / (2 * a));
            else
                solution = zIsFixed(A, B, -b / (2 * a));

            // Check the distance between the ray's origin and the unique point
            float distance1 = glm::distance(A, solution.point);
            float distance2 = glm::distance(A + bv2->direction * bv2->length, solution.point);

            if (fabs(distance1 + distance2 - bv2->length) <= EPS) {
                std::cout << "brah0\n";
                return CollisionPoint(solution.point, solution.point); // return true;
            }
        } else {
            Solution solution1{};

            if (x0 != x1)
                solution1 = xIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));
            else if (y0 != y1)
                solution1 = yIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));
            else
                solution1 = zIsFixed(A, B, (-b - sqrt(delta)) / (2 * a));

            // Check the distance between the ray's origin and the first point
            float distance11 = glm::distance(A, solution1.point);
            float distance12 = glm::distance(A + bv2->direction * bv2->length, solution1.point);

            /*
            if (fabs(distance11 + distance12 - bv2->length) <= EPS) {
                return CollisionPoint(solution1.point, solution1.point);; //return true;
            }
            */

            Solution solution2{};

            if (x0 != x1)
                solution2 = xIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
            else if (y0 != y1)
                solution2 = yIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));
            else
                solution2 = zIsFixed(A, B, (-b + sqrt(delta)) / (2 * a));

            float distance21, distance22;
            // Check the distance between the ray's origin and the second point
            distance21 = glm::distance(A, solution2.point);
            distance22 = glm::distance(A + bv2->direction * bv2->length, solution2.point);

            /*
            if (fabs(distance21 + distance22 - bv2->length) <= EPS) {
                std::cout << "brah1\n";
                return CollisionPoint(solution2.point, solution2.point);; // return true;
            }
            */


            if ((fabs(distance11 + distance12 - bv2->length) > WEAK_EPS) && (fabs(distance21 + distance22 - bv2->length) > WEAK_EPS))
            {
                //std::cout << distance11 << " " << distance12 << " " << distance21 << " " << distance22 << "\n";
                //std::cout << distance11 + distance12 - (float)bv2->length << " " << distance21 + distance22 - (float)bv2->length << "\n";
                //std::cout << glm::to_string(solution1.point) << glm::to_string(solution2.point) << "2\n";
                //std::cout << glm::to_string(origin) << glm::to_string(direction) << "2\n";
                return {};
            }

            glm::vec3 start, end;

            if (bv1->isInside(A))
                start = A;
            else if (distance11 < distance21)
                start = solution1.point;
            else start = solution2.point;

            if (bv1->isInside(B))
                end = B;
            else if (distance12 < distance22)
                end = solution1.point;
            else end = solution2.point;

            return CollisionPoint(start, end);



        }
        return {};
    }

    CollisionPoint checkCollision(Triangle* bv1, Capsule* bv2) {
        // for triangle inside a capsule
        for (auto & vertex : bv1->vertices) {
            glm::vec3 mid = getMidPoint(bv2->getStart(), bv2->getEnd(), vertex);
            if(glm::length(mid - vertex) < bv2->radius)
                return {}; // return true;
        }

        bool twoway = bv1->twoway;
        bv1->twoway = true;
        Ray r0(bv1->vertices[0], glm::normalize(- bv1->vertices[0] + bv1->vertices[1]), glm::length(bv1->vertices[0] - bv1->vertices[1]));
        Ray r1(bv1->vertices[1], glm::normalize(- bv1->vertices[1] + bv1->vertices[2]), glm::length(bv1->vertices[1] - bv1->vertices[2]));
        Ray r2(bv1->vertices[0], glm::normalize(- bv1->vertices[0] + bv1->vertices[2]), glm::length(bv1->vertices[0] - bv1->vertices[2]));
        return {}; //bool result = r0.checkCollision(this) || r1.checkCollision(this) || r2.checkCollision(this);
        bv1->twoway = twoway;
        return {}; //return result;
    } // TODO return CollisionPoint
    CollisionPoint checkCollision(Triangle* bv1, Ray* bv2) {
        glm::vec3 A = bv2->origin, B = bv2->origin + bv2->length * bv2->direction;

        if (!bv1->twoway && glm::dot(bv1->norm, bv2->direction) > 0)
            return {};

        // The intersection is an infinite number on points
        if (glm::dot(bv1->norm, B - A) == 0) {
            if (bv1->isInside(A) || bv1->isInside(B))
                return {}; // return true;
        }

        float x0 = A.x, y0 = A.y, z0 = A.z;
        float x1 = B.x, y1 = B.y, z1 = B.z;

        float a = bv1->norm.x, b = bv1->norm.y, c = bv1->norm.z;
        float d = glm::dot(bv1->norm, bv1->vertices[0]);

        float c1, c2, c3, c4;
        Solution solution{};

        Ray r0(bv1->vertices[0], glm::normalize(- bv1->vertices[0] + bv1->vertices[1]), glm::length(bv1->vertices[0] - bv1->vertices[1]));
        Ray r1(bv1->vertices[1], glm::normalize(- bv1->vertices[1] + bv1->vertices[2]), glm::length(bv1->vertices[1] - bv1->vertices[2]));
        Ray r2(bv1->vertices[0], glm::normalize(- bv1->vertices[0] + bv1->vertices[2]), glm::length(bv1->vertices[0] - bv1->vertices[2]));

        // Pick an axis which the line is not parallel to
        if (x0 != x1) {
            c1 = b * (y1 - y0)/(x1 - x0);
            c2 = b * (x1 * y0 - x0 * y1)/(x1 - x0);

            c3 = c * (z1 - z0)/(x1 - x0);
            c4 = c * (x1 * z0 - x0 * z1)/(x1 - x0);

            if (a + (c1 + c3) != 0) {
                // No intersection at all
                if (d - (c2 - c4))
                    return {};

                return {}; // return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
            }

            solution = xIsFixed(A, B, (d - c2 - c4) / (a + c1 + c3));
        } else if (y0 != y1) {
            c1 = a * (x1 - x0) / (y1 - y0);
            c2 = a * (y1 * x0 - y0 * x1) / (y1 - y0);

            c3 = c * (z1 - z0) / (y1 - y0);
            c4 = c * (y1 * z0 - y0 * z1) / (y1 - y0);

            if (b + c1 + c3 != 0) {
                // No intersection at all
                if (d - c2 - c4)
                    return {};

                return {}; // return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
            }

            solution = yIsFixed(A, B, (d - c2 - c4) / (b + c1 + c3));
        } else {
            c1 = a * (x1 - x0) / (z1 - z0);
            c2 = a * (z1 * x0 - z0 * x1) / (z1 - z0);

            c3 = c * (y1 - y0) / (z1 - z0);
            c4 = c * (z1 * y0 - z0 * y1) / (z1 - z0);

            if (c + c1 + c3 != 0) {
                // No intersection at all
                if (d - c2 - c4 != 0)
                    return {};

                return {}; //return checkCollision(&r0) || checkCollision(&r1) || checkCollision(&r2);
            }

            solution = zIsFixed(A, B, (d - c2 - c4) / (c + c1 + c3));
        }

        float distance1 = glm::distance(A, solution.point);
        float distance2 = glm::distance(B, solution.point);

        if (distance1 > bv2->length || distance2 > bv2->length)
            return {};

        return {}; // return bv1->isInside(solution.point);
    } // TODO return CollisionPoint

    CollisionPoint checkCollision(Capsule* bv1, Ray* bv2) {
        // capsule:
        glm::vec3 a_Normal = normalize(bv1->getStart() - bv1->getEnd());
        glm::vec3 a_LineEndOffset = a_Normal * bv1->radius;
        glm::vec3 a_A = bv1->getStart() + a_LineEndOffset;
        glm::vec3 a_B = bv1->getEnd() - a_LineEndOffset;

        // ray:
        glm::vec3 b_A = bv2->origin;
        glm::vec3 b_B = bv2->direction * bv2->length + bv2->origin;

        // vectors between line endpoints
        glm::vec3 v0 = b_A - a_A;
        glm::vec3 v1 = b_B - a_A;
        glm::vec3 v2 = b_A - a_B;
        glm::vec3 v3 = b_B - a_B;

        // squared distances:
        float d0 = glm::dot(v0, v0);
        float d1 = glm::dot(v1, v1);
        float d2 = glm::dot(v2, v2);
        float d3 = glm::dot(v3, v3);

        // select best potential endpoint on capsule A:
        glm::vec3 bestA;
        if (d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1)
        {
            bestA = a_B;
        }
        else
        {
            bestA = a_A;
        }

        // select point on line segment nearest to best potential endpoint on capsule:
        glm::vec3 bestB = ClosestPointOnLineSegment(b_A, b_B, bestA);

        // now do the same for capsule A segment:
        bestA = ClosestPointOnLineSegment(bv1->getStart(), bv1->getEnd(), bestB);

        Sphere s(bestA, bv1->radius);
        return s.checkCollision(bv2);
    }
}
