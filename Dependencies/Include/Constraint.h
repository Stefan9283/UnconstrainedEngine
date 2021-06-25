//
// Created by Stefan on 23-Jun-21.
//

#ifndef TRIANGLE_CONSTRAINT_H
#define TRIANGLE_CONSTRAINT_H

#include "Common.h"
class RigidBody;
class CollisionPoint;

typedef char constraint_type;

#define FREE '\0'
#define DISTANCE 'd'
#define VELOCITY 'v'
#define FRICTION 'f'


struct limit {
    constraint_type type;
    float force;
    // int cmin, cmax;
};

class Constraint {
public:
    limit
            linearX,
            linearY,
            linearZ,
            angularX,
            angularY,
            angularZ;
    RigidBody *rb1, *rb2; // TODO poate nu e nevoie

    Eigen::MatrixXd Jacobian, invM;

    Constraint(RigidBody* rb1, RigidBody* rb2);
    void buildJacobian(CollisionPoint& p); // TODO
    void solve(CollisionPoint& p, float dt); // TODO

};


#endif //TRIANGLE_CONSTRAINT_H
