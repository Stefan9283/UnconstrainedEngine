#include "Constraint.h"
#include "PhysicsWorld.h"

void Constraint::buildJacobian(CollisionPoint &p) {
    Jacobian.resize(12, 12);
    Jacobian.setZero();
    int i = 0;

    if (linearX.type == DISTANCE) {
        if (p.normal.x) {
            Jacobian(i, 0) = - p.normal.x;
            Jacobian(i, 6) = p.normal.x;
            i++;
        }
    }
    
    if (linearY.type == DISTANCE) {
        if (p.normal.y) {
            Jacobian(i, 1) = - p.normal.y;
            Jacobian(i, 7) = p.normal.y;
            i++;
        }
    } 
    
    if (linearZ.type == DISTANCE) {
        if (p.normal.z) {
            Jacobian(i, 2) = - p.normal.z;
            Jacobian(i, 8) = p.normal.z;
            i++;
        }
    }
    
    // TODO for angular velocity


    //
    Jacobian.conservativeResize(i, 12);
}

void Constraint::solve(CollisionPoint& p, float dt) {
    Eigen::VectorXd velocity(12);
    velocity.setZero();

    for (auto i = 0; i < 3; i++) {
        velocity(i) = rb1->velocity[i];
        velocity(i + 3) = rb1->angularVel[i];
        velocity(i + 6) = rb2->velocity[i];
        velocity(i + 9) = rb2->angularVel[i];
    }

    buildJacobian(p);


    Eigen::MatrixXd invDen = - (Jacobian * invM * Jacobian.transpose()).inverse() * Jacobian;
    Eigen::MatrixXd P = invM * Jacobian.transpose();
    
    Eigen::VectorXd lambda = invDen * velocity;
    Eigen::VectorXd dv = P * lambda;

    velocity = velocity + dv;


    for (int j = 0; j < 3; ++j) {
        rb1->velocity[j] = velocity[j];
        rb1->angularVel[j] = velocity[j + 3];
        rb2->velocity[j] = velocity[j + 6];
        rb2->angularVel[j] = velocity[j + 9];
    }
}

Constraint::Constraint(RigidBody* rb1, RigidBody* rb2) {
    this->rb1 = rb1;
    this->rb2 = rb2;
    
    invM.resize(12, 12);
    invM.setZero();
    float invM1, invM2;
    invM1 = rb1->movable ? 1 / rb1->mass : 0;
    invM2 = rb2->movable ? 1 / rb2->mass : 0;

    for (auto i = 0; i < 3; i++) {
        invM(i, i) = invM1;
        invM(6 + i, 6 + i) = invM2;
    }

    Jacobian.resize(12, 12);
    Jacobian.setZero();
}
