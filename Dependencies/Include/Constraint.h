//
// Created by Stefan on 23-Jun-21.
//

#ifndef TRIANGLE_CONSTRAINT_H
#define TRIANGLE_CONSTRAINT_H

#include "Common.h"
class RigidBody;
class CollisionPoint;

typedef char constraint_type;

struct limit {
    float force;
    int cmin, cmax; // for capping the lambda values
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
    RigidBody *rb1, *rb2;

    Eigen::MatrixXf Jacobian, invM;

    Eigen::VectorXf cached_lambda;

    Constraint(RigidBody* rb1, RigidBody* rb2);
    virtual void buildJacobian(CollisionPoint& p) = 0;
    virtual void solve(CollisionPoint& p, float dt) = 0;
    virtual Eigen::VectorXf getCachedLambda(CollisionPoint& p) = 0;
    virtual void setCachedLambda(CollisionPoint& p, Eigen::VectorXf& l) = 0;
    virtual ~Constraint();
};

class DistanceConstraint : public Constraint {
public:
    float minD, maxD;
    DistanceConstraint(RigidBody* rb1, RigidBody* rb2,
            float minDistance, float maxDistance);
    void buildJacobian(CollisionPoint& p) override; // TODO
    void solve(CollisionPoint& p, float dt) override; // TODO
    Eigen::VectorXf getCachedLambda(CollisionPoint& p) override;
    void setCachedLambda(CollisionPoint& p, Eigen::VectorXf& l) override;
    ~DistanceConstraint() override;
};

class RestingConstraint : public DistanceConstraint {
public:
    RestingConstraint(RigidBody* rb1, RigidBody* rb2);
};




#endif //TRIANGLE_CONSTRAINT_H
