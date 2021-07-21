#ifndef TRIANGLE_CONSTRAINT_H
#define TRIANGLE_CONSTRAINT_H

#include "Common.h"
class RigidBody;
class CollisionPoint;

class Constraint {
public:
    RigidBody* rb1, * rb2;

    Eigen::MatrixXf Jacobian, invM;

    Eigen::VectorXf total_lambda;

    Constraint(RigidBody* rb1, RigidBody* rb2);
    virtual void solve(CollisionPoint& p, float dt) = 0;
    virtual void buildJacobian(CollisionPoint& p) = 0;

    virtual ~Constraint();
};

class RestingConstraint : public Constraint {
public:

    

    RestingConstraint(RigidBody* rb1, RigidBody* rb2);
    ~RestingConstraint();
  
    void buildJacobian(CollisionPoint& p) override;
    void solve(CollisionPoint& p, float dt) override;
    
    
};

class DistanceConstraint : public Constraint {
public:
    float minD, maxD;
    
    DistanceConstraint(RigidBody* rb1, RigidBody* rb2, float minDistance, float maxDistance);
    
    void solve(CollisionPoint& p, float dt) override;
    
    void check(float dt);
};

#endif //TRIANGLE_CONSTRAINT_H
