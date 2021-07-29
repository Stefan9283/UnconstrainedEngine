#ifndef TRIANGLE_CONSTRAINT_H
#define TRIANGLE_CONSTRAINT_H

#include "Common.h"
class RigidBody;
class CollisionPoint;

class Constraint {
public:
    Eigen::MatrixXf Jacobian, invM;
    Eigen::VectorXf total_lambda;

    Constraint();
    virtual void solve(float dt) {};
    virtual void gui(int index) {};
    virtual void updateMassInverse() {};
    virtual void buildJacobian() {};
    virtual void buildJacobian(CollisionPoint& p) {};
    virtual ~Constraint();
};

class RestingConstraint : public Constraint {
public:
    RigidBody *rb1, *rb2;

    RestingConstraint(RigidBody* rb1, RigidBody* rb2);
    ~RestingConstraint();

    void solve(CollisionPoint& p, float dt);
    void buildJacobian(CollisionPoint& p);
    void updateMassInverse();
};

class BallSocketConstraint : public Constraint {
public:
    RigidBody *first, *second;
    glm::vec3 fstAnchor, sndAnchor;

    bool render = false;
    Mesh* body = nullptr;

    BallSocketConstraint(RigidBody *fst, RigidBody *snd);
    ~BallSocketConstraint();
    void gui(int index) override;

    bool check();
    void solve(float dt) override;
    void updateMassInverse() override;
    void buildJacobian();
};

class DistanceConstraint : public Constraint {
public:
    RigidBody *rb1, *rb2;
    float minD, maxD;
    
    DistanceConstraint(RigidBody* rb1, RigidBody* rb2,
                float minDistance, float maxDistance);
    void gui(int index) override;

    bool check();
    void solve(float dt) override;
};




class limitConstraint {
public:
    bool enabled;
    float offset;
    float lower, upper;
    void toggle(float lower = 0, float upper = 0);
    bool check(float dist);
};

class GenericConstraint : public Constraint {
public:
    limitConstraint angular[3]{}, linear[3]{};
    RigidBody *first, *second;
    bool breakable; float impulseTreshold;
    glm::vec3 fstAnchor, sndAnchor;

    GenericConstraint(RigidBody* fst, RigidBody* snd);
    void setBreakable(float impulseTreshold);
    void solve(float dt) override;
    bool check();
    void gui(int index) override;
};

#endif //TRIANGLE_CONSTRAINT_H
