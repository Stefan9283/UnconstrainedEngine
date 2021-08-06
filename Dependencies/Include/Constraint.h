#ifndef TRIANGLE_CONSTRAINT_H
#define TRIANGLE_CONSTRAINT_H

#include "Common.h"
class RigidBody;
class CollisionPoint;


enum class constraintType {
    resting,
    ballsocket,
    distance,
    slider,
    generic,
    dontKnow
};

class Constraint {
public:
    RigidBody *first = nullptr, 
              *second = nullptr;
    constraintType type = constraintType::dontKnow;
    Eigen::MatrixXf Jacobian, invM;
    Eigen::VectorXf total_lambda;
    bool render = false;
    virtual void solve(float dt) {};
    virtual void gui(int index) {};
    virtual void updateMassInverse() {};
    virtual void buildJacobian() {};
    virtual void buildTrJacobian() {};
    virtual void buildRotJacobian() {};
    virtual void buildJacobian(CollisionPoint& p) {};
    virtual ~Constraint() {}
    virtual void Draw(Shader* s) {}
};

class RestingConstraint : public Constraint {
public:
    RestingConstraint(RigidBody* rb1, RigidBody* rb2);
    ~RestingConstraint();

    void solve(CollisionPoint& p, float dt);
    void buildJacobian(CollisionPoint& p);
    void updateMassInverse();
};

class BallSocketConstraint : public Constraint {
public:
    glm::vec3 fstAnchor, sndAnchor;

    Mesh* body = nullptr;

    BallSocketConstraint(RigidBody *fst, RigidBody *snd);
    ~BallSocketConstraint();
    void gui(int index) override;

    bool check();
    void solve(float dt) override;
    void updateMassInverse() override;
    void buildJacobian();
    void Draw(Shader* s) override;
};

class SliderConstraint : public Constraint {
public:
    glm::vec3 fstAnchor, sndAnchor,
            directionAxis;
    float minDist = 0, maxDist = 10;
    Mesh* body = nullptr;


    SliderConstraint(RigidBody *fst, RigidBody *snd);
    ~SliderConstraint();
    void gui(int index) override;

    void buildTrJacobian() override;
    void buildRotJacobian() override;

    bool check();
    void solve(float dt) override;
    void updateMassInverse() override;
    void Draw(Shader* s) override;

    std::vector<glm::vec3> getOrthogonalVectors();
};

class DistanceConstraint : public Constraint {
public:
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
    bool breakable; float impulseTreshold;
    glm::vec3 fstAnchor, sndAnchor;

    GenericConstraint(RigidBody* fst, RigidBody* snd);
    void setBreakable(float impulseTreshold);
    void solve(float dt) override;
    bool check();
    void gui(int index) override;
};

#endif //TRIANGLE_CONSTRAINT_H
