#ifndef TRIANGLE_CONSTRAINT_H
#define TRIANGLE_CONSTRAINT_H

#include "Common.h"
class RigidBody;
class CollisionPoint;


enum class constraintType {
    resting,
    ballsocket,
    distance,
    fixeddistance,
    slider,
    hinge,
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
    float beta = 1.f;
    bool render = false;
    virtual void solve(float dt) {};
    virtual void gui(int index) {
        ImGui::SliderFloat(("Beta" + std::to_string(index)).c_str(), &beta, 0, 1);
    };
    virtual std::string typeName() = 0;
    virtual void updateMassInverse();
    virtual void buildJacobian() {};
    virtual void buildTrJacobian() {};
    virtual void buildRotJacobian() {};
    virtual void buildJacobian(CollisionPoint& p) {};
    virtual Eigen::VectorXf buildVelocityVector();

    virtual ~Constraint() {}
    virtual void Draw(Shader* s) {}
};

class RestingConstraint : public Constraint {
public:
    RestingConstraint(RigidBody* rb1, RigidBody* rb2);
    ~RestingConstraint();
    std::string typeName() override;

    void solve(CollisionPoint& p, float dt);
    void buildJacobian(CollisionPoint& p);
};

// TODO
class FrictionConstraint : public Constraint {
public:
};

class BallSocketConstraint : public Constraint {
public:
    glm::vec3 fstAnchor, sndAnchor;

    Mesh* body = nullptr;

    BallSocketConstraint(RigidBody *fst, RigidBody *snd);
    ~BallSocketConstraint();
    void gui(int index) override;
    std::string typeName() override;

    bool check();
    void solve(float dt) override;
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
    std::string typeName() override;

    void buildTrJacobian() override;
    void buildRotJacobian() override;

    bool check(); // TODO maybe remove

    void solve(float dt) override;
    void Draw(Shader* s) override;
};

class HingeConstraint : public Constraint {
public:
    glm::vec3 fstAnchor, sndAnchor,
                fstDirAxis, sndDirAxis,
                sndOrthoAxis1, sndOrthoAxis2;
    float minDist = 0, maxDist = 10;
    Mesh* body = nullptr;


    HingeConstraint(RigidBody* fst, RigidBody* snd);
    ~HingeConstraint();
    void gui(int index) override;
    std::string typeName() override;

    void buildJacobian() override;

    void solve(float dt) override;
    void Draw(Shader* s) override;
};

class DistanceConstraint : public Constraint {
public:
    float minD, maxD;
    
    DistanceConstraint(RigidBody* rb1, RigidBody* rb2,
                float minDistance, float maxDistance);
    void gui(int index) override;
    std::string typeName() override;

    bool check();
    void solve(float dt) override;
};

class FixedDistanceConstraint : public Constraint {
public:
    float d;

    FixedDistanceConstraint(RigidBody* rb1, RigidBody* rb2, float distance);
    void buildJacobian() override;
    void gui(int index) override;
    std::string typeName() override;
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
    std::string typeName() override;

};

#endif //TRIANGLE_CONSTRAINT_H
