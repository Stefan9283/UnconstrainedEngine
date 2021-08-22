#ifndef TRIANGLE_CONSTRAINT_H
#define TRIANGLE_CONSTRAINT_H

#include "Common.h"
class RigidBody;
class CollisionPoint;


enum class constraintType {
    resting,
    friction,
    
    distance,
    fixeddistance,
    slider,
    ballsocket,
    hinge,
    generic,

    dontKnow,
    dontKnow1, // SingleBody
    dontKnow2  // TwoBodies
};

class Constraint {
public:
    constraintType type = constraintType::dontKnow;
    Eigen::MatrixXf Jacobian, invM;
    float beta = 1.f, gamma = 0.f;
    bool render = false;
    virtual ~Constraint() {}
    virtual std::string typeName() = 0;
    virtual void updateMassInverse() = 0;
    virtual void buildJacobian() {};
    virtual void buildTrJacobian() {};
    virtual void buildRotJacobian() {};
    virtual Eigen::VectorXf buildVelocityVector() = 0;
    virtual void solve(float dt) {};
    virtual void gui(int index) {
        ImGui::SliderFloat(("Beta" + std::to_string(index)).c_str(), &beta, 0, 1);
        ImGui::SliderFloat(("Gamma" + std::to_string(index)).c_str(), &gamma, 0, 1);
    };
    virtual void Draw(Shader* s) {}
};

#pragma region SingleBodyConstraints

class SingleBodyConstraint : public Constraint {
    RigidBody* rigidbody;
    Eigen::VectorXf buildVelocityVector() override;
    void updateMassInverse() override;
};

#pragma endregion

#pragma region TwoBodiesConstraints

class TwoBodiesConstraint : public Constraint {
public:
    RigidBody *first = nullptr, 
              *second = nullptr;
    
    Eigen::VectorXf buildVelocityVector() override;
    void updateMassInverse() override;
};

#pragma region ContactConstraints

class ContactConstraint : public TwoBodiesConstraint {
public:
    virtual void buildJacobian(CollisionPoint& p) = 0;
    virtual void solve(CollisionPoint& p, float dt) = 0;
};

class RestingConstraint : public ContactConstraint {
public:
    Eigen::VectorXf total_lambda;
    RestingConstraint(RigidBody* rb1, RigidBody* rb2);
    ~RestingConstraint();
    std::string typeName() override;

    void solve(CollisionPoint& p, float dt) override;
    void buildJacobian(CollisionPoint& p);
};

// TODO
class FrictionConstraint : public ContactConstraint {
public:
};
#pragma endregion

class BallSocketConstraint : public TwoBodiesConstraint {
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

class SliderConstraint : public TwoBodiesConstraint {
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

class HingeConstraint : public TwoBodiesConstraint {
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

class DistanceConstraint : public TwoBodiesConstraint {
public:
    float minD, maxD;
    
    DistanceConstraint(RigidBody* rb1, RigidBody* rb2,
                float minDistance, float maxDistance);
    void gui(int index) override;
    std::string typeName() override;

    bool check();
    void solve(float dt) override;
};

class FixedDistanceConstraint : public TwoBodiesConstraint {
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

class GenericConstraint : public TwoBodiesConstraint {
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

#pragma endregion


#endif //TRIANGLE_CONSTRAINT_H
