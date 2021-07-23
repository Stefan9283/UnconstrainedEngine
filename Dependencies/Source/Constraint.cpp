#include "Constraint.h"
#include "PhysicsWorld.h"

#pragma region Basic Constraint
Constraint::Constraint(RigidBody* rb1, RigidBody* rb2) {
    
}
Constraint::~Constraint() {}
#pragma endregion

#pragma region RestingConstraint
void RestingConstraint::buildJacobian(CollisionPoint &p) {
    glm::vec3 ra, rb;
    ra = p.A - rb1->position;
    rb = p.B - rb2->position;

    glm::vec3 rota, rotb;
    rota = glm::cross(ra, p.normal);
    rotb = glm::cross(rb, p.normal);

    // va
    Jacobian(0, 0) = - p.normal.x;
    Jacobian(0, 1) = - p.normal.y;
    Jacobian(0, 2) = - p.normal.z;
    // wa

    Jacobian(0, 3) = - rota.x;
    Jacobian(0, 4) = - rota.y;
    Jacobian(0, 5) = - rota.z;
    // vb
    Jacobian(0, 6) = p.normal.x;
    Jacobian(0, 7) = p.normal.y;
    Jacobian(0, 8) = p.normal.z;
    // wb
    Jacobian(0,  9) = rotb.x;
    Jacobian(0, 10) = rotb.y;
    Jacobian(0, 11) = rotb.z;

}
// solve the collision between 2 bodies based on the constraint
void RestingConstraint::solve(CollisionPoint& p, float dt) {
    Eigen::VectorXf velocity(12);

    for (auto i = 0; i < 3; i++) {
        velocity(i) = rb1->velocity[i];
        velocity(i + 3) = rb1->angularVel[i];
        velocity(i + 6) = rb2->velocity[i];
        velocity(i + 9) = rb2->angularVel[i];
    }

    
    float beta = 0.7f;

    Eigen::VectorXf biasTerm(1);

    glm::vec3 ra, rb;
    ra = p.A - rb1->position;
    rb = p.B - rb2->position;
    
    glm::vec3 relativeVelocity = - rb1->velocity + rb2->velocity 
                                - glm::cross(ra, rb1->angularVel) + glm::cross(ra, rb2->angularVel);
    float closingVelocity = glm::dot(relativeVelocity, p.normal);

    biasTerm(0) =
            - p.depth * beta / dt + closingVelocity * rb1->restitution * rb2->restitution;

    buildJacobian(p);

    Eigen::MatrixXf effectiveMass = Jacobian * invM * Jacobian.transpose();

    // inverse
    effectiveMass(0) = 1.0f / effectiveMass(0);
    // effectiveMass = effectiveMass.llt().solve(Eigen::MatrixXf::Identity(effectiveMass.rows(), effectiveMass.cols()));

    Eigen::VectorXf lambda = effectiveMass * ( - (Jacobian * velocity + biasTerm));
    Eigen::VectorXf old_total_lambda(1);
    old_total_lambda = total_lambda;

    total_lambda(0) = std::max(0.0f, (lambda + total_lambda)(0));

    Eigen::VectorXf dv = invM * Jacobian.transpose() * (total_lambda - old_total_lambda);

    for (int j = 0; j < 3; ++j) {
        rb1->velocity[j]   += dv[j];
        rb1->angularVel[j] += dv[j + 3];
        rb2->velocity[j]   += dv[j + 6];
        rb2->angularVel[j] += dv[j + 9];
    }
}

RestingConstraint::RestingConstraint(RigidBody* rb1, RigidBody* rb2) : Constraint(rb1, rb2) {
    this->rb1 = rb1;
    this->rb2 = rb2;

    invM.resize(12, 12);
    invM.setZero();

    // if the rigid body is immovable then the inverse of its mass approaches infinity
    float invM1, invM2;
    invM1 = rb1->movable ? 1 / rb1->mass : 0;
    invM2 = rb2->movable ? 1 / rb2->mass : 0;

    for (auto i = 0; i < 3; i++) {
        invM(i, i) = invM1;
        invM(i + 6, i + 6) = invM2;
        
        // TODO replace identity matrices with inertia tensors
        invM(i + 3, i + 3) = 1;
        invM(i + 9, i + 9) = 1;
    }

    Jacobian.resize(1, 12);

    total_lambda.resize(1);
    total_lambda.setZero();
}
RestingConstraint::~RestingConstraint() {}
#pragma endregion

#pragma region DistanceConstraint
DistanceConstraint::DistanceConstraint(RigidBody* rb1, RigidBody* rb2, 
                                    float minDistance, float maxDistance) : Constraint(rb1, rb2) {
    minD = minDistance;
    maxD = maxDistance;
    this->rb1 = rb1;
    this->rb2 = rb2;
}

void DistanceConstraint::buildJacobian(CollisionPoint& p) {}
void DistanceConstraint::solve(float dt) {

    float beta = 0.5;

    float dist = glm::distance(rb1->position, rb2->position);

    float limit = minD - dist;
    if (dist > maxD) limit = dist - maxD;

    if (rb1->movable) {
        glm::vec3 c = limit * glm::normalize(rb1->position - rb2->position);
        glm::vec3 v = (- beta / dt) * c;
        v *= 0.9;
        rb1->position = v * dt;
    }
    if (rb2->movable) {
        glm::vec3 c = limit * glm::normalize(rb1->position - rb2->position);
        c = -c;
        glm::vec3 v = (- beta / dt) * c;
        v *= 0.9;
        rb2->position += v * dt;
        rb2->velocity += v;
    }
}
bool DistanceConstraint::check() {
    float d = glm::distance(rb1->position, rb2->position);
    return d <= maxD && d >= minD;
}
void DistanceConstraint::gui(int index) {
    ImGui::DragFloat("min", &minD, 0, maxD);
    ImGui::DragFloat("max", &maxD, minD, 100);
}

#pragma endregion 
