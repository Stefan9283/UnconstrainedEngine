#include "Constraint.h"
#include "PhysicsWorld.h"

#pragma region Basic Constraint
Constraint::Constraint(RigidBody* rb1, RigidBody* rb2) {
    
}
Constraint::~Constraint() {}
#pragma endregion

#pragma region RestingConstraint
void RestingConstraint::buildJacobian(CollisionPoint &p) {
    // va
    Jacobian(0, 0) = - p.normal.x;
    Jacobian(0, 1) = - p.normal.y;
    Jacobian(0, 2) = - p.normal.z;
    // wa
    Jacobian(0, 3) = -0;
    Jacobian(0, 4) = -0;
    Jacobian(0, 5) = -0;
    // vb
    Jacobian(0, 6) = p.normal.x;
    Jacobian(0, 7) = p.normal.y;
    Jacobian(0, 8) = p.normal.z;
    // wb
    Jacobian(0, 9) = 0;
    Jacobian(0, 10) = 0;
    Jacobian(0, 11) = 0;

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

    Eigen::VectorXf biasTerm(1);

    float beta = 0.7f;
    glm::vec3 relativeVelocity = - rb1->velocity + rb2->velocity;
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
        
        //invM(i + 3, i + 3) = 0;
        //invM(i + 9, i + 9) = 0;
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
void DistanceConstraint::solve(CollisionPoint& p, float dt) {

   /* std::cout << glm::to_string(p.A);
    std::cout << glm::to_string(p.B) << "\n";
    std::cout << glm::to_string(rb1->position);
    std::cout << glm::to_string(rb2->position) << "\n";
    rb1->collider->toString();
    rb2->collider->toString();
    std::cout << "//////////////////////////////////////\n";


    glm::vec3 pos1, pos2;
    pos1 = rb1->position + p.normal * p.depth / 2.0f;
    pos2 = rb2->position - p.normal * p.depth / 2.0f;
    
    rb1->velocity = (pos1 - rb1->position) / dt;
    rb2->velocity = (pos2 - rb2->position) / dt;
    
    rb1->position = pos1;
    rb2->position = pos2;*/
}
void DistanceConstraint::check(float dt) {
    float d = glm::distance(rb1->position, rb2->position);


    if (d < minD) {
      /*  glm::vec3 direction = glm::normalize(rb2->position - rb1->position);
        
        glm::vec3 pos1, pos2;
        pos1 = rb1->position - direction * d / 2.0f;
        pos2 = rb2->position + direction * d / 2.0f;

        rb1->velocity = (pos1 - rb1->position) / dt;
        rb2->velocity = (pos2 - rb2->position) / dt;

        rb1->position = pos1;
        rb2->position = pos2;*/
    } else if (d > maxD) {
       /* glm::vec3 direction = glm::normalize(rb2->position - rb1->position);

        glm::vec3 pos1, pos2;
        pos1 = rb1->position + direction * d / 2.0f;
        pos2 = rb2->position - direction * d / 2.0f;

        rb1->velocity = (pos1 - rb1->position) / dt;
        rb2->velocity = (pos2 - rb2->position) / dt;

        rb1->position = pos1;
        rb2->position = pos2;*/
    }
}


#pragma endregion 
