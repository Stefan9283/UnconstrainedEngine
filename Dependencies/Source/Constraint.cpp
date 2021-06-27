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
        /*
        else {
            if (!rb1->movable) {
                Jacobian(i, 6) = 1;
                Jacobian(i + 1, 6) = 1;
                i+=2;
            } else if (!rb2->movable) {
                Jacobian(i, 0) = -1;
                Jacobian(i + 1, 0) = -1;
                i+=2;
            } else {
                Jacobian(i, 0) = -1;
                Jacobian(i + 1, 6) = 1;
                i+=2;
            }
            Jacobian(i, 0) = -1;
            Jacobian(i + 1, 6) = 1;
            i += 2;
        }*/
    }
    
    if (linearY.type == DISTANCE) {
        if (p.normal.y) {
            Jacobian(i, 1) = - p.normal.y;
            Jacobian(i, 7) = p.normal.y;
            i++;
        }
        /*
        else {
            if (!rb1->movable) {
                Jacobian(i, 7) = 1;
                Jacobian(i + 1, 7) = 1;
                i+=2;
            } else if (!rb2->movable) {
                Jacobian(i, 1) = -1;
                Jacobian(i + 1, 1) = -1;
                i+=2;
            } else {
                Jacobian(i, 1) = -1;
                Jacobian(i + 1, 7) = 1;
                i+=2;
            }
            Jacobian(i, 1) = -1;
            Jacobian(i + 1, 7) = 1;
            i += 2;
        }*/
    } 
    
    if (linearZ.type == DISTANCE) {
        if (p.normal.z) {
            Jacobian(i, 2) = - p.normal.z;
            Jacobian(i, 8) = p.normal.z;
            i++;
        }
        /*
        else {
            if (!rb1->movable) {
                Jacobian(i, 8) = 1;
                Jacobian(i + 1, 8) = 1;
                i+=2;
            } else if (!rb2->movable) {
                Jacobian(i, 2) = -1;
                Jacobian(i + 1, 2) = -1;
                i+=2;
            } else {
                Jacobian(i, 2) = -1;
                Jacobian(i + 1, 8) = 1;
                i+=2;
            }
            Jacobian(i, 2) = -1;
            Jacobian(i + 1, 8) = 1;
            i += 2;
        }*/
    }
    // TODO do the same thing but for angular velocity
    //
    Jacobian.conservativeResize(i, 12);
}


// solve the collision between 2 bodies based on the constraint

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

    //Eigen::MatrixXd JacobianTranspose = Jacobian.transpose();
    Eigen::MatrixXd invDen = (Jacobian * invM * Jacobian.transpose()); // .inverse();
    Eigen::MatrixXd Identity;
    Identity.resize(invDen.rows(), invDen.cols());
    Identity.setIdentity();
    invDen = - invDen.llt().solve(Identity);

    //std::cout << Identity << "\n\n";
    //std::cout << invDen << "\n\n\n";

    Eigen::VectorXd corrective_lambda = invDen * Jacobian * velocity;

    /*
    std::cout << Jacobian << " = J\n\n";
    std::cout << invM << " = invM\n\n";
    std::cout << Jacobian * invM << " invDen\n\n";
    std::cout << Jacobian * invM * Jacobian.transpose() << " invDen\n\n";
    std::cout << invDen << " invDen\n\n";
    //std::cout << cached_lambda.transpose() << " cache\n";
    std::cout << corrective_lambda.transpose() << " corr\n\n";
    */

    Eigen::VectorXd old_lambda = getCachedLambda(p);
    Eigen::VectorXd accumulated_lambda = corrective_lambda + old_lambda;

    Eigen::MatrixXd P = invM * Jacobian.transpose();
    Eigen::VectorXd dv = P * (accumulated_lambda - old_lambda);

    setCachedLambda(p, accumulated_lambda);

    velocity = velocity + dv;

    std::cout << glm::to_string(p.normal) <<
        glm::to_string(p.A) << " " << glm::to_string(p.B) << "\n";

    std::cout << Jacobian << " = J\n\n"
              << velocity.transpose() << " = v\n\n"
              << dv.transpose() << " delta\n\n\n\n";

    float beta_restitution = 0.8f;


    float lost_energy = 0.2f;
    glm::vec3 bounce1, bounce2;
    bounce1 = //glm::dot(p.normal, rb1->velocity) * 
        glm::length(rb1->velocity) * (1 - lost_energy) *
        (- p.normal);
    bounce2 = //glm::dot(p.normal, rb2->velocity) * 
        glm::length(rb2->velocity) * (1 - lost_energy) *
        (+ p.normal);

    for (int j = 0; j < 3; ++j) {
        rb1->velocity[j] = velocity[j] * (1 - beta_restitution);
        rb1->angularVel[j] = velocity[j + 3];
        rb2->velocity[j] = velocity[j + 6] * (1 - beta_restitution);
        rb2->angularVel[j] = velocity[j + 9];
    }

    rb1->velocity += bounce1;
    rb2->velocity += bounce2;
}

void Constraint::setCachedLambda(CollisionPoint& p, Eigen::VectorXd& l) {
    int i = 0;

    std::vector<int> values;

    if (linearX.type == DISTANCE) {
        if (p.normal.x) {
            values.push_back(0);
            i++;
        }
    }

    if (linearY.type == DISTANCE) {
        if (p.normal.y) {
            values.push_back(1);
            i++;
        }
    }

    if (linearZ.type == DISTANCE) {
        if (p.normal.z) {
            values.push_back(2);
            i++;
        }
    }
    // TODO do the same thing but for angular velocity
    //
    std::sort(values.begin(), values.end());
    for (size_t i = 0; i < values.size(); i++)
        cached_lambda(values[i]) = l(i);
}
Eigen::VectorXd Constraint::getCachedLambda(CollisionPoint& p) {
    int i = 0;

    std::vector<int> values;

    if (linearX.type == DISTANCE) {
        if (p.normal.x) {
            values.push_back(0);
            i++;
        }
    }

    if (linearY.type == DISTANCE) {
        if (p.normal.y) {
            values.push_back(1);
            i++;
        }
    }

    if (linearZ.type == DISTANCE) {
        if (p.normal.z) {
            values.push_back(2);
            i++;
        }
    }
    // TODO do the same thing but for angular velocity
    //

    Eigen::VectorXd V(i);

    std::sort(values.begin(), values.end());
    for (size_t i = 0; i < values.size(); i++)
        V(i) = cached_lambda(values[i]);

    return V;
}

Constraint::Constraint(RigidBody* rb1, RigidBody* rb2) {
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
        invM(6 + i, 6 + i) = invM2;
    }

    Jacobian.resize(12, 12);
    Jacobian.setZero();

    int i = 0;
    if (linearX.type != FREE)
        i++;
    if (linearY.type != FREE)
        i++;
    if (linearZ.type != FREE)
        i++;
    if (angularX.type != FREE)
        i++;
    if (angularY.type != FREE)
        i++;
    if (angularZ.type != FREE)
        i++;

    cached_lambda.resize(i);
    cached_lambda.setZero();
}
