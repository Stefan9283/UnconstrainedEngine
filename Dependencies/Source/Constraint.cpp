#include "Constraint.h"
#include "PhysicsWorld.h"

#pragma region Basic Constraint
Constraint::Constraint() {}
Constraint::~Constraint() {}
#pragma endregion

#pragma region RestingConstraint
void RestingConstraint::buildJacobian(CollisionPoint &p) {
    glm::vec3 r1, r2;
    r1 = p.A - rb1->position;
    r2 = p.B - rb2->position;

    glm::vec3 rot1, rot2;
    rot1 = glm::cross(r1, p.normal);
    rot2 = glm::cross(r2, p.normal);

    // va
    Jacobian(0, 0) = - p.normal.x;
    Jacobian(0, 1) = - p.normal.y;
    Jacobian(0, 2) = - p.normal.z;
    // wa
    if (rb1->movable) {
        Jacobian(0, 3) = - rot1.x;
        Jacobian(0, 4) = - rot1.y;
        Jacobian(0, 5) = - rot1.z;
    } else {
        Jacobian(0, 3) = 0;
        Jacobian(0, 4) = 0;
        Jacobian(0, 5) = 0;

    }
    // vb
    Jacobian(0, 6) = p.normal.x;
    Jacobian(0, 7) = p.normal.y;
    Jacobian(0, 8) = p.normal.z;
    // wb
    if (rb2->movable) {
        Jacobian(0,  9) = rot2.x;
        Jacobian(0, 10) = rot1.y;
        Jacobian(0, 11) = rot2.z;
    } else {
        Jacobian(0,  9) = 0;
        Jacobian(0, 10) = 0;
        Jacobian(0, 11) = 0;
    }
}
// solve the collision between 2 bodies based on the constraint
void RestingConstraint::updateMassInverse() {
    invM.resize(12, 12);
    invM.setZero();
    // if the rigid body is immovable then the inverse of its mass approaches infinity
    float invM1, invM2;
    invM1 = rb1->movable ? 1 / rb1->mass : 0;
    invM2 = rb2->movable ? 1 / rb2->mass : 0;
    for (auto i = 0; i < 3; i++) {
        invM(i, i) = invM1;
        invM(i + 6, i + 6) = invM2;
    }

    glm::mat3 inertia1, inertia2;
    inertia1 = rb1->getInertiaTensor();
    inertia2 = rb2->getInertiaTensor();

    for (auto i = 0; i < 3; i++)
        for (int j = 0; j < 3; ++j) {
            invM(i + 3, j + 3) = inertia1[i][j];
            invM(i + 9, j + 9) = inertia2[i][j];
        }
}
void RestingConstraint::solve(CollisionPoint& p, float dt) {
    updateMassInverse();

    Eigen::VectorXf velocity(12);

    for (auto i = 0; i < 3; i++) {
        velocity(i) = rb1->velocity[i];
        velocity(i + 3) = rb1->angularVel[i];
        velocity(i + 6) = rb2->velocity[i];
        velocity(i + 9) = rb2->angularVel[i];
    }

    float beta = 0.5f;

    Eigen::VectorXf biasTerm(1);

    biasTerm(0) = -(beta / dt) * p.depth;

    glm::vec3 relativeVelocity;
    float closingVelocity;
    if (rb1->movable && rb2->movable) {
        glm::vec3
            r1 = p.A - rb1->position,
            r2 = p.B - rb2->position;

        relativeVelocity = -rb1->velocity + rb2->velocity
                                     - glm::cross(r1, rb1->angularVel) + glm::cross(r2, rb2->angularVel);
    } else {
        relativeVelocity = -rb1->velocity + rb2->velocity;
    }
    closingVelocity = glm::dot(relativeVelocity, p.normal);
    biasTerm(0) += closingVelocity * rb1->restitution * rb2->restitution;

    buildJacobian(p);

    Eigen::MatrixXf effectiveMass = Jacobian * invM * Jacobian.transpose();

    // inverse
    effectiveMass(0) = 1.0f / effectiveMass(0);

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

RestingConstraint::RestingConstraint(RigidBody* rb1, RigidBody* rb2) : Constraint() {
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
                                    float minDistance, float maxDistance) : Constraint() {
    minD = minDistance;
    maxD = maxDistance;
    this->rb1 = rb1;
    this->rb2 = rb2;
}

void DistanceConstraint::solve(float dt) {
    if (!check()) return;

    float beta = 0.5;

    float dist = glm::distance(rb1->position, rb2->position);

    float limit = - minD + dist;
    if (dist > maxD) limit = dist - maxD;

    if (rb1->movable) {
        glm::vec3 c = limit * glm::normalize(rb1->position - rb2->position);
        glm::vec3 v = (- beta / dt) * c;
        v *= 0.9;
        rb1->position += v * dt;
        rb1->velocity += v;
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
    ImGui::SliderFloat("min", &minD, 0, maxD);
    ImGui::SliderFloat("max", &maxD, minD, 100);
}

#pragma endregion 

glm::mat3 crossProductSkewMatrix(glm::vec3 a) {
    auto skew = glm::mat3(0);
    skew[0][1] = - a[2];
    skew[1][0] = + a[2];
    skew[0][2] = + a[1];
    skew[2][0] = - a[1];
    skew[1][2] = - a[0];
    skew[2][1] = + a[0];
    return skew;
}

void limitConstraint::toggle(float lower, float upper) {
    if (!enabled) {
        this->lower = lower;
        this->upper = upper;
    }
    enabled = !enabled;
}
bool limitConstraint::check(float dist) {
    if (!enabled) return true;
    return !(dist - offset < lower || dist - offset > upper);
}

#pragma region GenericConstraint

GenericConstraint::GenericConstraint(RigidBody* fst, RigidBody* snd) {
    first = fst; second = snd;
}
void GenericConstraint::setBreakable(float impulseTreshold) {
    breakable = true; this->impulseTreshold = impulseTreshold;
}
void GenericConstraint::gui(int index) {
    ImGui::Text("Angular");

    ImGui::Checkbox(("Enabled AngX" + std::to_string(index)).c_str(), &angular[0].enabled);
    if (angular[0].enabled) {
        ImGui::SliderFloat(("ang x lower" + std::to_string(index)).c_str(), &angular[0].lower, -10000, angular[0].upper);
        ImGui::SliderFloat(("ang x upper" + std::to_string(index)).c_str(), &angular[0].upper, angular[0].lower, 10000);
    }
    ImGui::Checkbox(("Enabled AngY" + std::to_string(index)).c_str(), &angular[1].enabled);
    if (angular[1].enabled) {
        ImGui::SliderFloat(("ang y lower" + std::to_string(index)).c_str(), &angular[1].lower, -10000, angular[1].upper);
        ImGui::SliderFloat(("ang y upper" + std::to_string(index)).c_str(), &angular[1].upper, angular[1].lower, 10000);
    }

    ImGui::Checkbox(("Enabled AngZ" + std::to_string(index)).c_str(), &angular[2].enabled);
    if (angular[2].enabled) {
        ImGui::SliderFloat(("ang z lower" + std::to_string(index)).c_str(), &angular[2].lower, -10000, angular[2].upper);
        ImGui::SliderFloat(("ang z upper" + std::to_string(index)).c_str(), &angular[2].upper, angular[2].lower, 10000);
    }


    ImGui::Text("Linear");

    ImGui::Checkbox(("Enabled LinX" + std::to_string(index)).c_str(), &linear[0].enabled);
    if (linear[0].enabled) {
        ImGui::SliderFloat(("linear x lower" + std::to_string(index)).c_str(), &linear[0].lower, -10000, linear[0].upper);
        ImGui::SliderFloat(("linear x upper" + std::to_string(index)).c_str(), &linear[0].upper, linear[0].lower,10000);
    }
    ImGui::Checkbox(("Enabled LinY" + std::to_string(index)).c_str(), &linear[1].enabled);
    if (linear[1].enabled) {
        ImGui::SliderFloat(("linear y lower" + std::to_string(index)).c_str(), &linear[1].lower, -10000, linear[1].upper);
        ImGui::SliderFloat(("linear y upper" + std::to_string(index)).c_str(), &linear[1].upper, linear[1].lower,10000);
    }
    ImGui::Checkbox(("Enabled LinZ" + std::to_string(index)).c_str(), &linear[2].enabled);
    if (linear[2].enabled) {
        ImGui::SliderFloat(("linear z lower" + std::to_string(index)).c_str(), &linear[2].lower, -10000, linear[2].upper);
        ImGui::SliderFloat(("linear z upper" + std::to_string(index)).c_str(), &linear[2].upper, linear[2].lower, 10000);
    }
}

// TODO
void GenericConstraint::solve(float dt) {
    if (check()) return;

    auto lin = glm::vec3(0);

    Eigen::MatrixXf Jacobian;
    Jacobian.resize(1, 3);
    Jacobian.setZero();

    for (int i = 0; i < 3; i++) {
        if (angular[i].enabled) {
            Jacobian(0, i) = 1;
        }
        if (linear[i].enabled) lin[i] = 1;
    }

    glm::vec3 dist = second->position - first->position;

    for (int i = 0; i < 3; i++) {
        if (linear[i].lower > dist[i])
            dist[i] = dist[i] - linear[i].lower;
        else if(linear[i].upper < dist[i])
            dist[i] = dist[i] - linear[i].upper;
        else dist[i] = 0;
    }

    float beta = 0.7f;
    glm::vec3 vlin = 0.9f * (- beta / dt) * dist * lin;

    // Rotation

    /*
    second->position += vlin * dt;
    second->velocity += vlin;

    Eigen::VectorXf v_snd, bias;
    v_snd.resize(3);
    bias.resize(1);

    glm::vec3 rot = glm::eulerAngles(second->rotation) - glm::eulerAngles(first->rotation);

    Eigen::MatrixXf inertia;
    inertia.resize(3, 3);
    inertia.setIdentity();


    Eigen::VectorXf totalAngVel;
    totalAngVel.resize(3);
    totalAngVel.setZero();
    for (int i = 0; i < 3; i++) {
        v_snd.setZero();
        v_snd(i) = second->angularVel[i];
        bias(0) = rot[i];
        Jacobian.setZero();
        Jacobian(0, i) = 1;

        Eigen::VectorXf lambda = (Jacobian * inertia.inverse() * Jacobian.transpose()).inverse() * ( - (Jacobian * v_snd + bias));
        Eigen::VectorXf newAng = inertia.inverse() * Jacobian.transpose() * lambda;
        std::cout << newAng.transpose() * dt << "\n";
        totalAngVel += newAng;
    }

    std::cout << "\n";
    */

//    second->rotation = glm::rotate(second->rotation, totalAngVel(0) * dt, glm::vec3(1, 0, 0));
//    second->rotation = glm::rotate(second->rotation, totalAngVel(1) * dt, glm::vec3(0, 1, 0));
//    second->rotation = glm::rotate(second->rotation, totalAngVel(2) * dt, glm::vec3(0, 0, 1));
//
//    for (int i = 0; i < 3; i++)
//        second->angularVel[i] += totalAngVel(i);
}
bool GenericConstraint::check() {
    glm::vec3 rot = glm::eulerAngles(glm::inverse(first->rotation) * second->rotation);
    glm::vec3 dist = - first->position + second->position;
    for (int i = 0; i < 3; i++) {
//        std::cout << angular[i].check(rot[i]) << linear[i].check(dist[i]) << "\n";
        if (!angular[i].check(rot[i])) return false;
        if (!linear[i].check(dist[i])) return false;
    }
    return true;
}


#pragma endregion

#pragma region BallSocket
BallSocketConstraint::BallSocketConstraint(RigidBody *fst, RigidBody *snd) {
    first = fst;
    second = snd;
    glm::vec3 anchor = fst->position + snd->position;
    anchor /= 2.0f;
    fstAnchor = glm::inverse(fst->getTransform()) * glm::vec4(anchor, 1);
    sndAnchor = glm::inverse(snd->getTransform()) * glm::vec4(anchor, 1);
    updateMassInverse();
}
void BallSocketConstraint::updateMassInverse() {
    invM.resize(12, 12);
    invM.setZero();
    // if the rigid body is immovable then the inverse of its mass approaches infinity
    float invM1, invM2;
    invM1 = first->movable ? 1 / first->mass : 0;
    invM2 = second->movable ? 1 / second->mass : 0;
    for (auto i = 0; i < 3; i++) {
        invM(i, i) = invM1;
        invM(i + 6, i + 6) = invM2;
    }

    glm::mat3 inertia1, inertia2;
    inertia1 = first->getInertiaTensor();
    inertia2 = second->getInertiaTensor();

    for (auto i = 0; i < 3; i++)
        for (int j = 0; j < 3; ++j) {
            invM(i + 3, j + 3) = inertia1[i][j];
            invM(i + 9, j + 9) = inertia2[i][j];
        }
}
bool BallSocketConstraint::check() {
    if (first->getTransform() * glm::vec4(fstAnchor, 1)
        != second->getTransform() * glm::vec4(sndAnchor, 1))
        return false;
    return true;
}
void BallSocketConstraint::gui(int index) {
    float anchor[] = {
                        fstAnchor.x, fstAnchor.y, fstAnchor.z,
                        sndAnchor.x, sndAnchor.y, sndAnchor.z
                    };
    ImGui::SliderFloat3(("anchor1 " + std::to_string(index)).c_str(), anchor, -100, 100);
    ImGui::SliderFloat3(("anchor2 " + std::to_string(index)).c_str(), anchor + 3, -100, 100);
    fstAnchor = glm::vec3(anchor[0], anchor[1], anchor[2]);
    sndAnchor = glm::vec3(anchor[3], anchor[4], anchor[5]);
}
void BallSocketConstraint::solve(float dt) {
    if (check()) return;

    updateMassInverse();

    float beta = 0.9f;

    Eigen::VectorXf biasTerm(3);
    Eigen::VectorXf velocity(12);

    Eigen::MatrixXf Jacobian(3, 12);
    Jacobian.setZero();

    glm::vec3
            r1 = first->position + fstAnchor,
            r2 = second->position + sndAnchor;


    for (int i = 0; i < 3; i++) {
        biasTerm(i) =  - beta / dt * (r1[i] - r2[i]);

        velocity(i + 0) = first->velocity[i];
        velocity(i + 3) = first->angularVel[i];
        velocity(i + 6) = second->velocity[i];
        velocity(i + 9) = second->angularVel[i];

        Jacobian(i, i) = -1;
        Jacobian(i, i + 6) = 1;
    }

    glm::mat3 skew1, skew2;
    skew1 = crossProductSkewMatrix(r1);
    skew2 = crossProductSkewMatrix(r2);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; ++j) {
            Jacobian(i, j + 3) = - skew1[i][j];
            Jacobian(i, j + 9) = skew2[i][j];
        }
    }


    Eigen::MatrixXf effectiveMass = Jacobian * invM * Jacobian.transpose();
    // inverse
    effectiveMass = effectiveMass.inverse();

    Eigen::VectorXf lambda = effectiveMass * ( - (Jacobian * velocity + biasTerm));

    Eigen::VectorXf dv = invM * Jacobian.transpose() * lambda;

    /*
    float beta = 0.7f;

    Eigen::VectorXf biasTerm(1);

    Eigen::VectorXf velocity;

    velocity.resize(12);


    Eigen::MatrixXf Jacobian;
    Jacobian.resize(1, 12);


    glm::vec3
        r1 = first->position + fstAnchor,
        r2 = second->position + sndAnchor;

    biasTerm(0) =
            - beta / dt * glm::length(r1 - r2);


    for (int i = 0; i < 3; i++) {
        velocity(i) = first->velocity[i];
        velocity(i + 3) = first->angularVel[i];
        velocity(i + 6) = second->velocity[i];
        velocity(i + 9) = second->angularVel[i];

        Jacobian(i) = -1;
        Jacobian(i + 3) = r1[i];
        Jacobian(i + 6) = 1;
        Jacobian(i + 9) = r2[i];
    }

    std::cout << Jacobian << "\n";


    Eigen::MatrixXf effectiveMass = Jacobian * invM * Jacobian.transpose();
    // inverse
    effectiveMass(0) = 1.0f / effectiveMass(0);
    Eigen::VectorXf lambda = effectiveMass * ( - (Jacobian * velocity + biasTerm));
    lambda(0) = std::max(0.0f, lambda(0));
    std::cout << lambda << " l\n";

    Eigen::VectorXf dv = 0.9f * invM * Jacobian.transpose() * lambda;
 */

    for (int j = 0; j < 3; ++j) {
        first->angularVel[j]  += dv[j + 3];
        second->angularVel[j] += dv[j + 9];
        first->velocity[j]    += dv[j];
        second->velocity[j]   += dv[j + 6];
    }
}
#pragma endregion

