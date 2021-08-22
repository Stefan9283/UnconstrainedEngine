#include <ObjLoad.h>
#include "Constraint.h"
#include "PhysicsWorld.h"

#pragma region More Funcs
void display_4x4(glm::mat4 m4)
{
    for (int col = 0; col < 4; ++col) {
        std::cout << "| ";
        for (int row = 0; row < 4; ++row) {
            std::cout << m4[row][col] << '\t';
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}
void display_3x3(glm::mat3 m4 )
{
    for (int col = 0; col < 3; ++col) {
        std::cout << "| ";
        for (int row = 0; row < 3; ++row) {
            std::cout << m4[row][col] << '\t';
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

Eigen::Matrix3f crossProductSkewMatrix(glm::vec3 a) {
    Eigen::Matrix3f skew;
    skew.setZero();
    skew(0, 1) = - a.z;
    skew(0, 2) =   a.y;
    skew(1, 0) = a.z;
    skew(1, 2) = -a.x;
    skew(2, 0) = - a.y;
    skew(2, 1) =   a.x;
    return skew;
}
glm::vec3 getARandomOrthogonalVector(glm::vec3 v) {
    glm::vec3 w{};
    bool modified = false;
    for (int i = 0; i < 3; i++)
        if (v[i] == 0) {
            modified = true;
            w[i] = 1;
        }
    if (!modified) {
        return glm::cross(v, glm::vec3(1, 1, 1));
        //glm::mat3 rotation = glm::rotate(glm::mat4(1), glm::radians(90.f), glm::normalize(glm::vec3(1, 1, 1)));
        //return rotation * v;
    }
    return w;
}

std::vector<glm::vec3> getOrthogonalVectors(glm::vec3 reference) {
    std::vector<glm::vec3> ortho;


    ortho.push_back(glm::normalize(getARandomOrthogonalVector(reference)));
    
    //std::cout << glm::to_string(reference) << glm::to_string(ortho[0]) << "\n";
    //std::cout << glm::dot(reference, ortho[0]) << "\n";

    ortho.push_back(glm::normalize(glm::cross(ortho[0], reference)));
    return ortho;
}

Eigen::VectorXf generalDVSolver(Eigen::MatrixXf& invMass, Eigen::MatrixXf& Jacobian,
        Eigen::VectorXf velocity, Eigen::VectorXf& total_lambda, Eigen::VectorXf& BiasTerm,
        float gammaOVERdt) {
    Eigen::MatrixXf effectiveMass = Jacobian * invMass * Jacobian.transpose();

    for (int i = 0; i < effectiveMass.rows(); i++)
        effectiveMass(i, i) = effectiveMass(i, i) + gammaOVERdt;

    effectiveMass = effectiveMass.inverse();

    Eigen::VectorXf lambda = effectiveMass * (-(Jacobian * velocity + BiasTerm));
    Eigen::VectorXf old_total_lambda(1);
    old_total_lambda = total_lambda;
    Eigen::VectorXf dv;

    total_lambda = (lambda + total_lambda); // .cwiseMax(0);
    dv = invMass * Jacobian.transpose() * (total_lambda - old_total_lambda);

    return dv;
}
Eigen::VectorXf generalDVSolver(Eigen::MatrixXf& invMass, Eigen::MatrixXf& Jacobian,
        Eigen::VectorXf velocity, Eigen::VectorXf& BiasTerm, float gammaOVERdt) {
    Eigen::MatrixXf effectiveMass = Jacobian * invMass * Jacobian.transpose();

    for (int i = 0; i < effectiveMass.rows(); i++)
        effectiveMass(i, i) = effectiveMass(i, i) + gammaOVERdt;

    effectiveMass = effectiveMass.inverse();
    Eigen::VectorXf lambda = effectiveMass * (-(Jacobian * velocity + BiasTerm));
    return invMass * Jacobian.transpose() * lambda;
}
void addDeltaVelocities(RigidBody* rb1, RigidBody* rb2, Eigen::VectorXf dv) {
    for (int i = 0; i < 3; i++) {
        rb1->velocity[i] += dv(i);
        rb2->velocity[i] += dv(i + 6);
        rb1->angularVel[i] += dv(i + 3);
        rb2->angularVel[i] += dv(i + 9);
    }
}
#pragma endregion

void SingleBodyConstraint::updateMassInverse() {
    invM.resize(6, 6);
    invM.setZero();
    // if the rigid body is immovable then the inverse of its mass approaches 0
    float invM1;
    invM1 = rigidbody->movable ? 1 / rigidbody->mass : 0;
    for (size_t i = 0; i < 3; i++) {
        invM(i, i) = invM1;
    }

    glm::mat3 inertia1;
    inertia1 = glm::inverse(rigidbody->getInertiaTensor());

    for (auto i = 0; i < 3; i++)
        for (auto j = 0; j < 3; ++j) {
            invM(i + 3, j + 3) = inertia1[i][j];
        }
}
Eigen::VectorXf SingleBodyConstraint::buildVelocityVector() {
    Eigen::VectorXf velocity(6);
    for (auto i = 0; i < 3; i++) {
        velocity(i) = rigidbody->velocity[i];
        velocity(i + 3) = rigidbody->angularVel[i];
    }
    return velocity;
}

void TwoBodiesConstraint::updateMassInverse() {
    invM.resize(12, 12);
    invM.setZero();
    // if the rigid body is immovable then the inverse of its mass approaches 0
    float invM1, invM2;
    invM1 = first->movable ? 1 / first->mass : 0;
    invM2 = second->movable ? 1 / second->mass : 0;
    for (size_t i = 0; i < 3; i++) {
        invM(i, i) = invM1;
        invM(i + 6, i + 6) = invM2;
    }

    glm::mat3 inertia1, inertia2;
    inertia1 = glm::inverse(first->getInertiaTensor());
    inertia2 = glm::inverse(second->getInertiaTensor());

    for (auto i = 0; i < 3; i++)
        for (auto j = 0; j < 3; ++j) {
            invM(i + 3, j + 3) = inertia1[i][j];
            invM(i + 9, j + 9) = inertia2[i][j];
        }
}
Eigen::VectorXf TwoBodiesConstraint::buildVelocityVector() {
    Eigen::VectorXf velocity(12);
    for (auto i = 0; i < 3; i++) {
        velocity(i) = first->velocity[i];
        velocity(i + 3) = first->angularVel[i];
        velocity(i + 6) = second->velocity[i];
        velocity(i + 9) = second->angularVel[i];
    }
    return velocity;
}

#pragma region RestingConstraint
void RestingConstraint::buildJacobian(CollisionPoint &p) {
    glm::vec3 r1, r2;
    r1 = p.A - first->position;
    r2 = p.B - second->position;

    glm::vec3 rot1, rot2;
    rot1 = glm::cross(r1, p.normal);
    rot2 = glm::cross(r2, p.normal);

    // va
    Jacobian(0, 0) = - p.normal.x;
    Jacobian(0, 1) = - p.normal.y;
    Jacobian(0, 2) = - p.normal.z;
    // wa
    if (first->movable) {
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
    if (second->movable) {
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
void RestingConstraint::solve(CollisionPoint& p, float dt) {

    Eigen::VectorXf velocity = buildVelocityVector();

    Eigen::VectorXf biasTerm(1);

    biasTerm(0) = (- beta / dt) * p.depth;

    glm::vec3 relativeVelocity;
    float closingVelocity;
    if (first->movable && second->movable) {
        glm::vec3
            r1 = p.A - first->position,
            r2 = p.B - second->position;

        relativeVelocity = - first->velocity + second->velocity
                                     - glm::cross(r1, first->angularVel) + glm::cross(r2, second->angularVel);
    } else {
        relativeVelocity = - first->velocity + second->velocity;
    }

    closingVelocity = glm::dot(relativeVelocity, p.normal);
    biasTerm(0) += closingVelocity * first->restitution * second->restitution;

    buildJacobian(p);
    updateMassInverse();

    Eigen::VectorXf dv = generalDVSolver(invM, Jacobian, velocity, total_lambda, biasTerm, gamma/dt);

    addDeltaVelocities(first, second, dv);
}
RestingConstraint::RestingConstraint(RigidBody* rb1, RigidBody* rb2) {
    this->first = rb1;
    this->second = rb2;
    this->type = constraintType::resting;

    Jacobian.resize(1, 12);

    total_lambda.resize(1);
    total_lambda.setZero();
}
RestingConstraint::~RestingConstraint() {}
std::string RestingConstraint::typeName() { return "Resting"; }
#pragma endregion

#pragma region BallSocket

BallSocketConstraint::BallSocketConstraint(RigidBody *fst, RigidBody *snd) {
    this->type = constraintType::ballsocket;
    first = fst;
    second = snd;
    glm::vec3 anchor = first->position + snd->position + glm::vec3(1);
    anchor /= 2.0f;
    fstAnchor = glm::inverse(glm::mat3(first->getRotationMatrix())) * (anchor - first->position);
    sndAnchor = glm::inverse(glm::mat3(second->getRotationMatrix())) * (anchor - second->position);
}
BallSocketConstraint::~BallSocketConstraint() {
    delete body;
}
void BallSocketConstraint::buildJacobian() {
    Jacobian.resize(3, 12);
    Jacobian.setZero();

    glm::vec3
            r1 = glm::mat3(first->getRotationMatrix()) * fstAnchor,
            r2 = glm::mat3(second->getRotationMatrix()) * sndAnchor;
    Eigen::Matrix3f
            skew1 = crossProductSkewMatrix(r1),
            skew2 = crossProductSkewMatrix(r2);

    for (auto i = 0; i < 3; i++) {
        Jacobian(i, i) = -1;
        Jacobian(i, i + 6) = 1;
        for (auto j = 0; j < 3; ++j) {
            Jacobian(i, j + 3) =   skew1(i, j);
            Jacobian(i, j + 9) = - skew2(i, j);
        }
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
    ImGui::SliderFloat3(("anchor" +  std::to_string(first->id) + " " + std::to_string(index)).c_str(), anchor, -100, 100);
    ImGui::SliderFloat3(("anchor" + std::to_string(second->id) + " " + std::to_string(index)).c_str(), anchor + 3, -100, 100);
    fstAnchor = glm::vec3(anchor[0], anchor[1], anchor[2]);
    sndAnchor = glm::vec3(anchor[3], anchor[4], anchor[5]);
    Constraint::gui(index);
    ImGui::Checkbox(("Render" + std::to_string(index)).c_str(), &render);
    if (render && !body) body = readObj("3D/Sphere.obj");
}
std::string BallSocketConstraint::typeName() { return "BallSocket"; }

void BallSocketConstraint::solve(float dt) {
    Eigen::VectorXf biasTerm(3);

    glm::vec3
            r1 = glm::mat3(first->getRotationMatrix()) * fstAnchor,
            r2 = glm::mat3(second->getRotationMatrix()) * sndAnchor,
            x1 = first->position,
            x2 = second->position;

    glm::vec3 biasVec = (x2 + r2) - (x1 + r1);

    for (auto i = 0; i < 3; i++) 
        biasTerm(i) =  beta * biasVec[i] / dt;

    Eigen::VectorXf velocity = buildVelocityVector();

    updateMassInverse();
    buildJacobian();
    
    Eigen::VectorXf dv = generalDVSolver(invM, Jacobian, velocity, biasTerm, gamma/dt);
    addDeltaVelocities(first, second, dv);
}

void BallSocketConstraint::Draw(Shader* s) {
    if (render && body) {
        glm::vec3
                r1 = glm::mat3(first->getRotationMatrix()) * fstAnchor,
                r2 = glm::mat3(second->getRotationMatrix()) * sndAnchor,
                x1 = first->position,
                x2 = second->position;

        body->localTransform.sc = glm::vec3(0.3f);
        body->solidON = false;
        body->wireframeON = true;

        body->localTransform.tr = x1 + r1;
        s->setVec3("color", glm::vec3(1, 0, 0));
        body->Draw(s);

        body->localTransform.tr = x2 + r2;
        s->setVec3("color", glm::vec3(0, 0, 1));
        body->Draw(s);
    }
}

#pragma endregion

#pragma region SliderConstraint

SliderConstraint::SliderConstraint(RigidBody *fst, RigidBody *snd) {
    this->type = constraintType::slider;
    first = fst;
    second = snd;
    glm::vec3 anchor = fst->position + snd->position;
    anchor /= 2.0f;
    fstAnchor = glm::inverse(fst->getTransform()) * glm::vec4(anchor, 1);
    sndAnchor = glm::inverse(snd->getTransform()) * glm::vec4(anchor, 1);
    directionAxis = glm::normalize(snd->position - fst->position);
    updateMassInverse();
}
SliderConstraint::~SliderConstraint() {
    delete body;
}
void SliderConstraint::gui(int index) {
    float anchor[] = {
            fstAnchor.x, fstAnchor.y, fstAnchor.z,
            sndAnchor.x, sndAnchor.y, sndAnchor.z
    };
    ImGui::SliderFloat3(("anchor1 " + std::to_string(index)).c_str(), anchor, -100, 100);
    ImGui::SliderFloat3(("anchor2 " + std::to_string(index)).c_str(), anchor + 3, -100, 100);
    fstAnchor = glm::vec3(anchor[0], anchor[1], anchor[2]);
    sndAnchor = glm::vec3(anchor[3], anchor[4], anchor[5]);
    float dirAxis[] = {directionAxis.x, directionAxis.y, directionAxis.z};
    ImGui::SliderFloat3(("directionAxis " + std::to_string(index)).c_str(), dirAxis, -1, 1);
    Constraint::gui(index);
    directionAxis = glm::normalize(glm::vec3(dirAxis[0], dirAxis[1], dirAxis[2]));
    ImGui::Checkbox(("Render"  + std::to_string(index)).c_str(), &render);
    if (render && !body) body = readObj("3D/Sphere.obj");
    
}
bool SliderConstraint::check() { return false; }
void SliderConstraint::solve(float dt) {
    if (check()) return;

    updateMassInverse();

    Eigen::VectorXf biasTermTr(2), biasTermRot(3);
    Eigen::VectorXf velocity = buildVelocityVector();

    glm::vec3
            r1 = glm::mat3(first->getRotationMatrix()) * fstAnchor,
            r2 = glm::mat3(second->getRotationMatrix()) * sndAnchor,
            x1 = first->position,
            x2 = second->position;
    std::vector<glm::vec3> ortho = getOrthogonalVectors(directionAxis);

    glm::vec3 biasRot = glm::eulerAngles(second->rotation * glm::inverse(first->rotation));
    biasTermTr(0) = glm::dot((x2 + r2 - x1 - r1), ortho[0]);
    biasTermTr(1) = glm::dot((x2 + r2 - x1 - r1), ortho[1]);
    biasTermTr = biasTermTr * (beta / dt);


    for (auto i = 0; i < 3; i++) 
        biasTermRot(i) =  (beta / dt) * biasRot[i];

    Eigen::VectorXf dvTr, dvRot;

    buildTrJacobian();
    dvTr = generalDVSolver(invM, Jacobian, velocity, biasTermTr, gamma/dt); //invM * Jacobian.transpose() * lambda;

    buildRotJacobian();
    dvRot = generalDVSolver(invM, Jacobian, velocity, biasTermRot, gamma/dt); //invM * Jacobian.transpose() * lambda;

    addDeltaVelocities(first, second, dvRot + dvTr);
}
void SliderConstraint::buildTrJacobian() {
    Jacobian.resize(2, 12);
    Jacobian.setZero();
    std::vector<glm::vec3> ortho = getOrthogonalVectors(directionAxis);
    glm::vec3
            r1 = glm::mat3(first->getRotationMatrix()) * fstAnchor,
            r2 = glm::mat3(second->getRotationMatrix()) * sndAnchor;

    glm::vec3 cr11, cr21, cr12, cr22;
    cr11 = - glm::cross(r1, ortho[0]); // TODO add limit u
    cr21 =   glm::cross(r2, ortho[0]);
    cr12 = - glm::cross(r1, ortho[1]); // TODO add limit u
    cr22 =   glm::cross(r2, ortho[1]);
    for (auto i = 0; i < 3; ++i) {
        Jacobian(0, i) = - ortho[0][i];
        Jacobian(0, i + 3) = cr11[i];
        Jacobian(0, i + 6) = ortho[0][i];
        Jacobian(0, i + 9) = cr21[i];

        Jacobian(1, i) = - ortho[1][i];
        Jacobian(1, i + 3) = cr12[i];
        Jacobian(1, i + 6) = ortho[1][i];
        Jacobian(1, i + 9) = cr22[i];
    }
}
void SliderConstraint::buildRotJacobian() {
    Jacobian.resize(3, 12);
    Jacobian.setZero();
    for (size_t i = 0; i < 3; i++) {
        Jacobian(i, i + 3) = -1;
        Jacobian(i, i + 9) = 1;
    }
}
std::string SliderConstraint::typeName() { return "Slider"; }

void SliderConstraint::Draw(Shader* s) {
    if (render && body) {
        glm::vec3
            r1 = glm::mat3(first->getRotationMatrix()) * fstAnchor,
            r2 = glm::mat3(second->getRotationMatrix()) * sndAnchor;

        body->localTransform.tr = first->position + r1;
        body->localTransform.sc = glm::vec3(0.3f);
        body->solidON = false;
        body->wireframeON = true;
        s->setVec3("color", glm::vec3(1, 0, 0));
        body->Draw(s);

        body->localTransform.tr = second->position + r2;
        s->setVec3("color", glm::vec3(0, 0, 1));
        body->Draw(s);
    }
}


#pragma endregion

#pragma region Hinge
HingeConstraint::HingeConstraint(RigidBody* fst, RigidBody* snd) {
    first = fst;
    second = snd;
    
    glm::vec3 anchor = fst->position + snd->position;
    anchor /= 2.0f;
    fstAnchor = glm::inverse(fst->getTransform()) * glm::vec4(anchor, 1);
    sndAnchor = glm::inverse(snd->getTransform()) * glm::vec4(anchor, 1);
    
    glm::vec3 axis = glm::vec3(0, 1, 0);

    glm::mat3 rb1WorldToLocalRot = glm::inverse(glm::mat3(fst->getRotationMatrix()));
    glm::mat3 rb2WorldToLocalRot = glm::inverse(glm::mat3(snd->getRotationMatrix()));

    fstDirAxis = rb1WorldToLocalRot * axis;
    sndDirAxis = rb2WorldToLocalRot * axis;

    sndOrthoAxis1 = glm::vec3(1, 0, 0);
    sndOrthoAxis2 = glm::vec3(0, 0, 1);
/*
    std::vector<glm::vec3> ortho = getOrthogonalVectors(axis);
    sndOrthoAxis1 = rb2WorldToLocalRot * ortho[0];
    sndOrthoAxis2 = rb2WorldToLocalRot * ortho[1];
*/
}
HingeConstraint::~HingeConstraint() { delete body;  }
void HingeConstraint::gui(int index) {
    float anchor[] = {
        fstAnchor.x, fstAnchor.y, fstAnchor.z,
        sndAnchor.x, sndAnchor.y, sndAnchor.z
    };
    ImGui::SliderFloat3(("anchor1 " + std::to_string(index)).c_str(), anchor, -100, 100);
    ImGui::SliderFloat3(("anchor2 " + std::to_string(index)).c_str(), anchor + 3, -100, 100);
    fstAnchor = glm::vec3(anchor[0], anchor[1], anchor[2]);
    sndAnchor = glm::vec3(anchor[3], anchor[4], anchor[5]);
    float dirAxis[] = { fstDirAxis.x, fstDirAxis.y, fstDirAxis.z,
                        sndDirAxis.x, sndDirAxis.y, sndDirAxis.z };
    ImGui::SliderFloat3(("hingeAxis1 " + std::to_string(index)).c_str(), dirAxis, -1, 1);
    ImGui::SliderFloat3(("hingeAxis2 " + std::to_string(index)).c_str(), dirAxis + 3, -1, 1);
    fstDirAxis = glm::normalize(glm::vec3(dirAxis[0], dirAxis[1], dirAxis[2]));
    sndDirAxis = glm::normalize(glm::vec3(dirAxis[3], dirAxis[4], dirAxis[5]));
    Constraint::gui(index);
    ImGui::Checkbox(("Render" + std::to_string(index)).c_str(), &render);
    if (render && !body) body = readObj("3D/Sphere.obj");
}
void HingeConstraint::buildJacobian() {
    Jacobian.resize(5, 12);
    Jacobian.setZero();
    for (int i = 0; i < 3; i++) {
        Jacobian(i, i) = -1;
        Jacobian(i, i + 6) = 1;
    }

    glm::mat3 R1, R2;
    R1 = first->getRotationMatrix();
    R2 = second->getRotationMatrix();

    // TRANSLATION

    glm::vec3 r1, r2;
    r1 = R1 * fstAnchor;
    r2 = R2 * sndAnchor;

    Eigen::Matrix3f r1x, r2x;
    r1x = crossProductSkewMatrix(r1);
    r2x = crossProductSkewMatrix(r2);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; ++j) {
            Jacobian(i, j + 3) =   r1x(i, j);
            Jacobian(i, j + 9) = - r2x(i, j);
        }
    }

    // ROTATION
    glm::vec3 a1, b2, c2, cr_ba, cr_ca;
    b2 = R2 * sndOrthoAxis1;
    c2 = R2 * sndOrthoAxis2;
    a1 = R1 * fstDirAxis;
    cr_ba = glm::cross(b2, a1);
    cr_ca = glm::cross(c2, a1);

    for (int i = 0; i < 3; i++) {
        Jacobian(3, i + 3) = - cr_ba[i];
        Jacobian(3, i + 9) =   cr_ba[i];
        Jacobian(4, i + 3) = - cr_ca[i];
        Jacobian(4, i + 9) =   cr_ca[i];
    }
}
void HingeConstraint::solve(float dt) {
    Eigen::VectorXf velocity = buildVelocityVector();
    updateMassInverse();
    this->buildJacobian();

    Eigen::VectorXf biasTerm(5);
    biasTerm.setZero();

    glm::mat3 R1, R2;
    R1 = first->getRotationMatrix();
    R2 = second->getRotationMatrix();

    // TRANSLATION
    glm::vec3
            r1 = R1 * fstAnchor,
            r2 = R2 * sndAnchor,
            x1 = first->position,
            x2 = second->position;

    glm::vec3 biasVec = (x2 + r2) - (x1 + r1);
    for (auto i = 0; i < 3; i++)
        biasTerm(i) = biasVec[i];

    // ROTATION
    glm::vec3 a1, b2, c2;
    b2 = R2 * sndOrthoAxis1;
    c2 = R2 * sndOrthoAxis2;
    a1 = R1 * fstDirAxis;
    biasTerm(3) = glm::dot(a1, b2);
    biasTerm(4) = glm::dot(a1, c2);

    biasTerm = (beta / dt) * biasTerm;

    Eigen::VectorXf dv = generalDVSolver(invM, Jacobian, velocity, biasTerm, gamma/dt);
    addDeltaVelocities(first, second, dv);
}
void HingeConstraint::Draw(Shader* s) {
    if (render && body) {
        glm::vec3
            r1 = glm::mat3(first->getRotationMatrix()) * fstAnchor,
            r2 = glm::mat3(second->getRotationMatrix()) * sndAnchor;

        body->localTransform.tr = first->position + r1;
        body->localTransform.sc = glm::vec3(0.3f);
        body->solidON = false;
        body->wireframeON = true;
        s->setVec3("color", glm::vec3(1, 0, 0));
        body->Draw(s);

        body->localTransform.tr = second->position + r2;
        s->setVec3("color", glm::vec3(0, 0, 1));
        body->Draw(s);
    }
}
std::string HingeConstraint::typeName() { return "Hinge"; }

#pragma endregion

#pragma region FixedDistanceConstraint

FixedDistanceConstraint::FixedDistanceConstraint(RigidBody* rb1, RigidBody* rb2, float distance) {
    first = rb1;
    second = rb2;
    d = distance;
}
void FixedDistanceConstraint::buildJacobian() {
    Jacobian.resize(1, 12);
    Jacobian.setZero();
    glm::vec3 norm = glm::normalize(first->position - second->position);
    for (size_t i = 0; i < 3; i++) {
        Jacobian(0, i) = norm[i];
        Jacobian(0, i + 6) = - norm[i];
    }
}
void FixedDistanceConstraint::gui(int index) {
    ImGui::SliderFloat(("Distance" + std::to_string(index)).c_str(), &d, 0, 100);
    
    Constraint::gui(index);
}
void FixedDistanceConstraint::solve(float dt) {
    Eigen::VectorXf velocity = buildVelocityVector();
    updateMassInverse();
    buildJacobian();

    Eigen::VectorXf biasTerm(1);
    biasTerm(0) = (glm::length(first->position - second->position) - d) * beta / dt;

    Eigen::VectorXf dv = generalDVSolver(invM, Jacobian, velocity, biasTerm, gamma/dt);

    addDeltaVelocities(first, second, dv);
}
std::string FixedDistanceConstraint::typeName() { return "FixedDistance"; }



#pragma endregion

#pragma region DistanceConstraint
DistanceConstraint::DistanceConstraint(RigidBody* rb1, RigidBody* rb2, 
                                    float minDistance, float maxDistance) {
    this->type = constraintType::distance;
    minD = minDistance;
    maxD = maxDistance;
    this->first = rb1;
    this->second = rb2;
}

void DistanceConstraint::solve(float dt) {
    if (!check()) return;

    float dist = glm::distance(first->position, second->position);

    float limit = - minD + dist;
    if (dist > maxD) limit = dist - maxD;

    if (first->movable) {
        glm::vec3 c = limit * glm::normalize(first->position - second->position);
        glm::vec3 v = (- beta / dt) * c;
        v *= 0.9;
        first->position += v * dt;
        first->velocity += v;
    }
    if (second->movable) {
        glm::vec3 c = limit * glm::normalize(first->position - second->position);
        c = -c;
        glm::vec3 v = (- beta / dt) * c;
        v *= 0.9;
        second->position += v * dt;
        second->velocity += v;
    }
}
bool DistanceConstraint::check() {
    float d = glm::distance(first->position, second->position);
    return d <= maxD && d >= minD;
}
void DistanceConstraint::gui(int index) {
    ImGui::SliderFloat("min", &minD, 0, maxD);
    ImGui::SliderFloat("max", &maxD, minD, 100);
}
std::string DistanceConstraint::typeName() { return "Distance"; }


#pragma endregion 

#pragma region GenericConstraint

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


GenericConstraint::GenericConstraint(RigidBody* fst, RigidBody* snd) {
    this->type = constraintType::generic;
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

    glm::vec3 vlin = 0.9f * (- beta / dt) * dist * lin;

    // Rotation

    /*
    second->position += vlin * dt;
    second->velocity += vlin;

    Eigen::VectorXf v_snd, bias;
    v_snd.resize(3);
    bias.resize(1);

    glm::vec3 rot = glm::eulerAngles(glm::inverse(first->rotation) - second->rotation);

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
std::string GenericConstraint::typeName() { return "Generic"; }


#pragma endregion
