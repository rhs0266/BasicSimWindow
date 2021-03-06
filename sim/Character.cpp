//
// Created by ryu on 7/15/19.
//

#include <tinyxml.h>
#include "Character.hpp"
#include "SkeletonBuilder.hpp"
#include "Functions.hpp"

using namespace std;
using namespace Eigen;
using namespace dart::dynamics;

Character::Character(const std::string &skeletonPath) {
    this->mSkeleton = SkeletonBuilder::BuildFromFile(skeletonPath);
    this->mCharacterPath = skeletonPath;
    this->loadBVHMap(skeletonPath);
    bvh = BVH();
}

const dart::dynamics::SkeletonPtr &Character::getSkeleton() {
    return this->mSkeleton;
}

const std::string &Character::getCharacterPath() {
    return this->mCharacterPath;
}

void Character::setPDParams(double kp, double kv) {
    int dof = this->mSkeleton->getNumDofs();
    this->setPDParams(VectorXd::Constant(dof, kp),VectorXd::Constant(dof, kv));
}

void Character::setPDParams(const Eigen::VectorXd &kp, const Eigen::VectorXd &kv) {
    this->mKp = kp;
    this->mKv = kv;
    this->mKp.block<6,1>(0,0).setZero();
    this->mKv.block<6,1>(0,0).setZero();
    this->mKpDefault = this->mKp;
    this->mKvDefault = this->mKv;

}

void Character::setPDParams(const Eigen::VectorXd &k) {
    int dof = this->mSkeleton->getNumDofs();
    this->mKp = VectorXd::Zero(dof);
    this->mKv = VectorXd::Zero(dof);
    this->mKp.segment(6, dof-6) = k.segment(0, dof-6).array() * this->mKpDefault.segment(6, dof-6).array();
    this->mKv.segment(6, dof-6) = k.segment(dof-6, dof-6).array() * this->mKpDefault.segment(6, dof-6).array();
}

void Character::applyForces(const Eigen::VectorXd &forces) {
    this->mSkeleton->setForces(forces);
}

Eigen::VectorXd Character::GetPDForces(const Eigen::VectorXd &pDesired, const Eigen::VectorXd &vDesired) {
    auto& skel = this->mSkeleton;
    int dof = skel->getNumDofs();
    VectorXd dq = skel->getVelocities();
    double dt = skel->getTimeStep();

    VectorXd pDiff(dof);
    pDiff = this->mSkeleton->getPositionDifferences(dq*dt, -1.0 * this->mSkeleton->getPositions()); // predicting accumulated position.
    pDiff = this->mSkeleton->getPositionDifferences(pDesired, pDiff); // Calculating error of position.
    
    pDiff.segment<6>(0) = VectorXd::Zero(6);

    /**
     * Below code clamps radian angles into [-pi, pi). We Assume that every joints except root are ball joint.
     */
    for (int i=6;i<dof;i+=3){
        Vector3d v = pDiff.segment<3>(i);
        double angle = v.norm();

        if (angle > 1e-8){
            angle = radianClamp(angle);
            Vector3d axis = v.normalized();

            pDiff.segment<3>(i) = angle * axis;
        }else{
            pDiff.segment<3>(i) = v;
        }
    }
    VectorXd vDiff(dof);
    vDiff = vDesired - skel->getVelocities();

    VectorXd tau = mKp.cwiseProduct(pDiff) + mKv.cwiseProduct(vDiff);

    tau.segment<6>(0) = VectorXd::Zero(6);
    return tau;
}

Eigen::VectorXd Character::GetSPDForces(const Eigen::VectorXd &pDesired, const Eigen::VectorXd &vDesired) {
    auto& skel = mSkeleton;
    int dof = skel->getNumDofs();
    VectorXd q = skel->getPositions(), dq = skel->getVelocities();
    double dt = skel->getTimeStep();
    MatrixXd M_inv = (skel->getMassMatrix() + MatrixXd(dt*mKv.asDiagonal())).inverse();

    VectorXd pDiff = VectorXd::Zero(q.rows()), vDiff = VectorXd::Zero(q.rows());

    for (int i=6;i<dof;i++){
        Quaterniond q_s = DARTPositionToQuaternion(q.segment<3>(i));
        Quaterniond dq_s = DARTPositionToQuaternion(dt * dq.segment<3>(i));
        Quaterniond q_d_s = DARTPositionToQuaternion(pDesired.segment<3>(i));
        Quaterniond p_d_s = q_d_s.inverse() * q_s * dq_s;

        Vector3d v = QuaternionToDARTPosition(p_d_s);
        double angle = v.norm();
        if (angle > 1e-8){
            angle = radianClamp(angle);
            Vector3d axis = v.normalized();

            pDiff.segment<3>(i) = angle * axis;
        }else{
            pDiff.segment<3>(i) = v;
        }
    }

    pDiff = -mKp.cwiseProduct(pDiff);
    vDiff = -mKv.cwiseProduct(dq - vDesired);
    VectorXd qdDot = M_inv * (-skel->getCoriolisAndGravityForces() + pDiff + vDiff + skel->getConstraintForces());

    VectorXd tau = pDiff + vDiff - dt * mKv.cwiseProduct(qdDot);
    tau.segment<6>(0) = VectorXd::Zero(6);

    return tau;
}

void Character::loadBVH(const std::string& BVHPath){
    bvh.parse(BVHPath);
    initBVH();
}

void Character::loadBVHMap(const std::string &skeletonPath) {
    TiXmlDocument doc;
    if (!doc.LoadFile(skeletonPath)){
        cout << "Can't open file: " << skeletonPath << endl;
        return;
    }

    TiXmlElement *skelDoc = doc.FirstChildElement("Skeleton");

    string skelName = skelDoc->Attribute("name");

    for (TiXmlElement *body = skelDoc->FirstChildElement("Joint"); body!=nullptr; body=body->NextSiblingElement("Joint")){
        string name = body->Attribute("name");
        if (body->Attribute("bvh") != nullptr)
            mBVHMap.insert(make_pair(name, body->Attribute("bvh")));
    }

}

void Character::initBVH() {
    for (const auto ss: mBVHMap)
        bvh.addMapping(ss.first, ss.second);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Character::getTargetPosAndVelFromBVH(double t) {
    // TODO
    return std::pair<Eigen::VectorXd, Eigen::VectorXd>();
}

Eigen::VectorXd Character::getTargetPos(double t) {
    int dof = this->mSkeleton->getNumDofs();
    VectorXd p = VectorXd::Zero(dof);

    // Set p
    bvh.setMotion(t);
    for (auto ss: mBVHMap){
        BodyNode* bn = this->mSkeleton->getBodyNode(ss.first);
        Matrix3d R = bvh.getBodyNodeRotation(ss.first);

        Joint* jn = bn->getParentJoint();
        Vector3d a = BallJoint::convertToPositions(R);
        int jnIdx = jn->getIndexInSkeleton(0);

        a = QuaternionToDARTPosition(DARTPositionToQuaternion(a));

        if (dynamic_cast<BallJoint*>(jn) != nullptr || dynamic_cast<FreeJoint*>(jn) != nullptr){
            p.block<3,1>(jnIdx,0) = a;
        }else if (dynamic_cast<RevoluteJoint*>(jn) != nullptr) {
            p[jnIdx] = a[0];
            if (p[jnIdx] > M_PI)
                p[jnIdx] -= 2 * M_PI;
            else if (p[jnIdx] < -M_PI)
                p[jnIdx] += 2 * M_PI;
        }
    }
    p.block<3,1>(3,0) = bvh.getRootCOM();
    return p;
}

void Character::followBVH(double t) {
    while (t > bvh.getMaxTime())
        t -= bvh.getMaxTime();
    this->mSkeleton->setPositions(this->getTargetPos(t));
}
