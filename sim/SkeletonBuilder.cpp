//
// Created by ryu on 6/13/19.
//

#include "SkeletonBuilder.hpp"
#include <tinyxml.h>
#include "Functions.hpp"

using namespace std;
using namespace Eigen;
using namespace dart::dynamics;


dart::dynamics::SkeletonPtr SkeletonBuilder::BuildFromFile(const std::string &filename) {

    TiXmlDocument doc;
    if (!doc.LoadFile(filename)){
        cout << "Can't Open File: " << filename << endl;
        return nullptr;
    }

    TiXmlElement *skelDoc = doc.FirstChildElement("Skeleton");

    std::string skelName = skelDoc->Attribute("name");
    SkeletonPtr skel = Skeleton::create(skelName);

    for (TiXmlElement *jointElem = skelDoc->FirstChildElement("Joint"); jointElem!= nullptr; jointElem = jointElem->NextSiblingElement("Joint")){
        string jointType = jointElem->Attribute("type");
        string name = jointElem->Attribute("name");
        string parentName = jointElem->Attribute("parent_name");
        BodyNode *parent;
        if (!parentName.compare("None"))
            parent = nullptr;
        else
            parent = skel->getBodyNode(parentName);
        Vector3d size = string_to_vector3d(jointElem->Attribute("size"));

        TiXmlElement *bodyPosElem = jointElem->FirstChildElement("BodyPosition");
        Isometry3d bodyPos;
        bodyPos.setIdentity();
        if (bodyPosElem->Attribute("linear") != nullptr)
            bodyPos.linear() = string_to_matrix3d(bodyPosElem->Attribute("linear"));
        if (bodyPosElem->Attribute("translation") != nullptr)
            bodyPos.translation() = string_to_vector3d(bodyPosElem->Attribute("translation"));

        TiXmlElement *jointPosElem = jointElem->FirstChildElement("JointPosition");
        Isometry3d jointPos;
        jointPos.setIdentity();
        if (jointPosElem->Attribute("linear") != nullptr)
            jointPos.linear() = string_to_matrix3d(bodyPosElem->Attribute("linear"));
        if (jointPosElem->Attribute("translation") != nullptr)
            jointPos.translation() = string_to_vector3d(bodyPosElem->Attribute("translation"));
        jointPos = Orthonormalize(jointPos);

        double mass = atof(jointElem->Attribute("mass"));

        bool contact = true;

        Vector3d axis;
        if (jointElem->Attribute("axis") != nullptr)
            axis = string_to_vector3d(jointElem->Attribute("axis"));

        JOINT_TYPE _jointType;
        if (!jointType.compare("FreeJoint")) _jointType = JOINT_TYPE::Free;
        if (!jointType.compare("BallJoint")) _jointType = JOINT_TYPE::Ball;
        if (!jointType.compare("RevoluteJoint")) _jointType = JOINT_TYPE::Revolute;
        if (!jointType.compare("PrismaticJoint")) _jointType = JOINT_TYPE::Prismatic;
        if (!jointType.compare("WeldJoint")) _jointType = JOINT_TYPE::Weld;

        MakeJoint(_jointType, name, skel, parent, size, jointPos, bodyPos, mass, contact, axis);

    }

    return skel;
}

dart::dynamics::BodyNode *SkeletonBuilder::MakeJoint(const JOINT_TYPE jointType, const std::string &bodyName,
                                                           const dart::dynamics::SkeletonPtr &targetSkel,
                                                           dart::dynamics::BodyNode *const parent,
                                                           const Eigen::Vector3d &size,
                                                           const Eigen::Isometry3d &jointPosition,
                                                           const Eigen::Isometry3d &bodyPosition,
                                                           double mass,
                                                           bool contact,
                                                           const Eigen::Vector3d axis) {

    ShapePtr shape = shared_ptr<BoxShape>(new BoxShape(size));

    Inertia inertia;
    inertia.setMass(mass);
    inertia.setMoment(shape->computeInertia(mass));

    BodyNode *bn;

    if (jointType == JOINT_TYPE::Free) {
        FreeJoint::Properties props;
        props.mName = bodyName;
        props.mT_ParentBodyToJoint = bodyPosition;
        bn = targetSkel->createJointAndBodyNodePair<FreeJoint>(parent, props,
                                                               BodyNode::AspectProperties(bodyName)).second;
    }else if (jointType == JOINT_TYPE::Ball){
        BallJoint::Properties props;
        props.mName = bodyName;
        props.mT_ChildBodyToJoint = bodyPosition.inverse() * jointPosition;
        props.mT_ParentBodyToJoint = parent->getTransform().inverse() * jointPosition;
        bn = targetSkel->createJointAndBodyNodePair<BallJoint>(parent, props,
                                                               BodyNode::AspectProperties(bodyName)).second;
    }
    else if (jointType == JOINT_TYPE::Revolute) {
        RevoluteJoint::Properties props;
        props.mName = bodyName;
        props.mAxis = axis;
        props.mT_ChildBodyToJoint = bodyPosition.inverse() * jointPosition;
        props.mT_ParentBodyToJoint = parent->getTransform().inverse() * jointPosition;
        bn = targetSkel->createJointAndBodyNodePair<RevoluteJoint>(parent, props,
                                                                   BodyNode::AspectProperties(bodyName)).second;
    }else if (jointType == JOINT_TYPE::Prismatic) {
        PrismaticJoint::Properties props;
        props.mName = bodyName;
        props.mAxis = axis;
        props.mT_ChildBodyToJoint = bodyPosition.inverse() * jointPosition;
        props.mT_ParentBodyToJoint = parent->getTransform().inverse() * jointPosition;
        bn = targetSkel->createJointAndBodyNodePair<RevoluteJoint>(parent, props,
                                                                   BodyNode::AspectProperties(bodyName)).second;
    }else if (jointType == JOINT_TYPE::Weld){
        BallJoint::Properties props;
        props.mName = bodyName;
        props.mT_ChildBodyToJoint = bodyPosition.inverse() * jointPosition;
        props.mT_ParentBodyToJoint = parent->getTransform().inverse() * jointPosition;
        bn = targetSkel->createJointAndBodyNodePair<BallJoint>(parent, props,
                                                               BodyNode::AspectProperties(bodyName)).second;
    }

    if (jointType == JOINT_TYPE::Ball || jointType == JOINT_TYPE::Revolute){
        JointPtr jn = bn->getParentJoint();
        for (int i=0;i<jn->getNumDofs();i++)
            jn->getDof(i)->setDampingCoefficient(0.05);
    }

    if (contact){
        bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
    }else{
        bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
    }
    bn->setInertia(inertia);
    return nullptr;
}
