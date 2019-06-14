//
// Created by ryu on 6/13/19.
//

#ifndef EDITOR_SKELETONBUILDER_HPP
#define EDITOR_SKELETONBUILDER_HPP

#include "dart/dart.hpp"

enum JOINT_TYPE{
    Free,
    Ball,
    Revolute,
    Prismatic,
    Weld
};

class SkeletonBuilder
{
public:
    static dart::dynamics::SkeletonPtr BuildFromFile(const std::string& filename);
    //static void WriteSkeleton(std::string filename, dart::dynamics::SkeletonPtr& skel);

    static dart::dynamics::BodyNode* MakeJoint(
            const JOINT_TYPE jointType,
            const std::string& bodyName,
            const dart::dynamics::SkeletonPtr& targetSkel,
            dart::dynamics::BodyNode* const parent,
            const Eigen::Vector3d& size,
            const Eigen::Isometry3d& jointPosition,
            const Eigen::Isometry3d& bodyPosition,
            double mass,
            bool contact,
            const Eigen::Vector3d axis);

};


#endif //EDITOR_SKELETONBUILDER_HPP
