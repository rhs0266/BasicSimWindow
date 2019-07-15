//
// Created by ryu on 6/13/19.
//

#ifndef EDITOR_SKELETONBUILDER_HPP
#define EDITOR_SKELETONBUILDER_HPP

#include "dart/dart.hpp"


/**
 * @enum Type of joint.
 */
enum JOINT_TYPE{
    Free,
    Ball,
    Revolute,
    Prismatic,
    Weld
};

/**
 * @class SkeletonBuilder
 * @brief Building skeleton object by using xml file which contains skeleton information.
 */

class SkeletonBuilder
{
public:

    /**
     * @param Filename name of xml file which contains skeleton information.
     * @return dart::dynamics::SkeletonPtr type object.
     */
    static dart::dynamics::SkeletonPtr BuildFromFile(const std::string& filename);
    //static void WriteSkeleton(std::string filename, dart::dynamics::SkeletonPtr& skel);

    /**
     * @param jointType Type of joint, one of JOINT_TYPE.
     * @param bodyName Name of body node.
     * @param targetSkel Skeleton pointer that will contains this new joint.
     * @param parent Hierarchical parent of this new joint.
     * @param size Basically, size of body node(both visual and collision aspect) which is a cube shape.
     * @param jointPosition Global coordinates of joint node.
     * @param bodyPosition Global coordinateso of body node.
     * @param mass Mass of body node.
     * @param contact Flag of contactness.
     * @param axis Axis information if joint type is either revolute or prismatic.
     * @return
     */
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
