//
// Created by ryu on 7/15/19.
//

#ifndef WINDOW_CHARACTER_HPP
#define WINDOW_CHARACTER_HPP

#include "dart/dart.hpp"
#include "BVH.hpp"

/**
 * @class Character
 * @details Including character skeleton, state getter, state setter, and optional BVH information.
 */
class Character {
public:
    /// Constructor.
    Character(){}
    /// Constructor with skeleton path.
    Character(const std::string& skeletonPath);

    /// Getter for skeleton pointer.
    const dart::dynamics::SkeletonPtr& getSkeleton();

    /// Getter for character path.
    const std::string& getCharacterPath();

    /// Setter for PD parameters.
    void setPDParams(double kp, double kv);

    /// Setter for PD parameters.
    void setPDParams(const Eigen::VectorXd& kp, const Eigen::VectorXd& kv);

    /**
     * @breif Setter for PD parameters.
     * @param k Concatenated array which contains k_p, k_v values of joints except root.
     */
    void setPDParams(const Eigen::VectorXd& k);

    /// Apply joint torque to skeleton.
    void applyForces(const Eigen::VectorXd& forces);

    /// Getter for PD forces exert by skeleton.
    Eigen::VectorXd GetPDForces(const Eigen::VectorXd& pDesired, const Eigen::VectorXd& vDesired);

    /// Getter for SPD forces exert by skeleton.
    Eigen::VectorXd GetSPDForces(const Eigen::VectorXd& pDesired, const Eigen::VectorXd& vDesired);

    /// Loding BVH mappign from BVH file.
    void loadBVHMap(const std::string& BVHPath);

    /// Initializing BVH.
    void initBVH(BVH* bvh);

    /// Getter for approximated positions and velocities from bvh file.
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getTargetPosAndVelFromBVH(BVH* bvh, double t);

    /// Getter for approximated positions from bvh file.
    Eigen::VectorXd getTargetPos(BVH* bvh, double t);

protected:
    std::string mCharacterPath;
    dart::dynamics::SkeletonPtr mSkeleton;

    std::map<std::string, std::string> mBVHMap;
    Eigen::VectorXd mKp, mKv;
    Eigen::VectorXd mKpDefault, mKvDefault;

};


#endif //WINDOW_CHARACTER_HPP