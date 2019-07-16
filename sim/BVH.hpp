//
// Created by ryu on 7/15/19.
//

#ifndef WINDOW_BVH_HPP
#define WINDOW_BVH_HPP

#include <Eigen/Core>
#include <string>
#include <fstream>
#include <map>
#include <utility>
#include <vector>
#include <initializer_list>

class BVHNode
{
public:
    enum CHANNEL
    {
        Xpos=0,
        Ypos=1,
        Zpos=2,
        Xrot=3,
        Yrot=4,
        Zrot=5
    };
    static std::map<std::string,BVHNode::CHANNEL> CHANNEL_NAME;


    BVHNode(const std::string& name,BVHNode* parent);
    void setChannel(int c_offset,std::vector<std::string>& c_name);
    void set(const Eigen::VectorXd& m_t);
    void set(const Eigen::Matrix3d& R_t);
    Eigen::Matrix3d get();

    void addChild(BVHNode* child);
    BVHNode* getNode(const std::string& name);
private:
    BVHNode* mParent;
    std::vector<BVHNode*> mChildren;

    Eigen::Matrix3d mR;
    std::string mName;

    int mChannelOffset;
    int mNumChannels;
    std::vector<BVHNode::CHANNEL> mChannel;
};
class BVH
{

public:
    BVH();
    void addMapping(const std::string& body_node,const std::string& bvh_node);

    void setMotion(double t);
    const Eigen::Vector3d& getRootCOM(){return mRootCOM;}
    Eigen::Matrix3d getBodyNodeRotation(const std::string& bodyNode);

    double getMaxTime(){return mNumTotalFrames*mTimeStep;}
    double getTimeStep(){return mTimeStep;}
    void parse(const std::string& file);
private:
    std::vector<Eigen::VectorXd> mMotions;
    std::map<std::string,BVHNode*> mMap;
    double mTimeStep;
    int mNumTotalChannels;
    int mNumTotalFrames;

    BVHNode* mRoot;
    Eigen::Vector3d mRootCOM;
    Eigen::Vector3d mRootCOMOffset;
    Eigen::VectorXd mMotionDiff;

    int numInterpolate;
    BVHNode* readHierarchy(BVHNode* parent,const std::string& name,int& channel_offset,std::ifstream& is);
};


#endif //WINDOW_BVH_HPP
