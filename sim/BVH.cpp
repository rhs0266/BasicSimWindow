//
// Created by ryu on 7/15/19.
//

#include "BVH.hpp"
#include <iostream>
#include <Eigen/Geometry>
#include "dart/dart.hpp"

using namespace Eigen;
using namespace std;

Matrix3d
R_x(double x) {
    double cosa = cos(x * M_PI / 180.0);
    double sina = sin(x * M_PI / 180.0);
    Matrix3d R;
    R << 1, 0, 0,
            0, cosa, -sina,
            0, sina, cosa;
    return R;
}
Matrix3d R_y(double y) {
    double cosa = cos(y * M_PI / 180.0);
    double sina = sin(y * M_PI / 180.0);
    Matrix3d R;
    R << cosa, 0, sina,
            0, 1, 0,
            -sina, 0, cosa;
    return R;
}
Matrix3d R_z(double z) {
    double cosa = cos(z * M_PI / 180.0);
    double sina = sin(z * M_PI / 180.0);
    Matrix3d R;
    R << cosa, -sina, 0,
            sina, cosa, 0,
            0, 0, 1;
    return R;
}
BVHNode::
BVHNode(const string& name,BVHNode* parent)
        :mParent(parent),mName(name)
{

}
void
BVHNode::
setChannel(int c_offset,vector<string>& c_name) {
    mChannelOffset = c_offset;
    mNumChannels = c_name.size();
    for (const auto &cn : c_name)
        mChannel.push_back(CHANNEL_NAME[cn]);
}
void
BVHNode::
set(const VectorXd& m_t) {
    mR.setIdentity();
    for (int i = 0; i < mNumChannels; i++) {
        switch (mChannel[i]) {
            case Xpos:
                break;
            case Ypos:
                break;
            case Zpos:
                break;
            case Xrot:
                mR = mR * R_x(m_t[mChannelOffset + i]);
                break;
            case Yrot:
                mR = mR * R_y(m_t[mChannelOffset + i]);
                break;
            case Zrot:
                mR = mR * R_z(m_t[mChannelOffset + i]);
                break;
            default:
                break;
        }
    }
}
void
BVHNode::
set(const Matrix3d& R_t) {
    mR = R_t;
}
Matrix3d
BVHNode::
get() {
    return mR;
}

void
BVHNode::
addChild(BVHNode* child) {
    mChildren.push_back(child);
}
BVHNode*
BVHNode::
getNode(const string& name) {
    if (!mName.compare(name))
        return this;

    for (auto &c : mChildren) {
        BVHNode *bn = c->getNode(name);
        if (bn != nullptr)
            return bn;
    }

    return nullptr;
}


BVH::
BVH() {

}

void
BVH::
addMapping(const string& body_node,const string& bvh_node) {
    BVHNode *b = mRoot->getNode(bvh_node);
    mMap.insert(make_pair(body_node, b));
}

void
BVH::
setMotion(double t) {
    int k = ((int) floor(t / mTimeStep)) % mNumTotalFrames;
    int k1 = min(k + 1, mNumTotalFrames - 1);
    double dt = (t / mTimeStep - floor(t / mTimeStep));
    vector<Matrix3d> R_0, R_1;

    //R0
    for (auto &bn: mMap)
        bn.second->set(mMotions[k]);

    for (auto &bn:mMap)
        R_0.push_back(bn.second->get());

    //R1
    for (auto &bn: mMap)
        bn.second->set(mMotions[k1]);

    for (auto &bn:mMap)
        R_1.push_back(bn.second->get());

    //slerp
    int count = 0;
    for (auto &bn:mMap) {
        Matrix3d exp_w = (R_0[count].transpose()) * R_1[count];
        AngleAxisd w(exp_w);
        Matrix3d R_t = R_0[count] * dart::math::expMapRot(dt * w.angle() * w.axis());
        bn.second->set(R_t);
        count++;
    }

    Vector3d root_k = mMotions[k].segment<3>(0);
    Vector3d root_k1 = mMotions[k1].segment<3>(0);

    mRootCOM.setZero();
    mRootCOM = (root_k * (1 - dt) + root_k1 * dt - mRootCOMOffset) * 0.01;
}

Matrix3d
BVH::
getBodyNodeRotation(const string& bodyNode) {
    return mMap[bodyNode]->get();
}

void
BVH::
parse(const string& file) {
    ifstream is(file);

    char buffer[256];

    if (!is) {
        cout << "Can't Open File" << endl;
        return;
    }
    while (is >> buffer) {
        if (!strcmp(buffer, "HIERARCHY")) {
            is >> buffer;//Root
            is >> buffer;//Name
            int c_offset = 0;
            mRoot = readHierarchy(nullptr, buffer, c_offset, is);
            mNumTotalChannels = c_offset;
        } else if (!strcmp(buffer, "MOTION")) {
            is >> buffer; //Frames:
            is >> buffer; //num_frames
            mNumTotalFrames = atoi(buffer);
            is >> buffer; //Frame
            is >> buffer; //Time:
            is >> buffer; //time step
            mTimeStep = atof(buffer);
            mMotions.resize(mNumTotalFrames);
            for (auto &m_t : mMotions)
                m_t = VectorXd::Zero(mNumTotalChannels);
            double val;
            for (int i = 0; i < mNumTotalFrames; i++) {
                for (int j = 0; j < mNumTotalChannels; j++) {
                    is >> val;
                    mMotions[i][j] = val;
                }
            }
        }
    }
    is.close();
    mMotionDiff = mMotions[0] - mMotions.back();
    mMotionDiff[1] = 0.0;

    // for debug
    mMotionDiff.setZero();
    numInterpolate = mMotions.size() / 10;
}

BVHNode*
BVH::
readHierarchy(BVHNode* parent,const string& name,int& channel_offset,ifstream& is) {
    char buffer[256];
    double offset[3];
    vector<string> c_name;

    BVHNode *new_node = new BVHNode(name, parent);

    is >> buffer; //{

    while (is >> buffer) {
        if (!strcmp(buffer, "}"))
            break;
        if (!strcmp(buffer, "OFFSET")) {
            //Ignore
            double x, y, z;

            is >> x;
            is >> y;
            is >> z;
            if (parent == nullptr) {
                mRootCOMOffset[0] = x;
                mRootCOMOffset[1] = y;
                mRootCOMOffset[2] = z;
            }
        } else if (!strcmp(buffer, "CHANNELS")) {

            is >> buffer;
            int n;
            n = atoi(buffer);

            for (int i = 0; i < n; i++) {
                is >> buffer;
                c_name.push_back(string(buffer));
            }

            new_node->setChannel(channel_offset, c_name);
            channel_offset += n;
        } else if (!strcmp(buffer, "JOINT")) {
            is >> buffer;
            BVHNode *child = readHierarchy(new_node, string(buffer), channel_offset, is);
            new_node->addChild(child);
        } else if (!strcmp(buffer, "End")) {
            is >> buffer;
            BVHNode *child = readHierarchy(new_node, string("EndEffector"), channel_offset, is);
            new_node->addChild(child);
        }
    }

    return new_node;
}

map<string,BVHNode::CHANNEL> BVHNode::CHANNEL_NAME =
        {
                {"Xposition",Xpos},
                {"XPOSITION",Xpos},
                {"Yposition",Ypos},
                {"YPOSITION",Ypos},
                {"Zposition",Zpos},
                {"ZPOSITION",Zpos},
                {"Xrotation",Xrot},
                {"XROTATION",Xrot},
                {"Yrotation",Yrot},
                {"YROTATION",Yrot},
                {"Zrotation",Zrot},
                {"ZROTATION",Zrot}
        };
