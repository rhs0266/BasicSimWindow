//
// Created by ryu on 6/14/19.
//

#include "Functions.hpp"

using namespace std;
using namespace Eigen;

std::vector<double> split_to_double(const std::string& input, int num) {
    vector<double> res;
    string::size_type sz = 0, nsz = 0;
    for (int i=0;i<num;i++){
        res.push_back(stold(input.substr(sz), &nsz));
        sz += nsz;
    }
    return res;
}
std::vector<double> split_to_double(const std::string& input){
    vector<double> res;
    string::size_type sz = 0, nsz = 0;
    while (sz < input.length()){
        res.push_back(stold(input.substr(sz), &nsz));
        sz += nsz;
    }
    return res;
}

Eigen::Vector3d string_to_vector3d(const std::string& input){
    return string_to_vectorXd(input, 3);
}
Eigen::VectorXd string_to_vectorXd(const std::string& input, int n){
    vector<double> v = split_to_double(input, n);
    VectorXd res(n);
    for (int i=0;i<n;i++) res[i]=v[i];
    return res;
}
Eigen::VectorXd string_to_vectorXd(const std::string& input){
    vector<double> v = split_to_double(input);
    VectorXd res(v.size());
    for (int i=0;i<v.size();i++) res[i]=v[i];
    return res;
}
Eigen::Matrix3d string_to_matrix3d(const std::string& input){
    vector<double> v = split_to_double(input, 9);
    Matrix3d res;
    res << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return res;
}
Eigen::Vector3d Proj(const Eigen::Vector3d& u,const Eigen::Vector3d& v)
{
    Eigen::Vector3d proj;
    proj = u.dot(v)/u.dot(u)*u;
    return proj;
}
Eigen::Isometry3d Orthonormalize(const Eigen::Isometry3d& T_old)
{
    Eigen::Isometry3d T;
    T.translation() = T_old.translation();
    Eigen::Vector3d v0,v1,v2;
    Eigen::Vector3d u0,u1,u2;
    v0 = T_old.linear().col(0);
    v1 = T_old.linear().col(1);
    v2 = T_old.linear().col(2);

    u0 = v0;
    u1 = v1 - Proj(u0,v1);
    u2 = v2 - Proj(u0,v2) - Proj(u1,v2);

    u0.normalize();
    u1.normalize();
    u2.normalize();

    T.linear().col(0) = u0;
    T.linear().col(1) = u1;
    T.linear().col(2) = u2;
    return T;
}