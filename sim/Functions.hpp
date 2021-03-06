//
// Created by ryu on 6/14/19.
//

#ifndef EDITOR_FUNCTIONS_HPP
#define EDITOR_FUNCTIONS_HPP

#include "dart/dart.hpp"

//#include <boost/python.hpp>
//#include <boost/python/numpy.hpp>
//namespace p = boost::python;
//namespace np = boost::python::numpy;

/*//always return 1-dim array
np::ndarray toNumPyArray(const std::vector<float>& val);
//always return 1-dim array
np::ndarray toNumPyArray(const std::vector<double>& val);
//always return 1-dim array
np::ndarray toNumPyArray(const std::vector<bool>& val);
//always return 1-dim array
np::ndarray toNumPyArray(const Eigen::VectorXd& vec);
//always return 2-dim array
np::ndarray toNumPyArray(const Eigen::MatrixXd& matrix);
//always return 2-dim array
np::ndarray toNumPyArray(const std::vector<Eigen::VectorXd>& matrix);
//always return 2-dim array
np::ndarray toNumPyArray(const std::vector<std::vector<double>>& matrix);
Eigen::VectorXd toEigenVector(const np::ndarray& array,int n);
Eigen::VectorXd toEigenVector(const p::object& array,int n);
Eigen::MatrixXd toEigenMatrix(const np::ndarray& array,int n,int m);*/
// Utilities
std::vector<double> split_to_double(const std::string& input, int num);
std::vector<double> split_to_double(const std::string& input);
Eigen::Vector3d string_to_vector3d(const std::string& input);
Eigen::VectorXd string_to_vectorXd(const std::string& input, int n);
Eigen::VectorXd string_to_vectorXd(const std::string& input);
Eigen::Matrix3d string_to_matrix3d(const std::string& input);

double exp_of_squared(const Eigen::VectorXd& vec,double sigma = 1.0);
double exp_of_squared(const Eigen::Vector3d& vec,double sigma = 1.0);
double exp_of_squared(const Eigen::MatrixXd& mat,double sigma = 1.0);
std::pair<int, double> maxCoeff(const Eigen::VectorXd& in);

double radianClamp(double input);

Eigen::Quaterniond DARTPositionToQuaternion(Eigen::Vector3d in);
Eigen::Vector3d QuaternionToDARTPosition(const Eigen::Quaterniond& in);
void QuaternionNormalize(Eigen::Quaterniond& in);

void SetBodyNodeColors(dart::dynamics::BodyNode* bn, const Eigen::Vector3d& color);
void SetSkeletonColor(const dart::dynamics::SkeletonPtr& object, const Eigen::Vector3d& color);
void SetSkeletonColor(const dart::dynamics::SkeletonPtr& object, const Eigen::Vector4d& color);

void EditBVH(std::string& path);
Eigen::Quaterniond GetYRotation(Eigen::Quaterniond q);

Eigen::Vector3d Proj(const Eigen::Vector3d& u,const Eigen::Vector3d& v);
Eigen::Isometry3d Orthonormalize(const Eigen::Isometry3d& T_old);

Eigen::Vector3d changeToRNNPos(Eigen::Vector3d pos);
Eigen::Isometry3d getJointTransform(dart::dynamics::SkeletonPtr skel, std::string bodyname);
Eigen::Vector4d rootDecomposition(dart::dynamics::SkeletonPtr skel, Eigen::VectorXd positions);
Eigen::VectorXd solveIK(dart::dynamics::SkeletonPtr skel, const std::string& bodyname, const Eigen::Vector3d& delta,  const Eigen::Vector3d& offset);
Eigen::VectorXd solveMCIK(dart::dynamics::SkeletonPtr skel, const std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d>>& constraints);


#endif //EDITOR_FUNCTIONS_HPP
