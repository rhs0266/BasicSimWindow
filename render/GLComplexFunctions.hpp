//
// Created by rhs0266 on 2/22/19.
//

#ifndef GUI_GLCOMPLEXFUNCTIONS_HPP
#define GUI_GLCOMPLEXFUNCTIONS_HPP
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "GLPrimitiveFunctions.hpp"

typedef std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d> AnchorPoint;
namespace COLOR {

const Eigen::Vector3d muscleLine(153 / 255.0, 205 / 255.0, 255 / 255.0);
const Eigen::Vector3d muscleWaypoint(153 / 255.0, 255 / 255.0, 205 / 255.0);

}
namespace GUI
{
/*
void DrawSkeleton(
		const dart::dynamics::SkeletonPtr& skel, std::map<std::string, int> name2ID,
		const Eigen::Vector3d& color=Eigen::Vector3d(0.8,0.8,0.8), bool box = true);

void DrawSkeleton(
		const dart::dynamics::SkeletonPtr& skel,
		const Eigen::Vector3d& color=Eigen::Vector3d(0.8,0.8,0.8), bool box = true);

*/
void DrawShape(const Eigen::Isometry3d& T,
               const dart::dynamics::Shape* shape,
               const Eigen::Vector3d& color);

//void DrawMuscleWayPoints(std::shared_ptr<Muscle> muscle, GLUquadric* qobj, bool initialFlag = false, bool highlightFlag = false);
};

#endif //GUI_GLCOMPLEXFUNCTIONS_HPP
