//
// Created by rhs0266 on 2/21/19.
//

#ifndef GUI_GLPRIMITIVEFUNCTIONS_HPP
#define GUI_GLPRIMITIVEFUNCTIONS_HPP

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <assimp/cimport.h>
#include <assimp/scene.h>
namespace GUI
{
// WARNING: Total coordinates are locally represented without additional comment.


/// p0, p1 are global coordinates of center of capsule's bottoms, thickness is radius of hemi-sphere.
void drawCapsule(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double thickness,
                 const Eigen::Vector3d &color = Eigen::Vector3d(0.8, 0.8, 0.8));

/// r is radius.
void drawSphere(double r);

/// size contains length of each edge.
void drawCube(const Eigen::Vector3d& size);

/// p0, p1, p2, p3 are vertices of tetrahedron.
void drawTetrahedron(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8));

/// p0, p1, p2 are vertices of triangle.
void drawTriangle(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8));

/// p0, p1 are end points of line.
void drawLine(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8));

/// p0 is point.
void drawPoint(const Eigen::Vector3d& p0,const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8));

/// start at _pt, pointing _dir
void drawArrow3D(const Eigen::Vector3d& _pt, const Eigen::Vector3d& _dir,
                 const double _length, const double _thickness,const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8),
                 const double _arrowThickness = -1);

/// p0, p1, p2 are control points of bezier curve.
void drawBezierCurve(
		const Eigen::Vector3d& p0,
		const Eigen::Vector3d& p1,
		const Eigen::Vector3d& p2,
		const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8));

/// draw mesh after _scale scaling.
void drawMesh(const Eigen::Vector3d& _scale, const aiScene* _mesh,const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8));

/// _x, _y are place of string, _bigFont is flag for bigger font.
void drawStringOnScreen(float _x, float _y, const std::string& _s,bool _bigFont,const Eigen::Vector3d& color=Eigen::Vector3d(0.8,0.8,0.8));

};



#endif //GUI_GLPRIMITIVEFUNCTIONS_HPP
