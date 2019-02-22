//
// Created by rhs0266 on 2/22/19.
//

#include "GLComplexFunctions.hpp"

using namespace Eigen;
using namespace dart::dynamics;

void GUI::DrawShape(const Eigen::Isometry3d &T, const dart::dynamics::Shape *shape, const Eigen::Vector3d &color) {
	glPushMatrix();

	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(color[0], color[1], color[2]);

	glMultMatrixd(T.data());

	if (shape->is<SphereShape>()){
		const auto* sphere = dynamic_cast<const SphereShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		drawSphere(sphere->getRadius());
	}


	glPopMatrix();
}
