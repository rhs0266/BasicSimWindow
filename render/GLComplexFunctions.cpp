//
// Created by rhs0266 on 2/22/19.
//

#include "GLComplexFunctions.hpp"

using namespace Eigen;
using namespace dart::dynamics;

void
GUI::
DrawSkeleton(
        const dart::dynamics::SkeletonPtr& skel,
        const Eigen::Vector3d& color, bool box)
{
    for(int i=0;i<skel->getNumBodyNodes();i++)
    {
        auto bn = skel->getBodyNode(i);
        auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();

        auto T = shapeNodes[0]->getTransform();
        DrawShape(T,shapeNodes[0]->getShape().get(),color);
    }
}

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
	}else if (shape->is<BoxShape>())
    {
        const auto* box = dynamic_cast<const BoxShape*>(shape);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
//        GUI::DrawRoundedBox(box->getSize(), 0.01);
         GUI::drawCube(box->getSize());
    }
//    else if(shape->is<MeshShape>())
//    {
//        auto* mesh = dynamic_cast<const MeshShape*>(shape);
//
//        // for(int i =0;i<16;i++)
//        // std::cout<<(*mesh->getMesh()->mRootNode->mTransformation)[i]<<" ";
//        GUI::DrawMesh(mesh->getScale(),mesh->getMesh());
//
//    }


	glPopMatrix();
}
