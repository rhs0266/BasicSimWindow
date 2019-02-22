//
// Created by rhs0266 on 2/19/19.
//

#include "Camera.hpp"
#include <GL/glut.h>
using namespace Eigen;

GUI::Camera::Camera():fovy(60.0),
					  lookAt(Vector3d(0,0.5,0)),
					  eye(Vector3d(0,0.5,2)),
					  up(Vector3d(0,1,0)) {

}

GUI::Camera::~Camera() {

}

void GUI::Camera::setCamera(const Vector3d &lookAt, const Vector3d &eye, const Vector3d &up) {
	this->lookAt = lookAt;
	this->eye = eye;
	this->up = up;
}

void GUI::Camera::applySetting() {
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const double size = 1.5;
	gluPerspective(fovy, (GLfloat)w / h, 0.01, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eye[0], eye[1], eye[2],
			  lookAt[0], lookAt[1], lookAt[2],
			  up[0], up[1], up[2]);
}

void GUI::Camera::zoomCamera(int x, int y, int prevX, int prevY) {
	double delta = prevY - y;
	fovy += delta / 20.0;
}

void GUI::Camera::rotateCamera(int x, int y, int prevX, int prevY) {
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	Vector3d prevPoint = getTrackballPoint(prevX, prevY, w, h);
	Vector3d curPoint = getTrackballPoint(x, y, w, h);
	Vector3d rotVec = curPoint.cross(prevPoint);

	if (rotVec.norm() < 1e-6) return;

	rotVec = unProject(rotVec);

	if (rotVec.norm() > 1e6) return;

	double cosT = curPoint.dot(prevPoint) / (curPoint.norm()*prevPoint.norm());
	double sinT = (curPoint.cross(prevPoint)).norm() / (curPoint.norm()*prevPoint.norm());

	double angle = -atan2(sinT, cosT);

	Vector3d n = this->lookAt - this->eye;
	n = rotateQ(n, rotVec, angle);
	this->up = rotateQ(this->up, rotVec, angle);
	this->eye = this->lookAt - n;
}

void GUI::Camera::translateCamera(int x, int y, int prevX, int prevY) {
	Vector3d delta(x-prevX, y-prevY, 0);
	delta = unProject(delta);
	if (delta.norm() > 1e6) return;
	delta *= (lookAt - eye).norm() / 1200.0;

	lookAt += delta, eye += delta;
}

Vector3d GUI::Camera::rotateQ(const Vector3d &target, const Vector3d &rotateVector, double angle) {

	Vector3d axis = rotateVector.normalized();

	Quaterniond rot(cos(angle/2.0), axis.x() * sin(angle/2.0), axis.y() * sin(angle/2.0), axis.z() * sin(angle/2.0));
	rot.normalize();
	Quaterniond tar(0, target.x(), target.y(), target.z());

	tar = rot.inverse() * tar * rot;

	return tar.vec();
}

Vector3d GUI::Camera::getTrackballPoint(int mouseX, int mouseY, int w, int h) {
	double rad = sqrt((double)(w*w+h*h)) / 2.0;
	double dx = (double)mouseX - (double)w / 2.0;
	double dy = (double)mouseY - (double)h / 2.0;
	double t = dx*dx + dy*dy;

	if (rad*rad - t <= 1e-5){
		return Vector3d(dx, dy, 0);
	}else{
		return Vector3d(dx, dy, sqrt(rad*rad - t));
	}
}

Vector3d GUI::Camera::unProject(const Vector3d &vec) {
	Vector3d n = lookAt - eye;
	if (n.norm() < 1e-6) return Vector3d(1e6,1e6,1e6);
	n.normalize();

	Vector3d v = up.cross(n);
	if (v.norm() < 1e-6) return Vector3d(1e6,1e6,1e6);
	v.normalize();

	Vector3d u = n.cross(v);
	if (u.norm() < 1e-6) return Vector3d(1e6,1e6,1e6);
	u.normalize();

	return vec.z()*n + vec.x()*v + vec.y()*u;
}
