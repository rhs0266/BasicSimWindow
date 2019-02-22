//
// Created by rhs0266 on 2/21/19.
//

#include "SimWindow.hpp"


#include <algorithm>
#include <fstream>
//#include <boost/filesystem.hpp> // for record
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <stdio.h>

using namespace std;
using namespace Eigen;
using namespace GUI;
using namespace dart::simulation;
using namespace dart::dynamics;

SimWindow::SimWindow() : GLUTWindow(), mIsRotate(true), mIsDrag(false), mIsPlay(false), mFrame(0), mDisplayRatio(1.0){
	mWorld = std::make_shared<dart::simulation::World>();
	mWorld->setGravity(Vector3d(0, -9.81, 0));
	mWorld->setTime(1.0 / 1000.0);
	mWorld->checkCollision();
}

SimWindow::~SimWindow() = default;

void SimWindow::display() {
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();
	mCamera->applySetting();
	glDisable(GL_LIGHTING);
	glColor3f(0,0,0);

	// Drawing in here.
	GUI::drawStringOnScreen(0.5, 0.5, "initial", true, Vector3d(0,0,0));

	glutSwapBuffers();
}

void SimWindow::keyboard(unsigned char key, int x, int y) {
	glutPostRedisplay();
}

void SimWindow::mouse(int button, int state, int x, int y) {
	glutPostRedisplay();
}

void SimWindow::motion(int x, int y) {
	glutPostRedisplay();
}

void SimWindow::reshape(int w, int h) {
	glViewport(0, 0, w, h);
	mCamera->applySetting();
}

void SimWindow::timer(int value) {
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, timerEvent, 1);
}
