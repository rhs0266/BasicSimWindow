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
#include "../sim/SkeletonBuilder.hpp"
#include "GLComplexFunctions.hpp"

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

	mWorld->addSkeleton(SkeletonBuilder::BuildFromFile("../character/box.xml"));
    mWorld->addSkeleton(SkeletonBuilder::BuildFromFile("../character/ground.xml"));
}

SimWindow::~SimWindow() = default;

void SimWindow::display() {
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();
	mCamera->applySetting();

    glEnable(GL_BLEND);

    DrawSkeleton(mWorld->getSkeleton(0), Vector3d(0.5, 1.0, 0.2));
    DrawSkeleton(mWorld->getSkeleton(1));

	glutSwapBuffers();
}

void SimWindow::keyboard(unsigned char key, int x, int y) {
    switch(key){
        case 27: exit(0);
        default: break;
    }
	glutPostRedisplay();
}

void SimWindow::mouse(int button, int state, int x, int y) {
    if(button == 3 || button == 4){
        if (button == 3)
        {
            mCamera->panCamera(0,-5,0,0);
        }
        else
        {
            mCamera->panCamera(0,5,0,0);
        }
    }
    else{
        if (state == GLUT_DOWN)
        {
            mIsDrag = true;
            mMouseType = button;
            mPrevX = x;
            mPrevY = y;
        }
        else
        {
            mIsDrag = false;
            mMouseType = 0;
        }
    }
}

void SimWindow::motion(int x, int y) {
    if (!mIsDrag)
        return;

    int mod = glutGetModifiers();
    if (mMouseType == GLUT_LEFT_BUTTON)
    {
        mCamera->translateCamera(x,y,mPrevX,mPrevY);
    }
    else if (mMouseType == GLUT_RIGHT_BUTTON)
    {
        mCamera->rotateCamera(x,y,mPrevX,mPrevY);

    }
    mPrevX = x;
    mPrevY = y;
}

void SimWindow::reshape(int w, int h) {
	glViewport(0, 0, w, h);
	mCamera->applySetting();
}

void SimWindow::timer(int value) {
    mWorld->step();
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, timerEvent, 1);
}
