//
// Created by rhs0266 on 2/19/19.
//

#include "GLUTWindow.hpp"
#include "Camera.hpp"
#include <iostream>
#include <GL/glut.h>

using namespace std;
using namespace GUI;
std::vector<GLUTWindow*> GLUTWindow::mWindows;
std::vector<int> GLUTWindow::mWinIDs;

GUI::GLUTWindow::GLUTWindow(): mCamera(new Camera()),
                               mIsDrag(false),
                               mMouseType(0),
                               mPrevX(0),mPrevY(0),
                               mDisplayTimeout(1.0/30.0) {

}

GUI::GLUTWindow::~GLUTWindow() {

}

void GUI::GLUTWindow::initWindow(int _w, int _h, const char *_name) {
    mWindows.push_back(this);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE | GLUT_ACCUM);
    glutInitWindowPosition(150, 100);
    glutInitWindowSize(_w, _h);
    mWinIDs.push_back(glutCreateWindow(_name));
    glutDisplayFunc(displayEvent);
    glutReshapeFunc(reshapeEvent);
    glutMouseFunc(mouseEvent);
    glutMotionFunc(motionEvent);
    glutKeyboardFunc(keyboardEvent);
    glutTimerFunc(mDisplayTimeout, timerEvent, 0);

}

GUI::GLUTWindow *GUI::GLUTWindow::current() {
    int id = glutGetWindow();
    for (int i=0;i<mWinIDs.size();i++){
        if (mWinIDs[i] == id){
            return mWindows[i];
        }
    }
    cout << "An unknown error occurred!" << endl;
    exit(0);
}

void GUI::GLUTWindow::displayEvent() {
    current()->display();
}

void GUI::GLUTWindow::keyboardEvent(unsigned char key, int x, int y) {
    current()->keyboard(key, x, y);
}

void GUI::GLUTWindow::mouseEvent(int button, int state, int x, int y) {
    current()->mouse(button, state, x, y);
}

void GUI::GLUTWindow::motionEvent(int x, int y) {
    current()->motion(x, y);
}

void GUI::GLUTWindow::reshapeEvent(int w, int h) {
    current()->reshape(w, h);
}

void GUI::GLUTWindow::timerEvent(int value) {
    current()->timer(value);

}

void GUI::GLUTWindow::initLights() {
    static float ambient[]             = {0.2, 0.2, 0.2, 1.0};
    static float diffuse[]             = {0.6, 0.6, 0.6, 1.0};
    static float front_mat_shininess[] = {60.0};
    static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
    static float front_mat_diffuse[]   = {0.5, 0.28, 0.38, 1.0};
    static float lmodel_ambient[]      = {0.2, 0.2,  0.2,  1.0};
    static float lmodel_twoside[]      = {GL_FALSE};

    GLfloat position[] = {0.0, 5.0, 1.0, 0.0};
//    GLfloat position1[] = {0.0, 5.0, -1.0, 0.0};

    // Light0
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

    // Light1
//    glEnable(GL_LIGHT1);
//    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
//    glLightfv(GL_LIGHT1, GL_POSITION, position1);

    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);

    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);
}