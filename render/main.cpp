#include <iostream>
#include <GL/glut.h>

#include "SimWindow.hpp"

using namespace std;

void render(int argc, char **argv, SimWindow *SimWindow){
	glutInit(&argc, argv);
	SimWindow->initWindow(1000, 1000, "Render");
	glutMainLoop();
}

int main(int argc, char** argv) {

	SimWindow *simWindow = new SimWindow();
	render(argc, argv, simWindow);

    return 0;
}