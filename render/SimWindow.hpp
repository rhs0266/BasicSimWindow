//
// Created by rhs0266 on 2/21/19.
//

#ifndef GUI_SIMWINDOW_HPP
#define GUI_SIMWINDOW_HPP

#include "../sim/Character.hpp"
#include "GLUTWindow.hpp"
#include "Camera.hpp"
#include "GLPrimitiveFunctions.hpp"
#include "dart/simulation/simulation.hpp"


class SimWindow : public GUI::GLUTWindow{
public:
	/// Constructor.
	SimWindow();

	/// Destructor.
	~SimWindow();

	/// dart world in my window.
	dart::simulation::WorldPtr mWorld;

	/// Frame number.
	int mFrame;

	/// Flag of playing.
	bool mIsPlay;

	/// Flags of mouse motion.
	bool mIsRotate;
	bool mIsDrag;

	/// Display ratio.
	double mDisplayRatio;

private:
	/// Display event.
	void display() override;

	/// Keyboard interrupt event.
	void keyboard(unsigned char key, int x, int y) override;

	/// Mouse click event.
	void mouse(int button, int state, int x, int y) override;

	/// Motion of mouse event.
	void motion(int x, int y) override;

	/// Reshape window size event.
	void reshape(int w, int h) override;

	/// Timer event.
	void timer(int value) override;

	std::vector<Character> mCharacters;
};


#endif //GUI_SIMWINDOW_HPP
