//
// Created by rhs0266 on 2/19/19.
//

#ifndef GUI_GLUTWINDOW_H
#define GUI_GLUTWINDOW_H
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <memory>

namespace GUI {

class Camera;

class GLUTWindow {
public:
    /// Constructor
    GLUTWindow();

    /// Destructor
    ~GLUTWindow();

    /// Initialize window size and name.
    virtual void initWindow(int _w, int _h, const char* _name);

    /// Return pointer of current window.
    static GLUTWindow* current();

    /// Several event callback functions such as display, keyboard, mouse, motion, reshape, and timer.
    static void displayEvent();
    static void keyboardEvent(unsigned char key, int x, int y);
    static void mouseEvent(int button, int state, int x, int y);
    static void motionEvent(int x, int y);
    static void reshapeEvent(int w, int h);
    static void timerEvent(int value);

    static std::vector<GLUTWindow*> mWindows;
    static std::vector<int> mWinIDs;

protected:

    /// Initialize light setting.
    virtual void initLights();

    /// Display event.
    virtual void display() = 0;

    /// Keyboard interrupt event.
    virtual void keyboard(unsigned char key, int x, int y) = 0;

    /// Mouse click event.
    virtual void mouse(int button, int state, int x, int y) = 0;

    /// Motion of mouse event.
    virtual void motion(int x, int y) = 0;

    /// Reshape window size event.
    virtual void reshape(int w, int h) = 0;

    /// Timer event.
    virtual void timer(int value) = 0;

    /// Camera object.
    std::unique_ptr<Camera> 		mCamera;

    /// Flag of dragging.
    bool 							mIsDrag;

    /// Memorize button type.
    int 							mMouseType;

    /// Memorize previous mouse position.
    int 							mPrevX,mPrevY;

    /// Display Timeout unit.
    int 							mDisplayTimeout;

};
}
#endif //GUI_GLUTWINDOW_H
