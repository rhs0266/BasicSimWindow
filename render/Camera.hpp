//
// Created by rhs0266 on 2/19/19.
//

#ifndef GUI_CAMERA_HPP
#define GUI_CAMERA_HPP
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

namespace GUI {
class Camera {
public:
    Camera();
    ~Camera();

    /// Set position to look, eye position, up vector explicitly.
    void setCamera(const Eigen::Vector3d& lookAt,const Eigen::Vector3d& eye,const Eigen::Vector3d& up);

    /// Apply camera setting at window.
    void applySetting();

    /// Zoom camera.
    void zoomCamera(int x,int y, int prevX, int prevY);

    /// Rotate camera.
    void rotateCamera(int x,int y, int prevX, int prevY);

    /// Translate camera.
    void translateCamera(int x,int y, int prevX, int prevY);

    /// Varibles about camera.
    Eigen::Vector3d lookAt;
    Eigen::Vector3d eye;
    Eigen::Vector3d up;
    double fovy;

private:
    Eigen::Vector3d rotateQ(const Eigen::Vector3d &target, const Eigen::Vector3d &rotateVector, double angle);
    Eigen::Vector3d getTrackballPoint(int mouseX, int mouseY, int w, int h);
    Eigen::Vector3d unProject(const Eigen::Vector3d &vec);

};
}


#endif //GUI_CAMERA_HPP
