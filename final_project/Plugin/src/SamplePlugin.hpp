#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/math.hpp>
#include <rw/math/LinearAlgebra.hpp>


// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include <QTimer>

#include "ui_SamplePlugin.h"

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();
    void timer();
    void stateChangedListener(const rw::kinematics::State& state);

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QTimer* _timer;

    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;

    /*own variables*/
    std::vector<std::vector<double>> _poses;
    rw::models::Device::Ptr _device;
    std::vector<rw::math::Q> _jointConfigurations;
    std::vector<rw::math::Transform3D<double>> _toolPoses;
    std::vector<std::vector<double>> _tracking_error;
    bool _btn1_pressed = false;
    bool _btn2_pressed = false;

    /*own functions*/
    rw::math::Vector2D<double> get_newU_1point(rw::kinematics::Frame* markerFrame,
                                        rw::kinematics::Frame* camera);

    rw::math::Jacobian get_newU_3points(rw::kinematics::Frame* markerFrame,
                                        rw::kinematics::Frame* camera);

    rw::math::Jacobian getZimage_1point(rw::models::WorkCell::Ptr wc,
                       rw::kinematics::State state, rw::models::Device::Ptr device,
                       rw::math::Vector2D<> U);

    rw::math::Jacobian getZimage_3points(rw::models::WorkCell::Ptr wc,
                       rw::kinematics::State state, rw::models::Device::Ptr device,
                       rw::math::Jacobian U);

    Eigen::MatrixXd pseudo_inverse_Zimage(rw::math::Jacobian J);

    rw::math::Q compute_dq(rw::math::Vector2D<double> dU, Eigen::MatrixXd Zimage_inverse);

    rw::math::Q checkLimits(rw::math::Q dq, double dt, rw::models::Device::Ptr device);

    std::vector<double> get_distance(Eigen::MatrixXd real, Eigen::MatrixXd reference);

    void save_jointConfiguration_toolPose_trackingError(std::vector<rw::math::Q> joint,
                                std::vector<rw::math::Transform3D<double>> tool,
                                std::vector<std::vector<double>> tracking_error,
                                std::string joint_path, std::string tool_path,
                                std::string error_path);

};


#endif /*RINGONHOOKPLUGIN_HPP_*/
