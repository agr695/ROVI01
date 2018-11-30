#include "SamplePlugin.hpp"
#include "auxiliar_library.h"
#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

using namespace std::placeholders;

using namespace std;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/student/Downloads/ROVI01/final_project/Workcell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
	im = imread("/home/student/Downloads/ROVI01/final_project/Plugin/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

  if (_wc != NULL) {
  	// Add the texture render to this workcell if there is a frame for texture
  	Frame* textureFrame = _wc->findFrame("MarkerTexture");
  	if (textureFrame != NULL) {
  		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
  	}
  	// Add the background render to this workcell if there is a frame for texture
  	Frame* bgFrame = _wc->findFrame("Background");
  	if (bgFrame != NULL) {
  		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
  	}

  	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
  	Frame* cameraFrame = _wc->findFrame("CameraSim");
  	if (cameraFrame != NULL) {
  		if (cameraFrame->getPropertyMap().has("Camera")) {
  			// Read the dimensions and field of view
  			double fovy;
  			int width,height;
  			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
  			std::istringstream iss (camParam, std::istringstream::in);
  			iss >> fovy >> width >> height;
  			// Create a frame grabber
  			_framegrabber = new GLFrameGrabber(width,height,fovy);
  			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
  			_framegrabber->init(gldrawer);
  		}
  	}

    const std::string device_name = "PA10";
    _device = _wc->findDevice(device_name);
    if(_device == nullptr) {
        RW_THROW("Device " << device_name << " was not found!");
    }
  }

  /*load motion file*/
  _poses=read_marker_motion_file("/home/student/Downloads/ROVI01/final_project/Plugin/motions/MarkerMotionSlow.txt");
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
		image = ImageLoader::Factory::load("/home/student/Downloads/ROVI01/final_project/Plugin/markers/Marker1.ppm");
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load("/home/student/Downloads/ROVI01/final_project/Plugin/backgrounds/color1.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
		    _timer->start(100); // run 10 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::timer() {
  static int index=0;
  Frame* cameraFrame = _wc->findFrame("CameraSim");
  Frame* markerFrame = _wc->findFrame("Marker");
  static rw::math::Vector2D<double> prevU=get_newU_1point(markerFrame, cameraFrame);

  // Frame* camera = _wc->findFrame("Camera");
  MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");

  rw::math::Vector3D<> worldPOSmarker(_poses[index][0], _poses[index][1], _poses[index][2]);
  rw::math::RPY<> worldROTmarker(_poses[index][3], _poses[index][4], _poses[index][5]);

  rw::math::Transform3D<> worldRmarker(worldPOSmarker, worldROTmarker.toRotation3D());

  marker->moveTo(worldRmarker, _state);


	if (_framegrabber != NULL) {
		// Get the image as a RW image
		// Frame* cameraFrame = _wc->findFrame("CameraSim");
    // Frame* markerFrame = _wc->findFrame("Marker");

		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
		Mat im = toOpenCVImage(image);
		Mat imflip;
		cv::flip(im, imflip, 0);

    //get previous joint configuration
    rw::math::Q previousQ = _device->getQ(_state);

    // Get new image point:
    rw::math::Vector2D<double> newU = get_newU_1point(markerFrame, cameraFrame);

    //compute Zimage
    rw::math::Jacobian Zimage(2,7);
    Zimage=getZimage(_wc, _state, _device, newU,cameraFrame);

    //get pseudo inverse Zimage
    Eigen::MatrixXd Zimage_inverse;
    Zimage_inverse = pseudo_inverse_Zimage(Zimage);

    //compute new joint configuration
    rw::math::Vector2D<double> dU=newU-prevU;
    rw::math::Q dq = compute_dq(dU, Zimage_inverse);
    rw::math::Q newQ = previousQ + dq;

    //update robot state
    _device->setQ(newQ, _state);

    // Update RWStudio scene
    getRobWorkStudio()->setState(_state);

		// Show in QLabel
		QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
		QPixmap p = QPixmap::fromImage(img);
		unsigned int maxW = 400;
		unsigned int maxH = 800;
		_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

    index++;
    prevU=newU;
	}
}

rw::math::Vector2D<double> SamplePlugin::get_newU_1point(Frame *marker, Frame *camera){

  rw::math::Vector2D<double> newU;

  // 3D coordinates of the center of the image
	rw::math::Vector3D<double> markerpoint = rw::math::Vector3D<double>(0, 0, 0);
	rw::math::Transform3D<double> cameraTmarker = rw::kinematics::Kinematics::frameTframe(camera, marker, _state); // inverse(marker->....)

  rw::math::Vector3D<double> cameraPOSmarker = cameraTmarker * markerpoint;

  newU(0) = cameraPOSmarker[0] / cameraPOSmarker[2] * f;
  newU(1) = cameraPOSmarker[1] / cameraPOSmarker[2] * f;

  return newU;

}

rw::math::Jacobian SamplePlugin::getZimage(rw::models::WorkCell::Ptr wc,
                    rw::kinematics::State state, rw::models::Device::Ptr device,
                    rw::math::Vector2D<> U, Frame* cameraFrame){

  Frame* camera = wc->findFrame("Camera");

  rw::math::Jacobian Jq = device->baseJframe(cameraFrame, state);
  // std::cout << Jq.size1() << '\n';
  // std::cout << Jq.size2() << '\n';

  rw::math::Jacobian Jimage(2,6);

  double u = U(0);
  double v = U(1);

  Jimage(0,0) = -f / z;
  Jimage(0,1) = 0;
  Jimage(0,2) = u / z;
  Jimage(0,3) = (u * v) / f;
  Jimage(0,4) = -(f * f + u * u) / f;
  Jimage(0,5) = v;

  Jimage(1,0) = 0;
  Jimage(1,1) = -f / z;
  Jimage(1,2) = v / z;
  Jimage(1,3) = (f * f + u * u) / f;
  Jimage(1,4) = -(u * v) / f;
  Jimage(1,5) = -u;

  rw::math::Transform3D<> baseTcamera = device->baseTframe(camera, state);

  rw::math::Rotation3D<> baseRcam=baseTcamera.R();
  inverse(baseRcam);

  rw::math::Jacobian Sq=Jacobian(baseRcam);

  rw::math::Jacobian Zimage(2,7);
  Zimage = Jimage * Sq * Jq;

  return Zimage;
}

Eigen::MatrixXd SamplePlugin::pseudo_inverse_Zimage(rw::math::Jacobian Zimage){

  Eigen::MatrixXd Zimage_matrix=Zimage.e();

  return LinearAlgebra::pseudoInverse(Zimage_matrix);
}

rw::math::Q SamplePlugin::compute_dq(rw::math::Vector2D<double> dU, Eigen::MatrixXd Zimage_inverse){

  rw::math::Jacobian dq_jac(Zimage_inverse*dU.e());

	// Convert to joint vector
  rw::math::Q dq(dq_jac.e());

  return dq;
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}
