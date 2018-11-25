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

// void SamplePlugin::timer() {
// 	if (_framegrabber != NULL) {
// 		// Get the image as a RW image
// 		Frame* cameraFrame = _wc->findFrame("CameraSim");
// 		_framegrabber->grab(cameraFrame, _state);
// 		const Image& image = _framegrabber->getImage();
//
// 		// Convert to OpenCV image
// 		Mat im = toOpenCVImage(image);
// 		Mat imflip;
// 		cv::flip(im, imflip, 0);
//
// 		// Show in QLabel
// 		QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
// 		QPixmap p = QPixmap::fromImage(img);
// 		unsigned int maxW = 400;
// 		unsigned int maxH = 800;
// 		_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
// 	}
// }


void SamplePlugin::timer() {
  static int index = 0;
  static rw::math::Vector2D<> previousU(0,0);
  static rw::math::Q previousQ(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);

  _device->setQ(previousQ, _state);


  Frame* cameraFrame = _wc->findFrame("CameraSim");

  rw::math::Transform3D<> worldTbase = _device->worldTbase(_state);
  rw::math::Transform3D<> baseTcamera = _device->baseTframe(cameraFrame, _state);
  rw::math::Transform3D<> worldTcamera = worldTbase * baseTcamera;

  rw::math::Vector3D<> worldPOSmarker(_poses[index][0], _poses[index][1], _poses[index][2]);
  rw::math::RPY<> worldROTmarker(_poses[index][3], _poses[index][4], _poses[index][5]);

  rw::math::Transform3D<> worldRmarker(worldPOSmarker, worldROTmarker.toRotation3D());
  rw::math::Transform3D<> cameraRmarker = worldTcamera * worldRmarker;

  rw::math::Vector3D<> cameraPOSmarker = cameraRmarker.P();

  double u = cameraPOSmarker[0] / cameraPOSmarker[2] * f;
  double v = cameraPOSmarker[1] / cameraPOSmarker[2] * f;

  // cout << index << '\n';

  rw::math::Vector2D<> newU(u,v);
  // rw::math::Vector2D<> ret(0,0);
  // newU(0) = u;
  // newU(1) = v;

  // newU = getNewU(_wc, _state, _device, _poses[index]);

  rw::math::Jacobian Zimage(2,7);
  Zimage=getZimage(_wc, _state, _device, newU);

  rw::math::Jacobian Zimage_inverse(7,2);
  Zimage_inverse = pseudo_inverse_27(Zimage);
  // cout << index << '\n';

  // Zimage_inverse * previousU;
  previousU = newU;
  // previousQ = newQ;
  // cout << index << '\n';
  index++;
}

rw::math::Vector2D<> getNewU(rw::models::WorkCell::Ptr wc,
                    rw::kinematics::State state,
                    rw::models::Device::Ptr device, std::vector<double> pose){

  Frame* cameraFrame = wc->findFrame("CameraSim");

  rw::math::Transform3D<> worldTbase = device->worldTbase(state);
  rw::math::Transform3D<> baseTcamera = device->baseTframe(cameraFrame, state);
  rw::math::Transform3D<> worldTcamera = worldTbase * baseTcamera;

  rw::math::Vector3D<> worldPOSmarker(pose[0], pose[1], pose[2]);
  rw::math::RPY<> worldROTmarker(pose[3], pose[4], pose[5]);

  rw::math::Transform3D<> worldRmarker(worldPOSmarker, worldROTmarker.toRotation3D());
  rw::math::Transform3D<> cameraRmarker = worldTcamera * worldRmarker;

  rw::math::Vector3D<> cameraPOSmarker = cameraRmarker.P();

  double u = cameraPOSmarker[0] / cameraPOSmarker[2] * f;
  double v = cameraPOSmarker[1] / cameraPOSmarker[2] * f;

  rw::math::Vector2D<> ret(0,0);
  ret(0) = u;
  ret(1) = v;

  return ret;
}

rw::math::Jacobian SamplePlugin::getZimage(rw::models::WorkCell::Ptr wc,
                    rw::kinematics::State state, rw::models::Device::Ptr device,
                    rw::math::Vector2D<> U){
  int i,j;

  Frame* cameraFrame = wc->findFrame("CameraSim");

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

  rw::math::Jacobian Sq(6,6);
  rw::math::Transform3D<> baseTcamera = device->baseTframe(cameraFrame, state);

  rw::math::Rotation3D<> baseRcam(baseTcamera.R().getCol(0),
                                  baseTcamera.R().getCol(1),
                                  baseTcamera.R().getCol(2));
  inverse(baseRcam);

  for(i = 0; i < 3; i++) {
    for(j = 0; j < 3; j++) {
        Sq(i,j) = baseRcam(i,j);
        Sq(i+3,j) = 0;
    }
  }

  for(i = 0; i < 3; i++) {
    for(j = 3; j < 6; j++) {
        Sq(i+3,j) = baseRcam(i,j-3);
        Sq(i,j) = 0;
    }
  }

  rw::math::Jacobian Zimage(2,7);
  Zimage = Jimage * Sq * Jq;

  return Zimage;
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}
