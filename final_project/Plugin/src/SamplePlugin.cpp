#include "SamplePlugin.hpp"
#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

//constant values
#define f 823 //pixels
#define z 0.5 //meters
#define delta_t 1 //s

//input files path
#define workcell_path "/home/student/Downloads/ROVI01/final_project/Workcell/ScenePA10RoVi1.wc.xml"
#define initial_image_path "/home/student/Downloads/ROVI01/final_project/Plugin/src/lena.bmp"
#define fast_marker_file_path "/home/student/Downloads/ROVI01/final_project/Plugin/motions/MarkerMotionFast.txt"
#define medium_marker_file_path "/home/student/Downloads/ROVI01/final_project/Plugin/motions/MarkerMotionMedium.txt"
#define slow_marker_file_path "/home/student/Downloads/ROVI01/final_project/Plugin/motions/MarkerMotionSlow.txt"
#define marker_image_path "/home/student/Downloads/ROVI01/final_project/Plugin/markers/Marker1.ppm"
#define background_image_path "/home/student/Downloads/ROVI01/final_project/Plugin/backgrounds/color1.ppm"

//output files path
#define joint_variable_file_1point "../output/joint_1point.txt"
#define tool_pose_file_1point "../output/tool_pose_1point.txt"
#define joint_variable_file_3points "../output/joint_3points.txt"
#define tool_pose_file_3points "../output/tool_pose_3points.txt"

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

//Read marker position file
std::vector<std::vector<double>> read_marker_motion_file(const string file_name){
  ifstream motion;                    // build a read-Stream
  motion.open(file_name, ios_base::in);  // open data

  std::vector<std::vector<double>> ret;
  std::vector<double> pose(6);
  double X=0, Y=0, Z=0, Roll=0, Pitch=0, Yaw=0;

  while(!motion.eof()){
    motion >> X >> Y >> Z >> Roll >> Pitch >> Yaw;
    pose[0] = X;
    pose[1] = Y;
    pose[2] = Z;
    pose[3] = Roll;
    pose[4] = Pitch;
    pose[5] = Yaw;

    ret.push_back(pose);
  }
  return ret;
}


SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn2    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
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
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(workcell_path);
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
	im = imread(initial_image_path, CV_LOAD_IMAGE_COLOR); // Read the file
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

    // load device
    const std::string device_name = "PA10";
    _device = _wc->findDevice(device_name);
    if(_device == nullptr) {
        RW_THROW("Device " << device_name << " was not found!");
    }
  }

  // set initial joint configuration
  rw::math::Q Q_init(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
  _device->setQ(Q_init, _state);
	getRobWorkStudio()->setState(_state);
  /*load motion file*/
  _poses=read_marker_motion_file(medium_marker_file_path);
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
		image = ImageLoader::Factory::load(marker_image_path);
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load(background_image_path);
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive()){
        _btn1_pressed = true;
        _btn2_pressed = false;
		    _timer->start(10/delta_t); //interval in hz=10/delta_t
		}else
			_timer->stop();
	} else if(obj==_btn2){
		log().info() << "Button 2\n";
		// Toggle the timer on and off
		if (!_timer->isActive()){
        _btn1_pressed = false;
        _btn2_pressed = true;
		    _timer->start(10/delta_t); //interval in hz=10/delta_t
		}else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::timer() {
  static unsigned int index=0;

  /***************************************************************************
   *************image following without vision using 1 point*****************
   **************************************************************************/
  if(index<_poses.size()){
    if(_btn1_pressed){
      MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");

      rw::math::Vector3D<double> worldPOSmarker(_poses[index][0], _poses[index][1], _poses[index][2]);
      rw::math::RPY<double> worldROTmarker(_poses[index][3], _poses[index][4], _poses[index][5]);

      rw::math::Transform3D<double> worldRmarker(worldPOSmarker, worldROTmarker.toRotation3D());

      marker->moveTo(worldRmarker, _state);


    	if (_framegrabber != NULL) {
    		// Get the image as a RW image
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        Frame* camera = _wc->findFrame("Camera");
        Frame* markerFrame = _wc->findFrame("Marker");
        static rw::math::Vector2D<double> prevU=get_newU_1point(markerFrame, camera);

    		_framegrabber->grab(cameraFrame, _state);
    		const Image& image = _framegrabber->getImage();

    		// Convert to OpenCV image
    		Mat im = toOpenCVImage(image);
    		Mat imflip;
    		cv::flip(im, imflip, 0);

        // Get new image point:
        rw::math::Vector2D<double> newU = get_newU_1point(markerFrame, camera);

        //compute Zimage
        rw::math::Jacobian Zimage(2,7);
        Zimage=getZimage_1point(_wc, _state, _device, newU);

        //get pseudo inverse Zimage
        Eigen::MatrixXd Zimage_inverse;
        Zimage_inverse = pseudo_inverse_Zimage(Zimage);

        //compute new joint configuration
        rw::math::Vector2D<double> dU=prevU-newU;
        rw::math::Q dq = compute_dq(dU, Zimage_inverse);
        double dt = 10.0/(_timer->interval());
        dq=checkLimits(dq, dt, _device);

        //update joint configuration
        rw::math::Q newQ = _device->getQ(_state);
        newQ += dq;
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

        //update static variables
        index++;
        prevU = get_newU_1point(markerFrame, camera);
        rw::math::Transform3D<double> tool_pose = _device->baseTframe(camera, _state);
        _jointConfigurations.push_back(newQ);
        _toolPoses.push_back(tool_pose);
    	}
      if(index == _poses.size())
        save_jointConfiguration_toolPose(_jointConfigurations, _toolPoses,
                                         joint_variable_file_1point, tool_pose_file_1point);
    }

    /***************************************************************************
     *************image following without vision using 3 points*****************
     **************************************************************************/
    else if(_btn2_pressed){
      MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");

      rw::math::Vector3D<double> worldPOSmarker(_poses[index][0], _poses[index][1], _poses[index][2]);
      rw::math::RPY<double> worldROTmarker(_poses[index][3], _poses[index][4], _poses[index][5]);

      rw::math::Transform3D<double> worldRmarker(worldPOSmarker, worldROTmarker.toRotation3D());

      marker->moveTo(worldRmarker, _state);

    	if (_framegrabber != NULL) {
    		// Get the image as a RW image
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        Frame* camera = _wc->findFrame("Camera");
        Frame* markerFrame = _wc->findFrame("Marker");
        static rw::math::Jacobian prevU=get_newU_3points(markerFrame, camera);

    		_framegrabber->grab(cameraFrame, _state);
    		const Image& image = _framegrabber->getImage();

    		// Convert to OpenCV image
    		Mat im = toOpenCVImage(image);
    		Mat imflip;
    		cv::flip(im, imflip, 0);

        // Get new image point:
        rw::math::Jacobian newU = get_newU_3points(markerFrame, camera);

        //compute Zimage
        rw::math::Jacobian Zimage(2,7);
        Zimage=getZimage_3points(_wc, _state, _device, newU);

        //get Zimage inverse
        Eigen::MatrixXd Zimage_inverse;
        Zimage_inverse = LinearAlgebra::inverse(Zimage.e());

        //compute new joint configuration
        rw::math::Jacobian dU(prevU.e() - newU.e());

        rw::math::Jacobian dq_jac(Zimage_inverse*dU.e());
      	// create joint vector
        rw::math::Q dq(dq_jac.e());

        double dt = 10.0/(_timer->interval());
        dq=checkLimits(dq, dt, _device);

        //update joint configuration
        rw::math::Q newQ = _device->getQ(_state);
        newQ += dq;
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

        //update static variables
        index++;
        prevU = get_newU_3points(markerFrame, camera);
        rw::math::Transform3D<double> tool_pose = _device->baseTframe(camera, _state);
        _jointConfigurations.push_back(newQ);
        _toolPoses.push_back(tool_pose);
    	}
      if(index == _poses.size())
        save_jointConfiguration_toolPose(_jointConfigurations, _toolPoses,
                                        joint_variable_file_3points, tool_pose_file_3points);
    }
  }
}

rw::math::Vector2D<double> SamplePlugin::get_newU_1point(Frame *markerFrame, Frame *camera){

  rw::math::Vector2D<double> newU;

  // 3D coordinates of the center of the image
	rw::math::Vector3D<double> markerpoint = rw::math::Vector3D<double>(0, 0, 0);

	rw::math::Transform3D<double> cameraTmarker = rw::kinematics::Kinematics::frameTframe(camera, markerFrame, _state); // inverse(marker->....)

  rw::math::Vector3D<double> cameraPOSmarker = cameraTmarker * markerpoint;

  newU(0) = f * cameraPOSmarker[0] / cameraPOSmarker[2];
  newU(1) = f * cameraPOSmarker[1] / cameraPOSmarker[2];

  return newU;
}

rw::math::Jacobian SamplePlugin::get_newU_3points(Frame *markerFrame,
                                                                Frame *camera){

  rw::math::Jacobian newU(6,1);

  // 3D coordinates of the three points in the image
  rw::math::Vector3D<double> markerpoint1 = rw::math::Vector3D<double>(0.15, 0.15, 0);
	rw::math::Vector3D<double> markerpoint2 = rw::math::Vector3D<double>(0.15, -0.15, 0);
	rw::math::Vector3D<double> markerpoint3 = rw::math::Vector3D<double>(-0.15, 0.15, 0);

	rw::math::Transform3D<double> cameraTmarker = rw::kinematics::Kinematics::frameTframe(camera, markerFrame, _state); // inverse(marker->....)

  //3D coordinates of each point
  std::vector<rw::math::Vector3D<double>> cameraPOSmarker;
	cameraPOSmarker.push_back(cameraTmarker * markerpoint1);
	cameraPOSmarker.push_back(cameraTmarker * markerpoint2);
	cameraPOSmarker.push_back(cameraTmarker * markerpoint3);

  //2D coordinates of each point
	for (size_t i=0; i<cameraPOSmarker.size(); i++ )
	{
		rw::math::Vector3D<double> point = cameraPOSmarker[i];
		newU(2*i,0) = f * point(0) / point(2);
		newU(1+2*i,0) = f * point(1) / point(2);
	}

  return newU;
}

rw::math::Jacobian SamplePlugin::getZimage_1point(rw::models::WorkCell::Ptr wc,
                    rw::kinematics::State state, rw::models::Device::Ptr device,
                    rw::math::Vector2D<> U){

  Frame* camera = wc->findFrame("Camera");

  rw::math::Jacobian Jq = device->baseJframe(camera, state);

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
  rw::math::Rotation3D<> baseRcam_inverse=inverse(baseRcam);

  rw::math::Jacobian Sq=Jacobian(baseRcam_inverse);

  rw::math::Jacobian Zimage(2,7);
  Zimage = Jimage * Sq * Jq;

  return Zimage;
}

rw::math::Jacobian SamplePlugin::getZimage_3points(rw::models::WorkCell::Ptr wc,
                    rw::kinematics::State state, rw::models::Device::Ptr device,
                    rw::math::Jacobian U){

  Frame* camera = wc->findFrame("Camera");

  rw::math::Jacobian Jq = device->baseJframe(camera, state);

  rw::math::Jacobian Jimage(U.size1(),6);
  for (size_t i = 0; i < U.size1()/2; i++) {
    double u = U(2*i,0);
    double v = U(1+2*i,0);

    Jimage(2*i,0) = -f / z;
    Jimage(2*i,1) = 0;
    Jimage(2*i,2) = u / z;
    Jimage(2*i,3) = (u * v) / f;
    Jimage(2*i,4) = -(f * f + u * u) / f;
    Jimage(2*i,5) = v;

    Jimage(1+2*i,0) = 0;
    Jimage(1+2*i,1) = -f / z;
    Jimage(1+2*i,2) = v / z;
    Jimage(1+2*i,3) = (f * f + u * u) / f;
    Jimage(1+2*i,4) = -(u * v) / f;
    Jimage(1+2*i,5) = -u;
  }

  rw::math::Transform3D<> baseTcamera = device->baseTframe(camera, state);

  rw::math::Rotation3D<> baseRcam=baseTcamera.R();
  rw::math::Rotation3D<> baseRcam_inverse=inverse(baseRcam);

  rw::math::Jacobian Sq=Jacobian(baseRcam_inverse);

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

	// create joint vector
  rw::math::Q dq(dq_jac.e());

  return dq;
}

rw::math::Q SamplePlugin::checkLimits(Q dq, double dt, rw::models::Device::Ptr device){
  //get velocity limits
  rw::math::Q vel_limits = device->getVelocityLimits();

  //get joint number
  unsigned int joint_size=dq.size();

  //dq limited
  rw::math::Q actual_dq;
  std::vector<double> q_limited(joint_size);


  for(size_t i = 0; i<joint_size; i++){
    double speed=abs(dq(i)/dt);

    if(speed>vel_limits(i)){
      //signed check to mantain the direction of the joint
      if(dq(i)>0)
        q_limited[i] = vel_limits(i)*dt;
      else
        q_limited[i] = -vel_limits(i)*dt;
    }
    else{
      q_limited[i] = dq(i);
    }
  }

  actual_dq=Q(q_limited);
  return actual_dq;
}

// save joint configuration and tool pose
void SamplePlugin::save_jointConfiguration_toolPose(std::vector<rw::math::Q> joint,
                                                std::vector<rw::math::Transform3D<double>> tool,
                                                std::string joint_path, std::string tool_path){

  ofstream joint_file;
  joint_file.open(joint_path);
  for (size_t i = 0; i < joint.size(); i++) {
    rw::math::Q configuration=joint[i];
    for (size_t j = 0; j < configuration.size(); j++) {
      joint_file <<configuration[j] << "\t";
    }
    joint_file << "\n";
  }
  joint_file.close();

  ofstream tool_file;
  tool_file.open(tool_path);
  for (size_t i = 0; i < tool.size(); i++) {
    rw::math::Vector3D<double> position = tool[i].P();
    rw::math::RPY<double> angles = RPY<double>(tool[i].R());
    tool_file <<position(0) <<"\t" << position(1) << "\t" << position(2) << "\t";
    tool_file <<angles(0) <<"\t" << angles(1) << "\t" << angles(2) << "\t";
    tool_file <<"\n";
  }
  tool_file.close();
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}
