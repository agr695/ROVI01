#include "SamplePlugin.hpp"
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

//constant values
#define f 823 //pixels
#define z 0.5 //meters

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
