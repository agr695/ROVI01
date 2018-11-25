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

#define f 823 //pixels
#define z 0.5 //meters

std::vector<std::vector<double>> read_marker_motion_file(const string file_name){
  ifstream motion;                    // build a read-Stream
  motion.open(file_name, ios_base::in);  // open data

  std::vector<std::vector<double>> ret(501, std::vector<double>(6));
  double X=0, Y=0, Z=0, Roll=0, Pitch=0, Yaw=0;
  int index = 0;

  while(!motion.eof()){
    motion >> X >> Y >> Z >> Roll >> Pitch >> Yaw;
    ret[index][0] = X;
    ret[index][1] = Y;
    ret[index][2] = Z;
    ret[index][3] = Roll;
    ret[index][4] = Pitch;
    ret[index][5] = Yaw;
    index++;
    // std::cout << X << '\n';
  }
  return ret;
}


rw::math::Jacobian pseudo_inverse_27(rw::math::Jacobian J){
  /*pseudo inverse computed with symbolic toolbox in matlab*/
  rw::math::Jacobian Ret(7,2);
  double J11=J(1,1);
  double J12=J(1,2);
  double J13=J(1,3);
  double J14=J(1,4);
  double J15=J(1,5);
  double J16=J(1,6);
  double J17=J(1,7);
  double J21=J(2,1);
  double J22=J(2,2);
  double J23=J(2,3);
  double J24=J(2,4);
  double J25=J(2,5);
  double J26=J(2,6);
  double J27=J(2,7);

  Ret(1,1) = (J11*(J21*J21 + J22*J22 + J23*J23 + J24*J24 + J25*J25 + J26*J26 + J27*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J21*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(1,2)=(J21*(J11*J11 + J12*J12 + J13*J13 + J14*J14 + J15*J15 + J16*J16 + J17*J17))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J11*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(2,1)=(J12*(J21*J21 + J22*J22 + J23*J23 + J24*J24 + J25*J25 + J26*J26 + J27*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J22*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(2,2)=(J22*(J11*J11 + J12*J12 + J13*J13 + J14*J14 + J15*J15 + J16*J16 + J17*J17))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J12*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(3,1)=(J13*(J21*J21 + J22*J22 + J23*J23 + J24*J24 + J25*J25 + J26*J26 + J27*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J23*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(3,2)=(J23*(J11*J11 + J12*J12 + J13*J13 + J14*J14 + J15*J15 + J16*J16 + J17*J17))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J13*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(4,1)= (J14*(J21*J21 + J22*J22 + J23*J23 + J24*J24 + J25*J25 + J26*J26 + J27*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J24*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(4,2)= (J24*(J11*J11 + J12*J12 + J13*J13 + J14*J14 + J15*J15 + J16*J16 + J17*J17))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J14*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(5,1)= (J15*(J21*J21 + J22*J22 + J23*J23 + J24*J24 + J25*J25 + J26*J26 + J27*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J25*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(5,2)= (J25*(J11*J11 + J12*J12 + J13*J13 + J14*J14 + J15*J15 + J16*J16 + J17*J17))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J15*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(6,1)= (J16*(J21*J21 + J22*J22 + J23*J23 + J24*J24 + J25*J25 + J26*J26 + J27*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J26*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(6,2)= (J26*(J11*J11 + J12*J12 + J13*J13 + J14*J14 + J15*J15 + J16*J16 + J17*J17))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J16*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(7,1)= (J17*(J21*J21 + J22*J22 + J23*J23 + J24*J24 + J25*J25 + J26*J26 + J27*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J27*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  Ret(7,2)= (J27*(J11*J11 + J12*J12 + J13*J13 + J14*J14 + J15*J15 + J16*J16 + J17*J17))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26) - (J17*(J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26 + J17*J27))/(J11*J11*J22*J22 + J11*J11*J23*J23 + J11*J11*J24*J24 + J11*J11*J25*J25 + J11*J11*J26*J26 + J11*J11*J27*J27 - 2*J11*J12*J21*J22 - 2*J11*J13*J21*J23 - 2*J11*J14*J21*J24 - 2*J11*J15*J21*J25 - 2*J11*J16*J21*J26 - 2*J11*J17*J21*J27 + J12*J12*J21*J21 + J12*J12*J23*J23 + J12*J12*J24*J24 + J12*J12*J25*J25 + J12*J12*J26*J26 + J12*J12*J27*J27 - 2*J12*J13*J22*J23 - 2*J12*J14*J22*J24 - 2*J12*J15*J22*J25 - 2*J12*J16*J22*J26 - 2*J12*J17*J22*J27 + J13*J13*J21*J21 + J13*J13*J22*J22 + J13*J13*J24*J24 + J13*J13*J25*J25 + J13*J13*J26*J26 + J13*J13*J27*J27 - 2*J13*J14*J23*J24 - 2*J13*J15*J23*J25 - 2*J13*J16*J23*J26 - 2*J13*J17*J23*J27 + J14*J14*J21*J21 + J14*J14*J22*J22 + J14*J14*J23*J23 + J14*J14*J25*J25 + J14*J14*J26*J26 + J14*J14*J27*J27 - 2*J14*J15*J24*J25 - 2*J14*J16*J24*J26 - 2*J14*J17*J24*J27 + J15*J15*J21*J21 + J15*J15*J22*J22 + J15*J15*J23*J23 + J15*J15*J24*J24 + J15*J15*J26*J26 + J15*J15*J27*J27 - 2*J15*J16*J25*J26 - 2*J15*J17*J25*J27 + J16*J16*J21*J21 + J16*J16*J22*J22 + J16*J16*J23*J23 + J16*J16*J24*J24 + J16*J16*J25*J25 + J16*J16*J27*J27 - 2*J16*J17*J26*J27 + J17*J17*J21*J21 + J17*J17*J22*J22 + J17*J17*J23*J23 + J17*J17*J24*J24 + J17*J17*J25*J25 + J17*J17*J26*J26);

  return Ret;
}
