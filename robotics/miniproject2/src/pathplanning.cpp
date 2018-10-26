#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <fstream>


using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 10.
void export2LUA(QPath path);
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

int main(int argc, char** argv) {
	rw::math::Math::seed();
	const string wcFile = "../Kr16WallWorkCell/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	const string bottleName = "Bottle";
	const string gripName = "Tool";


	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	Frame* frameBottle = wc->findFrame(bottleName);
	Frame* frameGrip = wc->findFrame(gripName);

	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}

	if (frameBottle == NULL) {
		cerr << "frame: " << bottleName << " not found!" << endl;
		return 0;
	}
	if (frameGrip == NULL) {
		cerr << "frame: " << gripName << " not found!" << endl;
		return 0;
	}

	State state = wc->getDefaultState();

	Kinematics::gripFrame(frameBottle,frameGrip,state);

	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	/** Most easy way: uses default parameters based on given device
		sampler: QSampler::makeUniform(device)
		metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
		extend: 0.05 */
	//QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);

	/** More complex way: allows more detailed definition of parameters and methods */
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

	Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
	//Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner
	Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);

	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;


	double extend;
	QPath path[301];
	Timer t;
	QToQPlanner::Ptr planner;
	int index=0;

	float length_prev=0;
	ofstream time_file;
	time_file.open ("time.txt",ios::out|ios::trunc);

	ofstream path_file;
	path_file.open ("path_size.txt",ios::out|ios::trunc);

	for(extend=0.01; extend <=3.01; extend+=0.01){
		// rw::math::Math::seed(extend*100);
		planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
		t.resetAndResume();
		planner->query(from,to,path[index],MAXTIME);
		t.pause();

		cout << "Path "<< index <<" of length " << path[index].size() << " found in " << t.getTime() << " seconds." << endl;
		if (t.getTime() >= MAXTIME) {
			cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
		}

		time_file << t.getTime();
		time_file << "\n";

		path_file << path[index].size()-length_prev;
		path_file << "\n";

		index++;
		// length_prev=path.size();
	}
	time_file.close();
	path_file.close();
	export2LUA(path[25]);

	cout << "program finish \n";
	return 0;
}

void export2LUA(QPath path){
	ofstream LuaFile;
  LuaFile.open ("../LUA/LuaScript.lua",ios::out|ios::trunc);
  LuaFile << "wc = rws.getRobWorkStudio():getWorkCell()\n";
  LuaFile << "state = wc:getDefaultState()\n";
  LuaFile << "device = wc:findDevice(\"KukaKr16\")\n";
  LuaFile << "gripper = wc:findFrame(\"Tool\");\n";
  LuaFile << "bottle = wc:findFrame(\"Bottle\");\n";
  LuaFile << "table = wc:findFrame(\"Table\");\n";
  LuaFile << "\n";

  LuaFile << "function setQ(q)\n";
  LuaFile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n";
  LuaFile << "device:setQ(qq,state)\n";
  LuaFile << "rws.getRobWorkStudio():setState(state)\n";
  LuaFile << "rw.sleep(0.1)\n";
  LuaFile << "end\n";
  LuaFile << "\n";

  LuaFile << "function attach(obj, tool)\n";
  LuaFile << "rw.gripFrame(obj, tool, state)\n";
  LuaFile << "rws.getRobWorkStudio():setState(state)\n";
  LuaFile << "rw.sleep(0.1)\n";
  LuaFile << "end\n";

  LuaFile << "\n";
  LuaFile << "\n";
  LuaFile << "setQ({-3.142, -0.827, -3.002, -3.143, 0.099, -1.573})\n";
  LuaFile << "attach(bottle,gripper)\n\n";
	string string_Q;
	ostringstream ss;
	for (QPath::iterator it = path.begin()+1; it < path.end()-1; it++) {
		ss << *it;
		string_Q=ss.str();
		string_Q.erase(0, 4);
		LuaFile << "setQ("<< string_Q <<")\n\n";
		ss.str("");
		ss.clear();
		// LuaFile << *it;
	}

	LuaFile << "\n";
  LuaFile << "setQ({1.571, 0.006, 0.03, 0.153, 0.762, 4.49})\n";
	LuaFile << "attach(bottle,table)\n";

  LuaFile.close();
}
