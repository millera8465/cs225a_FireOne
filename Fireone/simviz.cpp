/**
 * @file simviz.cpp
 * @brief Simulation + visualization 
 * 
 */

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include <signal.h>
#include <list>
#include <chrono>
#include <math.h>  


bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"
#include "../include/object.h"

using namespace std;
using namespace Eigen;

struct Fire {

	Vector3d startingPosition;
	Vector3d startingDim;
	Vector3d dim;

	Fire(Vector3d fireStartingPosition, Vector3d fireStartingDim, Vector3d fireDim):
	startingPosition(fireStartingPosition), startingDim(fireStartingDim), dim(fireDim) 
	{
	}
};

// specify urdf and robots 
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "panda";
const string camera_name = "camera_fixed";
const string base_link_name = "linkbase";
const string ee_link_name = "link7";

// dynamic objects information
const vector<string> object_names = {"cup"};
vector<Vector3d> object_pos;
vector<Vector3d> object_lin_vel;
vector<Quaterniond> object_ori;
vector<Vector3d> object_ang_vel;
const int n_objects = object_names.size();

// redis client 
RedisClient redis_client; 
RedisClient graphics_redis;

// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

void startFire(Sai2Graphics::Sai2Graphics* graphics, const string& name, const Vector3d& fireStartingPosition, const Vector3d& fireDim);

void updateFire(Sai2Graphics::Sai2Graphics* graphics, const string& name, const Vector3d& fireStartingPosition, const Vector3d& fireDim);

void startFires(Sai2Graphics::Sai2Graphics* graphics, const string& name, const std::list<Fire>& firesList);
void updateFires(Sai2Graphics::Sai2Graphics* graphics, const string& name, const std::list<Fire>& firesList);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// start redis client for graphics
	graphics_redis = RedisClient();
	graphics_redis.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->_world->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	graphics->showLinkFrame(true, robot_name, ee_link_name, 0.15);  // can add frames for different links
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = VectorXd::Zero(17);
	robot->_dq = VectorXd::Zero(17);
	robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setJointPositions(robot_name, robot->_q);
	sim->setJointVelocities(robot_name, robot->_dq);

	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		Vector3d _object_pos, _object_lin_vel, _object_ang_vel;
		Quaterniond _object_ori;
		sim->getObjectPosition(object_names[i], _object_pos, _object_ori);
		sim->getObjectVelocity(object_names[i], _object_lin_vel, _object_ang_vel);
		object_pos.push_back(_object_pos);
		object_lin_vel.push_back(_object_lin_vel);
		object_ori.push_back(_object_ori);
		object_ang_vel.push_back(_object_ang_vel);
	}

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "Panda Starter Code", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	// init redis client values 
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");  
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq); 

	// start simulation thread
	thread sim_thread(simulation, robot, sim);

	// initialize glew
	glewInitialize();

	// while window is open:
	int count = 0;
	Vector3d fireStartingPosition = Vector3d(-4.0, 4.0, 0);
	

	double fireLength = 0.05; // hardcoded
	double fireWidth = 0.05; // hardcoded

	Vector3d fireStartingDim = Vector3d(fireLength, fireWidth, 0.1);
	Vector3d fireDim = Vector3d(fireLength, fireWidth, 0.1);
	//startFire(graphics, "fire",fireStartingPosition, fireStartingDim);

	//------------------------------------------------------------------------------

	/*std::list<Fire> firesList = { Fire(Vector3d(-4.0, 4.0, 0), Vector3d(fireLength, fireWidth, 0.1), Vector3d(fireLength, fireWidth, 0.1)),
								Fire(Vector3d(-4.0, -4.0, 0), Vector3d(fireLength, fireWidth, 0.1), Vector3d(fireLength, fireWidth, 0.1)),
								Fire(Vector3d(4.0, -4.0, 0), Vector3d(fireLength, fireWidth, 0.1), Vector3d(fireLength, fireWidth, 0.1))//,
								Fire(Vector3d(4.0, 4.0, 0), Vector3d(fireLength, fireWidth, 0.1), Vector3d(fireLength, fireWidth, 0.1))
								};*/


	std::list<Fire> firesList = { Fire(Vector3d(4.0, 4.0, 0), Vector3d(fireLength, fireWidth, 0.1), Vector3d(fireLength, fireWidth, 0.1)),
								// Fire(Vector3d(4.0, -4.0, 0), Vector3d(fireLength, fireWidth, 0.1), Vector3d(fireLength, fireWidth, 0.1)),
								Fire(Vector3d(-4.0, -4.0, 0), Vector3d(fireLength, fireWidth, 0.1), Vector3d(fireLength, fireWidth, 0.1))
								//, Fire(Vector3d(-4.0, 4.0, 0), Vector3d(fireLength, fireWidth, 0.1), Vector3d(fireLength, fireWidth, 0.1))
								};

	startFires(graphics, "fire",firesList);
	auto start = std::chrono::high_resolution_clock::now();
	//------------------------------------------------------------------------------


	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{


		// increase fire for every nth count
		//if (count % 10 == 0) {  // default refresh rate 
		auto timeNow = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float> duration = timeNow -start;
		if (duration.count() >= 1.0) {
			//updateFire(graphics, "fire", fireStartingPosition, fireDim);
			updateFires(graphics, "fire", firesList);
			//redis_client.setEigenMatrixJSON(FIRE_INFO_KEY, Vector4d::Random());
			//auto end = std::chrono::high_resolution_clock::now();
			//std::chrono::duration<float> duration = end -start;
			std::cout << duration.count() << " s" << std::endl;
			start = std::chrono::high_resolution_clock::now();

		}

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot); 
		for (int i = 0; i < n_objects; ++i) {
			graphics->updateObjectGraphics(object_names[i], object_pos[i], object_ori[i]);
		}
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();

		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
		
		count++;
	}

	// wait for simulation to finish
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim)
{

	// prepare simulation
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	VectorXd g = VectorXd::Zero(dof);
	string controller_status = "0";
	double kv = 10;  // can be set to 0 if no damping is needed

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// add to write callback
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime();
	double last_time = start_time;

	// start simulation 
	fSimulationRunning = true;	
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// apply gravity compensation 
		robot->gravityVector(g);

		// set joint torques
		if (controller_status == "1") {
			sim->setJointTorques(robot_name, command_torques + g);
		} else {
			sim->setJointTorques(robot_name, g - robot->_M * (kv * robot->_dq));
		}

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// get dynamic object positions
		for (int i = 0; i < n_objects; ++i) {
			sim->getObjectPosition(object_names[i], object_pos[i], object_ori[i]);
			sim->getObjectVelocity(object_names[i], object_lin_vel[i], object_ang_vel[i]);
		}

		// execute redis write callback
		redis_client.executeWriteCallback(0);		

		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------
void startFire(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const Vector3d& fireStartingPosition,
                const Vector3d& fireDim) {
	increaseRectangularFire(graphics, name,fireStartingPosition, Quaterniond(1, 0, 0, 0), fireDim, Vector4d(1, 0, 0, 1));
}

//------------------------------------------------------------------------------
void updateFire(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const Vector3d& fireStartingPosition,
                const Vector3d& fireDim) {

	double roomLength = 10.0; // hardcoded
	double roomWidth = 10.0; // hardcoded
	double fireLength = 0.05; // hardcoded
	double fireWidth = 0.05; // hardcoded
	double increaseFireLength = 1e-1; // hardcoded
	double increaseFireWidth = 1e-1; // hardcoded
	int case1 = 0;
	int case2 = 0;
	int case3 = 0;
	int case4 = 0;

	Vector3d& _fireStartingPosition = const_cast<Vector3d&>(fireStartingPosition);
	Vector3d& _fireDim = const_cast<Vector3d&>(fireDim);

	removeFire(graphics);


	// Fire collision with the right wall
	if (_fireStartingPosition(0) + _fireDim(0)/2 + increaseFireLength/2 > roomLength/2) {
		case1 = 1;
		//cout << "Case 1"<< endl;
	}
	// Fire collision with the left wall
	if(_fireStartingPosition(0) - _fireDim(0)/2 - increaseFireLength/2 < -roomLength/2) {
		case2 = 1;
		//cout << "Case 2"<< endl;
	}
	// Fire collision with the front wall
	if (_fireStartingPosition(1) + _fireDim(1)/2 + increaseFireWidth/2 > roomWidth/2) {
		case3 = 1;
		//cout << "Case 3"<< endl;
	}
	// Fire collision with the back wall
	if(_fireStartingPosition(1) - _fireDim(1)/2 - increaseFireWidth/2 < -roomWidth/2) {
		case4 = 1;
		//cout << "Case 4"<< endl;
	}
	_fireStartingPosition(0) = _fireStartingPosition(0) - case1*increaseFireLength/2 + case2*increaseFireLength/2;
	_fireStartingPosition(1) = _fireStartingPosition(1) - case3*increaseFireWidth/2 + case4*increaseFireWidth/2;
	_fireDim = Vector3d((2-case1-case2)*increaseFireWidth/2, (2-case3-case4)*increaseFireLength/2, 0) + _fireDim;
	increaseRectangularFire(graphics, "fire", _fireStartingPosition, Quaterniond(1, 0, 0, 0), _fireDim, Vector4d(1, 0, 0, 1));
	
}



//------------------------------------------------------------------------------
void startFires(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const std::list<Fire>& firesList) {

	//Create an iterator of std::list
	//std::list<Fire>::iterator it;

	// Make iterate point to begining and incerement it one by one till it reaches the end of list.
	//int num = 1;
	/*for (it = firesList.begin(); it != firesList.end(); it++)
	{
	    // Access the object through iterator
	    Vector3d startingPosition = it->startingPosition;
		//Vector3d startingDim = it->startingDim;
		Vector3d dim = it->dim;
		std::string _name = name;
		_name += std::to_string(num);   
		addFireToList(_name, startingPosition, Quaterniond(1, 0, 0, 0), dim, Vector4d(1, 0, 0, 1));
		num = num + 1;
	}*/
	int num = 1;
	for (const Fire & fire : firesList)
	{
    	// Access the object through iterator
	    Vector3d startingPosition = fire.startingPosition;
		//Vector3d startingDim = it->startingDim;
		Vector3d dim = fire.dim;
		std::string _name = name;
		_name += std::to_string(num);   
		addFireToList(_name, startingPosition, Quaterniond(1, 0, 0, 0), dim, Vector4d(1, 0, 0, 1));
		num = num + 1;
	}

	loadFiresToScene(graphics);
}

//------------------------------------------------------------------------------
void updateFires(Sai2Graphics::Sai2Graphics* graphics,
				const string& name,
				const std::list<Fire>& firesList) {

	double roomLength = 10.0; // hardcoded
	double roomWidth = 10.0; // hardcoded
	double fireLength = 0.05; // hardcoded
	double fireWidth = 0.05; // hardcoded
	double increaseFireLength = 1e-1; // hardcoded
	double increaseFireWidth = 1e-1; // hardcoded
	int case1 = 0;
	int case2 = 0;
	int case3 = 0;
	int case4 = 0;

	std::list<Fire>& _firesList = const_cast<std::list<Fire>&>(firesList);	

	removeFires(graphics);

	MatrixXd firesInfo = MatrixXd::Zero(_firesList.size(), 4);

	int num = 1;
	for (const Fire & fire : _firesList)
	{
		case1 = 0;
		case2 = 0;
		case3 = 0;
		case4 = 0;

		Fire& _fire = const_cast<Fire&>(fire);
		// Fire collision with the right wall
		if (_fire.startingPosition(0) + _fire.dim(0)/2 + increaseFireLength/2 > roomLength/2) {
			case1 = 1;
			//cout << "Case 1"<< endl;
		}
		// Fire collision with the left wall
		if(_fire.startingPosition(0) - _fire.dim(0)/2 - increaseFireLength/2 < -roomLength/2) {
			case2 = 1;
			//cout << "Case 2"<< endl;
		}
		// Fire collision with the front wall
		if (_fire.startingPosition(1) + _fire.dim(1)/2 + increaseFireWidth/2 > roomWidth/2) {
			case3 = 1;
			//cout << "Case 3"<< endl;
		}
		// Fire collision with the back wall
		if(_fire.startingPosition(1) - _fire.dim(1)/2 - increaseFireWidth/2 < -roomWidth/2) {
			case4 = 1;
			//cout << "Case 4"<< endl;
		}

		_fire.startingPosition(0) = _fire.startingPosition(0) - case1*increaseFireLength/2 + case2*increaseFireLength/2;
		_fire.startingPosition(1) = _fire.startingPosition(1) - case3*increaseFireWidth/2 + case4*increaseFireWidth/2;
		_fire.dim = Vector3d((2-case1-case2)*increaseFireWidth/2, (2-case3-case4)*increaseFireLength/2, 0) + _fire.dim;

		std::string _name = name;
		_name += std::to_string(num);   
		addFireToList(_name, _fire.startingPosition, Quaterniond(1, 0, 0, 0), _fire.dim, Vector4d(1, 0, 0, 1));

		//double radius = sqrt(pow(_fire.startingPosition(0) - _fire.dim(0), 2) + pow(_fire.startingPosition(1) - _fire.dim(1), 2));
		double radius = sqrt(pow(_fire.dim(0), 2) + pow(_fire.dim(1), 2));
		firesInfo.row(num - 1) << _fire.startingPosition(0), _fire.startingPosition(1), _fire.startingPosition(2),radius;

		num = num + 1;
	}

	// Send message through redis
	//redis_client.setEigenMatrixJSON(FIRE_INFO_KEY, Vector4d(1, 0, 0, 1));
	//redis_client.setEigenMatrixJSON(FIRE_INFO_KEY, Vector4d::Random());
	//redis_client.setEigenMatrixJSON(FIRE_INFO_KEY, firesInfo);
	graphics_redis.setEigenMatrixJSON(FIRE_INFO_KEY, firesInfo);
	std::cout << firesInfo << std::endl;
	// execute redis write callback
	//redis_client.executeWriteCallback(0);	

	loadFiresToScene(graphics);

}