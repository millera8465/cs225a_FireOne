/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <math.h>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_arm.urdf";

//state machine will be added later
enum State 
{
	MOVING_DOWN = 0, 
	SQUEEZE
};

// helper function 
double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}

int main() {

	 // initial state 
	int state = MOVING_DOWN;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	
	int dof = robot->dof();
	VectorXd initial_q = VectorXd::Zero(dof);
	initial_q = robot->_q;
	robot->updateModel();


	// prepare controller
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	
	Vector3d x_pos;
	Matrix3d x_ori;

// pose task for left hand
	std::string control_link = "link7";
	Vector3d control_point = Vector3d(0, 0.0, 0.17);
	auto posori_task_left_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task_left_hand->setDynamicDecouplingFull();

	posori_task_left_hand->_use_interpolation_flag = true;
	posori_task_left_hand->_use_velocity_saturation_flag = true;

	
	VectorXd posori_task_torques_left_hand = VectorXd::Zero(dof);
	posori_task_left_hand->_kp_pos = 200.0;
	posori_task_left_hand->_kv_pos = 20.0;
	posori_task_left_hand->_kp_ori = 200.0;
	posori_task_left_hand->_kv_ori = 20.0;

	// set two goal positions/orientations 
	robot->position(x_pos, control_link, control_point);
	robot->rotation(x_ori, control_link);
	posori_task_left_hand->_desired_position = x_pos + Vector3d(0.0, 0.0, -0.8);
	posori_task_left_hand->_desired_orientation = x_ori; 
	




// pose task for right hand 
	control_link = "link7r";
	auto posori_task_right_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task_right_hand->setDynamicDecouplingFull();

	posori_task_right_hand->_use_interpolation_flag = true;
	posori_task_right_hand->_use_velocity_saturation_flag = true;

	
	VectorXd posori_task_torques_right_hand = VectorXd::Zero(dof);
	posori_task_right_hand->_kp_pos = 200.0;
	posori_task_right_hand->_kv_pos = 20.0;
	posori_task_right_hand->_kp_ori = 200.0;
	posori_task_right_hand->_kv_ori = 20.0;


	// set two goal positions/orientations 
	robot->position(x_pos, control_link, control_point);
	robot->rotation(x_ori, control_link);
	posori_task_right_hand->_desired_position = x_pos + Vector3d(0.0, 0.0, -0.8);
	posori_task_right_hand->_desired_orientation = x_ori; 




	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = robot->_q;
	joint_task->_desired_position = q_init_desired;

	// Moving Base 
	VectorXd q_d(dof);
	q_d = initial_q;
	q_d(0) = 8;
	q_d(1) = 8;
	
	q_d(2) = -0.78539816339;

	// 0.25*180/M_PI, 30.0,-30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0, -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	// q_init_desired *= M_PI/180.0;
	// joint_task->_desired_position = q_d;

	// containers
	Vector3d ee_pos;
	Matrix3d ee_rot;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long counter = 0;

	runloop = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();

		/*VectorXd dq_d = VectorXd::Zero(dof);
		double nu, lu;
		double Vmax = 0.5;
		double Omax = 0.1;
		double kpj = 400; //set kpj
		double kvj = 80; //set kvj
		
		// cout << state <<endl; //for state detection in display
		dq_d = kpj/kvj*(q_d-robot->_q);// calculate dx_d
		nu = sat(Vmax/sqrt(pow(dq_d.coeff(0), 2) + pow(dq_d.coeff(1), 2)));
		lu = sat(Omax/abs(dq_d.coeff(2))); //defining lu for angular velocity saturation
		MatrixXd su = MatrixXd::Zero(dof, dof);
		su(0,0)=nu;
		su(1,1)=nu;
		su(2,2)=lu;

		command_torques = robot->_M*(-80*(robot->_dq-su*dq_d));*/

		if (state == MOVING_DOWN) {
			// calculate torques to fix the feet 
			N_prec.setIdentity();


			// calculate torques to move right hand
			posori_task_right_hand->updateTaskModel(N_prec);
			posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

			// calculate torques to move left hand
			N_prec = posori_task_right_hand->_N;
			posori_task_left_hand->updateTaskModel(N_prec);
			posori_task_left_hand->computeTorques(posori_task_torques_left_hand);


			// calculate torques to maintain joint posture
			N_prec = posori_task_left_hand->_N;
			joint_task->updateTaskModel(N_prec);
			joint_task->computeTorques(joint_task_torques);
			// calculate torques 
			command_torques = posori_task_torques_right_hand + posori_task_torques_left_hand + joint_task_torques;  // gravity compensation handled in sim
			Vector3d current_x_pos;
			robot->position(current_x_pos, control_link, control_point);
			Vector3d diff = current_x_pos - (x_pos + Vector3d(0.0, 0.0, -0.8));

			cout<< diff.transpose()<< endl;

			if (abs(diff(2)) <= 1e-07) {
				// update model
				robot->updateModel();
				robot->position(x_pos, control_link, control_point);
				robot->rotation(x_ori, control_link);
				posori_task_right_hand->_desired_position = x_pos + Vector3d(-1, 0.0, 0.0);
				posori_task_right_hand->_desired_orientation = x_ori;
				control_link = "link7";
				robot->position(x_pos, control_link, control_point);
				robot->rotation(x_ori, control_link);
				posori_task_left_hand->_desired_position = x_pos + Vector3d(1, 0.0, 0.0);
				posori_task_left_hand->_desired_orientation = x_ori;
				q_init_desired = robot->_q;
				joint_task->_desired_position = q_init_desired;
				state = SQUEEZE;

			}
		} else {

			// calculate torques to fix the feet 
			N_prec.setIdentity();


			// calculate torques to move right hand
			posori_task_right_hand->updateTaskModel(N_prec);
			posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

			// calculate torques to move left hand
			N_prec = posori_task_right_hand->_N;
			posori_task_left_hand->updateTaskModel(N_prec);
			posori_task_left_hand->computeTorques(posori_task_torques_left_hand);


			// calculate torques to maintain joint posture
			N_prec = posori_task_left_hand->_N;
			joint_task->updateTaskModel(N_prec);
			joint_task->computeTorques(joint_task_torques);
			// calculate torques 
			command_torques = posori_task_torques_right_hand + posori_task_torques_left_hand + joint_task_torques;  // gravity compensation handled in sim

		}

		
		
		

		// execute redis write callback
		redis_client.executeWriteCallback(0);	

		counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
