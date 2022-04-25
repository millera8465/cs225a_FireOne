#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <fstream>
#include <string>

// definitions for terminal input parsing 
#define QUESTION_1A   	11
#define QUESTION_1C   	13
#define QUESTION_2D   	24
#define QUESTION_2E   	25
#define QUESTION_2F   	26
#define QUESTION_2G   	27
#define QUESTION_3    	30
#define QUESTION_4A   	41
#define QUESTION_4B   	42
#define QUESTION_5A   	51
#define QUESTION_5B   	52
#define QUESTION_5C   	53

// state machine
#define MOVING_DOWN		0
#define APPLY_FORCE		1
#define MOVING_CIRCLE	2


#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
const std::string EE_FORCE_KEY = "sai2::cs225a::panda_robot::sensors::force";
const std::string EE_MOMENT_KEY = "sai2::cs225a::panda_robot::sensors::moment";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

// helper function 
double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}

int main(int argc, char* argv[]) {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string ee_link_name = "link7";
	const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	// MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, ee_link_name, pos_in_ee_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	// robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// initialize force and moment on end-effector
	Eigen::Vector3d force, moment;
	force = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
	moment = redis_client.getEigenMatrixJSON(EE_MOMENT_KEY);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");

	// read terminal input, i.e. ./hw3 51
	int controller_number = atoi(argv[1]);  

	string filename;
	if(controller_number == QUESTION_1A)
		filename = "../../hw3_solution/data_files/question_1a.txt";
	else if(controller_number == QUESTION_1C)
		filename = "../../hw3_solution/data_files/question_1c.txt";
	else if(controller_number == QUESTION_2D)
		filename = "../../hw3_solution/data_files/question_2d.txt";
	else if(controller_number == QUESTION_2E)
		filename = "../../hw3_solution/data_files/question_2e.txt";
	else if(controller_number == QUESTION_2F)
		filename = "../../hw3_solution/data_files/question_2f.txt";
	else if(controller_number == QUESTION_2G)
		filename = "../../hw3_solution/data_files/question_2g.txt";
	else if(controller_number == QUESTION_3)
		filename = "../../hw3_solution/data_files/question_3.txt";
	else if(controller_number == QUESTION_4A)
		filename = "../../hw3_solution/data_files/question_4a.txt";
	else if(controller_number == QUESTION_4B)
		filename = "../../hw3_solution/data_files/question_4b.txt";
	else if(controller_number == QUESTION_5A)
		filename = "../../hw3_solution/data_files/question_5a.txt";
	else if(controller_number == QUESTION_5B)
		filename = "../../hw3_solution/data_files/question_5b.txt";
	else if(controller_number == QUESTION_5C)
		filename = "../../hw3_solution/data_files/question_5c.txt";

	ofstream data_file;
	data_file.open(filename);

	// for state machine
	unsigned state = MOVING_DOWN;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// read force sensor from redis
		force = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
		moment = redis_client.getEigenMatrixJSON(EE_MOMENT_KEY);

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1A)
		{
			Vector3d x, x_d, dx, F;
			VectorXd g(dof), joint_task_torque(dof);

			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;

			// update Jv
			// update Lambda
			// update N
			// update x
			// update dx
			// update g

			// set x_d

			// calculate joint_task_torque

			// calculate F

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\n';
			}
		}
		else if(controller_number == QUESTION_1C)
		{
			Vector3d x, x_d, dx, dx_d, ddx_d, F;
			VectorXd g(dof), b(dof), joint_task_torque(dof);

			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;

			// update Jv
			// update Lambda
			// update N
			// update x
			// update dx
			// update g

			// set x_d
			// set dx_d
			// set ddx_d

			// calculate joint_task_torque

			// calculate F

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\n';
			}
		}

		// ---------------------------  question 2 ---------------------------------------
		else if(controller_number == QUESTION_2D)
		{
			// part 1
			VectorXd g(dof), Gamma_damp(dof), q_high(dof), q_low(dof);
			Vector3d x, x_d, dx, F;

			double kp = 100;
			double kv = 20;
			double kdamp; // set kdamp value

			// update Jv
			// update Lambda
			// update N
			// update x
			// update dx
			// update g

			// set q_low
			// set q_high
			// set x_d

			// calculate F
			// calculate Gamma_damp

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << robot->_q(3) << '\t' << q_low(3) << '\t' << q_high(3) << '\t';
				data_file << robot->_q(5) << '\t' << q_low(5) << '\t' << q_high(5) << '\n';
			}
		}
		else if(controller_number == QUESTION_2E)
		{
			// part 2
			Vector3d x, x_d, dx, F;
			VectorXd g(dof), Gamma_damp(dof), Gamma_mid(dof), q_high(dof), q_low(dof);

			double kp = 100;
			double kv = 20;
			double kdamp; // set kdamp value
			double kmid; // set kmid value

			// update Jv
			// update Lambda
			// update N
			// update x
			// update dx
			// update g

			// set q_low
			// set q_high
			// set x_d

			// calculate F
			// calculate Gamma_damp
			// calculate Gamma_mid

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << robot->_q(3) << '\t' << q_low(3) << '\t' << q_high(3) << '\t';
				data_file << robot->_q(5) << '\t' << q_low(5) << '\t' << q_high(5) << '\n';
			}
		}
		else if(controller_number == QUESTION_2F)
		{
			Vector3d x, x_d, dx, F;
			VectorXd g(dof), Gamma_damp(dof), Gamma_mid(dof), q_high(dof), q_low(dof);
			
			double kp = 100;
			double kv = 20;
			double kdamp; // set kdamp value
			double kmid; // set kmid value

			// update Jv
			// update Lambda
			// update N
			// update x
			// update dx
			// update g

			// set q_low
			// set q_high
			// set x_d

			// calculate F
			// calculate Gamma_damp
			// calculate Gamma_mid

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << robot->_q(3) << '\t' << q_low(3) << '\t' << q_high(3) << '\t';
				data_file << robot->_q(5) << '\t' << q_low(5) << '\t' << q_high(5) << '\n';
			}
		}
		else if (controller_number == QUESTION_2G)
		{
			Vector3d x, x_d, dx, F;
			VectorXd g(dof), Gamma_damp(dof), Gamma_mid(dof), q_high(dof), q_low(dof);

			double kp = 100;
			double kv = 20;
			double kdamp; // set kdamp value
			double kmid; // set kmid value

			// update Jv
			// update Lambda
			// update N
			// update x
			// update dx
			// update g

			// set q_low
			// set q_high
			// set x_d

			// calculate F
			// calculate Gamma_damp
			// calculate Gamma_mid

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << robot->_q(3) << '\t' << q_low(3) << '\t' << q_high(3) << '\t';
				data_file << robot->_q(5) << '\t' << q_low(5) << '\t' << q_high(5) << '\n';
			}

		}

		// ---------------------------  question 3 ---------------------------------------
		else if(controller_number == QUESTION_3)
		{
			MatrixXd J(6, dof), Lambda0(6, 6), N(dof,dof);
			Vector3d x, x_d, dx, w, delta_phi;
			VectorXd F(6), g(dof);
			Matrix3d R, R_d;

			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;		

			// update J
			// update Lambda0
			// update N
			// update x
			// update dx
			// update w
			// update g
			// update R

			// set R_d
			// set x_d

			// calculate delta_phi
			// calculate pos_d, position desired
			// calculate F

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << delta_phi(0) << '\t' << delta_phi(1) << '\t' << delta_phi(2) << '\n';
			}

		}

		// ---------------------------  question 4 ---------------------------------------
		else if(controller_number == QUESTION_4A)		
		{
			// part 1
			VectorXd F(3), g(dof), joint_task_torque(dof);
			Vector3d x, x_d, dx;

			double Vmax = 0.1;

			double kp = 200;
			double kv; // set kv value
			double kpj; // set kpj value
			double kvj; // set kvj value

			// update Jv
			// update Lambda
			// update N
			// update x
			// update dx
			// update g

			// set x_d
			// set q_d, q desired to initial q 

			// calculate F
			// calculate joint_task_torque

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << dx(0) << '\t' << dx(1) << '\t' << dx(2) << '\t' << dx.norm() << '\t' << Vmax <<'\n';
			}
		}
		else if (controller_number == QUESTION_4B)
		{
			// part 2			
			VectorXd g(dof), joint_task_torque(dof);
			Vector3d x, x_d, dx_d, dx, F;

			double Vmax = 0.1;

			double kp = 200;
			double kv; // set kv value
			double kpj; // set kpj value
			double kvj; // set kvj value

			// update Jv
			// update Lambda
			// update N
			// update x
			// update dx
			// update g

			// set x_d
			// calculate dx_d
			// calculate \nu
			// calculate F

			// set q_d, q desired to initial q 
			// calculate joint_task_torque

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << dx(0) << '\t' << dx(1) << '\t' << dx(2) << '\t' << dx.norm() << '\t' << Vmax <<'\n';
			}
		}
		// ---------------------------  question 5 ---------------------------------------
		else if(controller_number == QUESTION_5A)
		{
			MatrixXd J(6, dof), Lambda0(6, 6);
			Vector3d x, x_d, dx_d, dx, ddx_d, w, delta_phi, pos_d_x, pos_d_w;
			VectorXd F(6), g(dof), b(dof), joint_task_torque(dof);
			Matrix3d R, R_d;

			double Vmax = 0.1;
			double nu;

			double kp = 200;
			double kv = 24;
			double kpj = 50;
			double kvj = 14;

			// update J
			// update Lambda0
			// update N
			// update x
			// update dx
			// update w
			// update g
			// update R

			// nested function for state machine
			auto moving_down  = [&] () {
				// set x_d, vertically moving down
				// calculate dx_d
				// calculate nu
				// calculate pos_d_x

				// if force reading is NOT 0, change state to MOVING_CIRCLE
			};

			// nested function for state machine
			auto moving_circle = [&] () {
				double s_time = time/3; // slower time rate
				// set x_d, circular motion
				// calculate dx_d, circular motion
				// calculate ddx_d, circular motion
				// calculate pos_d_x
			};

			// state switch for state machine
			switch(state) {
				case MOVING_DOWN :
					moving_down();
					break;
				case MOVING_CIRCLE :
					moving_circle();
					break;
				default:
					if (controller_counter%500 == 0) std::cout << "No state set" << std::endl;
			}

			// set R_d
			// calculate delta_phi
			// calculate pos_d_w
			
			// initialize and set pos_d

			// calculate F
			// calculate joint_task_torque

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << force(0) << "\t" << force(1) << "\t" << force(2) << "\n";
			}
		}
		else if(controller_number == QUESTION_5B)
		{
			MatrixXd J(6, dof), Lambda0(6, 6);
			Vector3d x, x_d, dx_d, dx, ddx_d, w, delta_phi, pos_d_x, pos_d_w;
			VectorXd F(6), g(dof), b(dof), joint_task_torque(dof);
			Matrix3d R, R_d;

			double Vmax = 0.1;
			double nu;

			double kp = 200;
			double kv = 24;
			double kpj = 50;
			double kvj = 14;

			// update J
			// update Lambda0
			// update N
			// update x
			// update dx
			// update w
			// update g
			// update R

			// nested function for state machine
			auto moving_down  = [&] () {
				// set x_d, vertically moving down
				// calculate dx_d
				// calculate nu
				// calculate pos_d_x

				// if force reading is NOT 0, change state to MOVING_CIRCLE
			};

			// nested function for state machine
			auto moving_circle = [&] () {
				double s_time = time/3; // slower time rate
				// set x_d, circular motion
				// calculate dx_d, circular motion
				// calculate ddx_d, circular motion
				// calculate pos_d_x
			};

			// state switch for state machine
			switch(state) {
				case MOVING_DOWN :
					moving_down();
					break;
				case MOVING_CIRCLE :
					moving_circle();
					break;
				default:
					if (controller_counter%500 == 0) std::cout << "No state set" << std::endl;
			}

			// set R_d
			// calculate delta_phi
			// calculate pos_d_w
			
			// initialize and set pos_d

			// calculate F
			// calculate joint_task_torque

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << force(0) << "\t" << force(1) << "\t" << force(2) << "\n";
			}

		}

		else if(controller_number == QUESTION_5C)
		{
			MatrixXd J(6, dof), Lambda0(6, 6);
			Vector3d x, x_d, dx_d, dx, ddx_d, w, delta_phi, pos_d_x;
			VectorXd F(6), F_desired(6), g(dof), b(dof), joint_task_torque(dof);
			Matrix3d R, R_d;

			double Vmax = 0.1;
			double nu;

			double kp = 200;
			double kv = 24;
			double kpj = 50;
			double kvj = 14;

			// update J
			// update Jv
			// update Lambda0
			// update Lambda
			// update N
			// update x
			// update dx
			// update w
			// update g
			// update b
			// update R

			// nested function for state machine
			auto moving_down  = [&] () {
				// set x_d, vertically moving down
				// calculate dx_d
				// calculate nu
				// calculate pos_d_x

				// if force reading is NOT 0, change state to APPLY_FORCE
			};

			// nested function for state machine
			auto apply_force = [&] () {
				// set x_d, vertically moving down
				// calculate dx_d
				// calculate nu
				// calculate pos_d_x

				// if force applied is >= 10, change state to MOVING_CIRCLE
			};

			// nested function for state machine
			auto moving_circle = [&] () {
				double s_time = time/3; // slower time rate
				// set x_d, circular motion
				// calculate dx_d, circular motion
				// calculate ddx_d, circular motion
				// calculate pos_d_x
			};

			switch(state) {
				case MOVING_DOWN :
					moving_down();
					break;
				case APPLY_FORCE :
					apply_force();
					break;
				case MOVING_CIRCLE :
					moving_circle();
					break;
				default:
					if (controller_counter%500 == 0) std::cout << "No state set" << std::endl;
			}

			// set R_d
			// calculate delta_phi
			// calculate pos_d_w
			
			// initialize and set pos_d

			// initialize and set section matrix, selc_omh
			// initialize and set section matrix, selc_omh_bar

			// set F_desired

			// initialize and set dx_vw (dx and w)

			// calculate joint_task_torque
			
			if (state == MOVING_CIRCLE) {
				// calculate F
			} else {
				// calculate F
			}

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << force(0) << "\t" << force(1) << "\t" << force(2) << "\n";
			}
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	data_file.close();

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
