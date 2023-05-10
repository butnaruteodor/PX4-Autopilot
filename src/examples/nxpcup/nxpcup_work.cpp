/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "nxpcup_work.hpp"

#include <drivers/drv_hrt.h>


using namespace time_literals;

typedef enum {
	Idle,
	WaitForStart,
	Driving,
	ObstacleDetection,
	Stop
} State;

// PID controller parameters
float dt = 0.01; // 10 ms loop interval in seconds

// PID controller internal states
float integral = 0.0;
float previous_error = 0.0;

#if 1
float NxpCupWork::calculate_pid(float setpoint, float measurement, float min_output, float max_output,
				float current_timest)
{
	static float last_timestep = 0;

	float error = setpoint - measurement;
	// in seconds
	dt = static_cast<double>((double)(current_timest - last_timestep) / 1e6);

	float p_term = kp * error;

	float i_term_candidate = integral + error * ki * dt;

	float d_term = kd * (error - previous_error) / dt;
	previous_error = error;

	float output_candidate = p_term + i_term_candidate + d_term;

	// Apply output saturation limits and integral anti-windup
	if (output_candidate > max_output) {
		output_candidate = max_output;
		i_term_candidate = integral; // Prevent integral windup

	} else if (output_candidate < min_output) {
		output_candidate = min_output;
		i_term_candidate = integral; // Prevent integral windup
	}

	integral = i_term_candidate;
	last_timestep = current_timest;

	//printf("SP: %4f, M: %4.2f, E: %4.2f\n", (double)setpoint,
	// (double)measurement,
	// (double)error);
	return output_candidate;
}
#else
float NxpCupWork::calculate_pid(float setpoint, float measurement, float min_output, float max_output,
				float current_timest)
{
	float error = setpoint - measurement;
	printf("SP: %4f, M: %4.2f, E: %4.2f\n", (double)setpoint,
	       (double)measurement,
	       (double)error);
	return kp * error;
}
#endif
void brake(unsigned long pwm_value)
{
	const char *dev = "/dev/pwm_output0";
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return;
	}

	int ret = px4_ioctl(fd, PWM_SERVO_SET(3), pwm_value);

	if (ret != OK) {
		PX4_ERR("failed setting brake");
	}

	if (fd >= 0) {
		px4_close(fd);
	}
}
NxpCupWork::NxpCupWork():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

NxpCupWork::~NxpCupWork()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool NxpCupWork::init()
{
	ScheduleOnInterval(20_ms); // 1000 us interval, 1000 Hz rate

	struct vehicle_control_mode_s _control_mode {};

	_control_mode.flag_control_manual_enabled = false;
	_control_mode.flag_control_attitude_enabled = true;
	_control_mode.flag_control_velocity_enabled = false;
	_control_mode.flag_control_position_enabled = false;

	_control_mode.timestamp = hrt_absolute_time();
	_control_mode_pub.publish(_control_mode);

	return true;
}

void NxpCupWork::roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp, vehicle_attitude_s &att)
{
	// Converting steering value from percent to euler angle
	control.steer *= -60.0f; //max turn angle 60 degree
	control.steer *= (float)3.14159 / 180; // change to radians

	// Get current attitude quaternion
	matrix::Quatf current_qe{att.q[0], att.q[1], att.q[2], att.q[3]};

	// Converting the euler angle into a quaternion for vehicle_attitude_setpoint
	matrix::Eulerf euler{0.0, 0.0, control.steer};
	matrix::Quatf qe{euler};

	// Create new quaternion from the difference of current vs steering
	matrix::Quatf new_qe;
	new_qe = current_qe * qe.inversed();

	// Throttle control of the rover
	_att_sp.thrust_body[0] = control.speed;

	// Steering control of the Rover
	new_qe.copyTo(_att_sp.q_d);

}
bool NxpCupWork::detectStartLine()
{
	/* Get start line struct data */
	start_line_detected_sub.update();
	const start_line_detected_s &start_line_detected = start_line_detected_sub.get();

	// if start line was detected return true else false
	if (start_line_detected.start_line_detected == true) {
		return true;

	} else {
		return false;
	}
}
float NxpCupWork::readDistance()
{
	/* Get start line struct data */
	distance_sub.update();
	const distance_sensor_s &distance_sensor = distance_sub.get();

	return distance_sensor.current_distance;
}
void NxpCupWork::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	//printf("merge\n");
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// DO WORK
	static float control_output = 0.0f;
	roverControl motorControl;

	static State CarState = ObstacleDetection;
	static bool isBraking = false;
	static int nr_of_distance_readings = 0;
	// used for start line detection debouncing
	static float last_time = 0;
	static float current_time = 0;

	rev_sub.update();
	const rev_counter_s &rev_s = rev_sub.get();

	pixy_sub.update();
	const pixy_vector_s &pixy = pixy_sub.get();

	att_sub.update();


	struct vehicle_attitude_s att = att_sub.get();
	struct vehicle_attitude_setpoint_s _att_sp {};
	static int div = 0;

	if (div++ > 4) {
		div = 0;
		printf("Distance %f\n", static_cast<double>(readDistance()));
	}

	switch (CarState) {
	case Idle: {
			// Car is in idle state until the start button is pressed
			CarState = WaitForStart;
			break;
		}

	case WaitForStart: {
			// speed is 0
			// get control commands based on lane lines
			motorControl = raceTrack(pixy, this->KP, this->KD, setp);
			setp = 0;

			if (detectStartLine() == true) {
				last_time = hrt_absolute_time();
				CarState = Driving;
			}

			break;
		}

	case Driving: {
			// Car is driving
			//setp = 80;

			// get control commands based on lane lines
			motorControl = raceTrack(pixy, this->KP, this->KD, setp);

			// pre process the controls(convert steering from percent to angle, etc)
			NxpCupWork::roverSteerSpeed(motorControl, _att_sp, att);

			current_time = hrt_absolute_time();

			if (detectStartLine() == true && current_time - last_time > 5000000) {
				static int nr = 0;
				nr++;

				if (nr == 1) {
					//printf("OBSTACLE DETECTION\n");
					CarState = ObstacleDetection;
					nr = 0;
				}
			}

			break;
		}

	case ObstacleDetection: {
			// Drive slowly until the obstacle is detected
			isBraking = false;
			// get control commands based on lane lines
			motorControl = raceTrack(pixy, this->KP, this->KD, setp);
			//setp = 50;
			// pre process the controls(convert steering from percent to angle, etc)
			NxpCupWork::roverSteerSpeed(motorControl, _att_sp, att);

			// code that counts the number of consecutive start lines using the start line detection function
			if (readDistance() <= 0.05f) {
				nr_of_distance_readings++;

			} else {
				nr_of_distance_readings = 0;
			}

			// printf("Distance %f\n", static_cast<double>(readDistance()));

			if (nr_of_distance_readings > 1) {

				CarState = Stop;
				//isBraking = true;
			}

			break;
		}

	case Stop: {
			printf("Frana\n");

			// for (int i = 0 ; i < 100; i++) {
			// 	_att_sp.thrust_body[0] = -1.0f;
			// 	_att_sp_pub.publish(_att_sp);
			// }

			setp = 0;
			// if (rev_s.frequency < 0.00001f) {
			// 	CarState = ObstacleDetection;
			// 	//isBraking = false;
			// 	//usleep(10000);
			// }

			//CarState = Driving;
			//isBraking = true;
			break;
		}

	}

	// Here we calculate the control output based on the setpoint and the current measurement
	float current_measurement = rev_s.frequency;

	if (isBraking == false) {
		control_output = calculate_pid(setp, current_measurement, -1.0f, 0.5f, hrt_absolute_time());

		static float prev_printing_time = 0;

		if (hrt_absolute_time() - prev_printing_time > 50000) {
			prev_printing_time = hrt_absolute_time();
			//printf("Setpoint: %4f, Current frequency: %4.2f, Control output: %4.2f\n", (double)setp,
			//(double)current_measurement,
			//(double)control_output);
		}

		_att_sp.thrust_body[0] = control_output;
		_att_sp.timestamp = hrt_absolute_time();

		_att_sp_pub.publish(_att_sp);
	}



	perf_end(_loop_perf);
}


int NxpCupWork::task_spawn(int argc, char *argv[])
{
	NxpCupWork *instance = new NxpCupWork();

	if (argc >= 3) {
		instance->KP = atof(argv[1]);
		instance->KD = atof(argv[2]);
		instance->setp = atof(argv[3]);
		//printf("argv[4] = %f, argv[5] = %f, argv[6] = %f, argv[7] = %f\n", (double)instance->KP, (double)instance->KD,
		//(double)instance->SPEED_MAX, (double)instance->SPEED_MIN);
	}

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int NxpCupWork::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int NxpCupWork::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NxpCupWork::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int nxpcup_work_main(int argc, char *argv[])
{
	return NxpCupWork::main(argc, argv);
}
