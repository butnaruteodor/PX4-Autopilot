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
	stop
} State;

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

NxpCupWork::NxpCupWork() :
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
	ScheduleOnInterval(30_ms); // 1000 us interval, 1000 Hz rate

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
	roverControl motorControl;
	static State CarState = Idle;
	static int nr_of_distance_readings = 0;
	static float last_time = 0;
	static float current_time = 0;

	struct vehicle_control_mode_s _control_mode {};

	_control_mode.flag_control_manual_enabled = false;
	_control_mode.flag_control_attitude_enabled = true;
	_control_mode.flag_control_velocity_enabled = false;
	_control_mode.flag_control_position_enabled = false;

	switch (CarState) {
	case Idle: {
			// Car is in idle state until the start button is pressed
			CarState = WaitForStart;
			break;
		}

	case WaitForStart: {
			pixy_sub.update();
			const pixy_vector_s &pixy = pixy_sub.get();

			// get control commands based on lane lines
			motorControl = raceTrack(pixy);

			// setup control structs
			att_sub.update();

			struct vehicle_attitude_s att = att_sub.get();
			struct vehicle_attitude_setpoint_s _att_sp {};

			// pre process the controls(convert steering from percent to angle, etc)
			NxpCupWork::roverSteerSpeed(motorControl, _att_sp, att);
			_att_sp.thrust_body[0] = 0.33f;
			_control_mode.timestamp = hrt_absolute_time();
			// publish control mode
			_att_sp_pub.publish(_att_sp);
			_control_mode_pub.publish(_control_mode);

			_att_sp.timestamp = hrt_absolute_time();

			if (detectStartLine() == true) {
				last_time = hrt_absolute_time();
				CarState = Driving;
			}

			break;
		}

	case Driving: {
			// Car is driving

			/* Get pixy data */
			pixy_sub.update();
			const pixy_vector_s &pixy = pixy_sub.get();

			// get control commands based on lane lines
			motorControl = raceTrack(pixy);

			// setup control structs
			att_sub.update();

			struct vehicle_attitude_s att = att_sub.get();
			struct vehicle_attitude_setpoint_s _att_sp {};

			// pre process the controls(convert steering from percent to angle, etc)
			NxpCupWork::roverSteerSpeed(motorControl, _att_sp, att);

			_control_mode.timestamp = hrt_absolute_time();
			// publish control mode
			_control_mode_pub.publish(_control_mode);

			_att_sp.timestamp = hrt_absolute_time();

			// publish controls
			_att_sp_pub.publish(_att_sp);

			current_time = hrt_absolute_time();

			if (detectStartLine() == true && current_time - last_time > 5000000) {
				static int nr = 0;
				nr++;
				printf("nr: %d\n", nr);

				if (nr == 1) {
					//printf("OBSTACLE DETECTION\n");
					CarState = ObstacleDetection;
					nr = 0;
				}
			}

			break;
		}

	case ObstacleDetection: {
			// Drive slow
			// static int contor = 0;
			/* Get pixy data */
			pixy_sub.update();
			const pixy_vector_s &pixy = pixy_sub.get();

			// get control commands based on lane lines
			motorControl = raceTrack(pixy);

			// setup control structs
			att_sub.update();

			struct vehicle_attitude_s att = att_sub.get();
			struct vehicle_attitude_setpoint_s _att_sp {};

			// pre process the controls(convert steering from percent to angle, etc)
			NxpCupWork::roverSteerSpeed(motorControl, _att_sp, att);

			_control_mode.timestamp = hrt_absolute_time();
			// publish control mode
			_control_mode_pub.publish(_control_mode);

			_att_sp.timestamp = hrt_absolute_time();

			// Throttle control of the rover


			// publish controls


			// code that counts the number of consecutive start lines using the start line detection function
			if (readDistance() <= 0.05f) {
				nr_of_distance_readings++;

			} else {
				nr_of_distance_readings = 0;
			}

			if (nr_of_distance_readings > 1) {
				for (int i = 0; i < 100000; i++) {
					brake(900);
				}

				//usleep(200);
				_att_sp.thrust_body[0] = 0.33f;
				_att_sp_pub.publish(_att_sp);
				usleep(10000);
				//_att_sp.thrust_body[0] = 0.75f;
				CarState = stop;
				break;
				//printf("Distance %f\n",static_cast<double>(readDistance()));
			}

			_att_sp_pub.publish(_att_sp);

			break;
		}

	case stop: {
			// Car is in idle state until the start button is pressed
			for (int i = 0; i < 1000; i++) {
				brake(1501);
			}

			CarState = WaitForStart;
			break;
		}

	}

	perf_end(_loop_perf);
}



int NxpCupWork::task_spawn(int argc, char *argv[])
{
	NxpCupWork *instance = new NxpCupWork();

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
