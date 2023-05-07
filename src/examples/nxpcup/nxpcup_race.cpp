#include "nxpcup_race.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/battery_status.h>

const int PREVIOUS_STEERING_VALUES = 5;
// const float SMOOTHING_FACTOR = 0.1f;
// const float STEERING_GAIN = 1.5f;

float SPEED_MAX = 0.15f;
float SPEED_MIN = 0.15f;
//when are suden
//const float KP_MIN = 1.0f;
float KP = 2.0f;
float KD = 0.0f;
float KI = 0.0000000f;

static float last_error = 0;
static float integral = 0; // Add the integral term
static uint64_t last_time = 0;


const float BATTERY_VOLTAGE_MIN = 6.2f;
const float BATTERY_VOLTAGE_MAX = 8.4f;


uint8_t get_num_vectors(Vector &vec1, Vector &vec2)
{

	uint8_t numVectors = 0;

	if (!(vec1.m_x0 == 0 && vec1.m_x1 == 0 && vec1.m_y0 == 0 && vec1.m_y1 == 0)) {
		numVectors++;
	}

	if (!(vec2.m_x0 == 0 && vec2.m_x1 == 0 && vec2.m_y0 == 0 && vec2.m_y1 == 0)) {
		numVectors++;
	}

	return numVectors;
}

Vector copy_vectors(const pixy_vector_s &pixy, uint8_t num)
{
	Vector vec;

	if (num == 1) {
		vec.m_x0 = pixy.m0_x0;
		vec.m_x1 = pixy.m0_x1;
		vec.m_y0 = pixy.m0_y0;
		vec.m_y1 = pixy.m0_y1;
	}

	if (num == 2) {
		vec.m_x0 = pixy.m1_x0;
		vec.m_x1 = pixy.m1_x1;
		vec.m_y0 = pixy.m1_y0;
		vec.m_y1 = pixy.m1_y1;
	}

	return vec;
}

roverControl raceTrack(const pixy_vector_s &pixy, float kp, float kd, float ki, float &setpoint)
{
	KP = kp;
	KD = kd;
	KI = ki;

	printf("kp = %f, kd = %f, ki = %f\n", (double)kp, (double)kd, (double)ki);
	//printf("kp = %f, kd = %f, speed_max = %f, speed_min = %f\n", (double)kp, (double)kd, (double)speed_max,
	//(double)speed_min);
	// static int battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	// // Declare a battery_status_s structure to store the battery data
	// static struct battery_status_s battery_status;
	// bool updated = false;
	// orb_check(battery_status_sub, &updated);

	// static float battery_voltage = 0.0f;
	// static float scaled_speed = 0.0f;

	// if (battery_voltage > 8.4f) {
	// 	battery_voltage = 8.4f;
	// }

	// if (updated) {
	// 	orb_copy(ORB_ID(battery_status), battery_status_sub, &battery_status);

	// 	// Access the battery status data
	// 	battery_voltage = battery_status.voltage_v;
	// 	//printf("Battery voltage: %.2f V\n", double(battery_voltage));
	// }

	// // Scale the speed according to battery voltage
	// if (battery_voltage >= BATTERY_VOLTAGE_MIN && battery_voltage <= BATTERY_VOLTAGE_MAX) {
	// 	float voltage_range = BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN;
	// 	float speed_range = SPEED_MAX - SPEED_MIN;
	// 	scaled_speed = SPEED_MIN + (((BATTERY_VOLTAGE_MAX - battery_voltage) / voltage_range) * speed_range);

	// } else {
	// 	scaled_speed = SPEED_MAX;
	// }


	Vector main_vec;
	Vector vec1 = copy_vectors(pixy, 1);
	Vector vec2 = copy_vectors(pixy, 2);
	uint8_t frameWidth = 79;
	//uint8_t frameHeight = 52;
	roverControl control{};
	float x; // calc gradient and position of main vector
	uint8_t num_vectors = get_num_vectors(vec1, vec2);

	static float previous_steering_values[PREVIOUS_STEERING_VALUES] = {0};
	static int steering_index = 0;

	// static float filtered_steering = 0.0f;
	//printf("num_vectors %u\n", num_vectors);

	switch (num_vectors) {
	case 0: {
			float sum = 0.0f;

			for (int i = 0; i < PREVIOUS_STEERING_VALUES; i++) {
				sum += previous_steering_values[i];
			}

			control.steer = sum / PREVIOUS_STEERING_VALUES;
			control.steer *= 0.3f;

			//control.steer *= 0.0f;
			if (control.steer > 1) {
				control.steer = 1;

			} else if (control.steer < -1) {
				control.steer = -1;
			}

			// control.speed = 0.03f;
			//
			// previous_steering_values[steering_index++] = control.steer;
			// if (steering_index >= PREVIOUS_STEERING_VALUES)
			// {
			// 	steering_index = 0;
			// }
		}
		break;

	case 2: {
			main_vec.m_x1 = (vec1.m_x1 + vec2.m_x1) / 2;
			main_vec.m_x0 = (vec1.m_x0 + vec2.m_x0) / 2;
			main_vec.m_y1 = (vec1.m_y1 + vec2.m_y1) / 2;
			main_vec.m_y0 = (vec1.m_y0 + vec2.m_y0) / 2;

			x = (float)(main_vec.m_x1 - main_vec.m_x0) / (float)frameWidth;
			//y = (float)(main_vec.m_y1 - main_vec.m_y0) / (float)frameHeight;

			// Compute the error value based on the horizontal deviation from the center
			float error = x - 0.5f;

			// Calculate the time difference for the PID control
			uint64_t current_time = hrt_absolute_time();
			float dt = (current_time - last_time) / 1e6f; // Convert time difference to seconds
			last_time = current_time;

			// Calculate the integral and derivative terms
			integral += error * dt;
			float derivative = (error - last_error) / dt;
			last_error = error;

			// Calculate the steering control output using the PID formula
			control.steer = -(KP * error + KI * integral + KD * derivative);

			// Ensure the control output is within the range [-1, 1]
			if (control.steer > 1) {
				control.steer = 1;

			} else if (control.steer < -1) {
				control.steer = -1;
			}

			// Update the steering history
			previous_steering_values[steering_index++] = control.steer;

			if (steering_index >= PREVIOUS_STEERING_VALUES) {
				steering_index = 0;
			}
		}
		break;

	// case 2: {
	// 		main_vec.m_x1 = (vec1.m_x1 + vec2.m_x1) / 2;
	// 		main_vec.m_x0 = (vec1.m_x0 + vec2.m_x0) / 2;
	// 		main_vec.m_y1 = (vec1.m_y1 + vec2.m_y1) / 2;
	// 		main_vec.m_y0 = (vec1.m_y0 + vec2.m_y0) / 2;

	// 		x = (float)(main_vec.m_x1 - main_vec.m_x0) / (float)frameWidth;
	// 		y = (float)(main_vec.m_y1 - main_vec.m_y0) / (float)frameHeight;

	// 		// avoid dividing by 0(the else shouldnt happen because we filter horizontal lines)
	// 		if (y > 0.01f || y < -0.01f) {
	// 			control.steer = x / y; // Gradient of the main vector

	// 		} else {
	// 			// go straight
	// 			control.steer = 0;
	// 		}

	// 		// to be sure its in the range [-1,1]
	// 		if (control.steer > 1) {
	// 			control.steer = 1;

	// 		} else if (control.steer < -1) {
	// 			control.steer = -1;
	// 		}

	// 		// Adjust the steering gain based on the speed
	// 		// control.steer *= STEERING_GAIN;
	// 		// control.speed = SPEED_NORMAL;
	// 		control.steer = -control.steer;
	// 		previous_steering_values[steering_index++] = control.steer;

	// 		if (steering_index >= PREVIOUS_STEERING_VALUES) {
	// 			steering_index = 0;
	// 		}

	// 		// control.speed = 0.05f;
	// 	}
	// 	break;



	default: {
			Vector single_vec;

			if (vec1.m_x0 != 0 || vec1.m_y0 != 0 || vec1.m_x1 != 0 || vec1.m_y1 != 0) {
				single_vec = vec1;
				// printf("vec1\n");

			} else {
				single_vec = vec2;
				// printf("vec2\n");
			}


			uint64_t current_time = hrt_absolute_time();
			float dt = (current_time - last_time) / 1e6f; // Convert time difference to seconds
			last_time = current_time;

			float single_vec_slope = (float)(single_vec.m_y1 - single_vec.m_y0) / (float)(single_vec.m_x1 - single_vec.m_x0);
			float single_vec_midpoint_x = (float)(single_vec.m_x1 + single_vec.m_x0) / 2;

			float base_lane_width = 78.0f;
			//float modulation = lane_width_modulation(single_vec_slope);
			float estimated_lane_width = base_lane_width; //* modulation;
			float estimated_other_boundary_midpoint_x;

			if (single_vec_slope > 0) {
				estimated_other_boundary_midpoint_x = single_vec_midpoint_x + estimated_lane_width;

			} else  {
				estimated_other_boundary_midpoint_x = single_vec_midpoint_x - estimated_lane_width;
			}



			float lane_center_x = (single_vec_midpoint_x + estimated_other_boundary_midpoint_x) / 2;
			float error = (lane_center_x / (float)frameWidth) - 0.5f;
			integral += error * dt;
			// Implement proportional-derivative control for steering


			// Calculate the steering angle using proportional-derivative control
			float derivative = (error - last_error) / dt;
			control.steer = -(KP * error + KI * integral + KD * derivative);

			// Update last_error
			last_error = error;

			// Make sure the steering angle is within the range [-1, 1]
			if (control.steer > 1) {
				control.steer = 1;

			} else if (control.steer < -1) {
				control.steer = -1;
			}

			previous_steering_values[steering_index++] = control.steer;

			if (steering_index >= PREVIOUS_STEERING_VALUES) {
				steering_index = 0;
			}

			break;
		}




		//control.speed = 0.33f;

	}

	setpoint = 60;
	//control.speed = (double)scaled_speed;
	//  printf("speed = %f    battery = %f\n", static_cast<double>(control.speed), static_cast<double>(battery_voltage));
	//control.speed = 1;
	//control.steer =.5f;

	return control;

}
