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

#include "WorkItemExample.hpp"

#include <drivers/drv_hrt.h>

#define GPIO_PORT_SHIFT        (8)  /* Bits 8-10: port number */
#define GPIO_PORT_MASK         (7 << GPIO_PORT_SHIFT)
#define GPIO_PIN_SHIFT             (0)  /* Bits 0-4: port number */
#define GPIO_PIN_MASK              (31 << GPIO_PIN_SHIFT)

#define GPIO_IN_QUAD (GPIO_INPUT | ((4<< GPIO_PORT_SHIFT) & GPIO_PORT_MASK) | ((12 << GPIO_PIN_SHIFT) & GPIO_PIN_MASK))

// 7 pulses equal 1 revolution, and 1 revolution equals to a travel of 23 cm
volatile int pulseCount = 0;
volatile float quad_frequency = 0;
volatile float prevPulseMicros = 0;
volatile bool canPublish = false;

float lastFrequencies[5];
int pos = 0;

// function that calculates the average of a float array

static float arrayAverage(int nr, float array[10])
{
	float sum = 0;

	for (int i = 0; i < nr; i++) {
		sum += array[i];
	}

	return sum / (float)nr;
}

static int handleQuadratureEncIRQ(int irq, void *context, void *arg)
{
	pulseCount++;
	float currentMicros = hrt_absolute_time();
	float pulseInterval = currentMicros - prevPulseMicros;
	prevPulseMicros = currentMicros;

	if (pulseInterval > 0) {
		quad_frequency = (1000000.0f) / pulseInterval;
		canPublish = true;
	}

	//printf("da\n");
	return 0;
}

using namespace time_literals;

WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemExample::~WorkItemExample()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
	ScheduleOnInterval(100000_us); // 10000 us interval, 10000 Hz rate
	px4_arch_configgpio(GPIO_IN_QUAD);
	// // rising edge
	px4_arch_gpiosetevent(GPIO_IN_QUAD, true, false, true, &handleQuadratureEncIRQ, NULL);
	// PX4_INFO("WorkItemExample::init()\n");
	return true;
}

void WorkItemExample::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// DO WORK

	struct rev_counter_s rev_structure;

	rev_structure.timestamp = hrt_absolute_time();

	if (canPublish) {
		rev_structure.frequency = quad_frequency;
		rev_structure.pulse_counter = pulseCount;

		lastFrequencies[pos++] = rev_structure.frequency;

		if (pos >= 5) {
			pos = 0;
		}

		canPublish = false;

	} else {
		rev_structure.frequency = arrayAverage(5, lastFrequencies);
		rev_structure.pulse_counter = pulseCount;

		lastFrequencies[pos++] = 0;

		if (pos >= 5) {
			pos = 0;
		}
	}

	printf("Freq: %5.1f, Counter: %9d\n", (double)rev_structure.frequency, rev_structure.pulse_counter);
	_rev_counter_pub.publish(rev_structure);


	// // Example
	// // grab latest accelerometer data
	// _sensor_accel_sub.update();
	// const sensor_accel_s &accel = _sensor_accel_sub.get();


	// // Example
	// // publish some data
	// orb_test_s data{};
	// data.timestamp = hrt_absolute_time();
	// data.val = accel.device_id;
	// _orb_test_pub.publish(data);




	perf_end(_loop_perf);
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

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

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
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

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}
