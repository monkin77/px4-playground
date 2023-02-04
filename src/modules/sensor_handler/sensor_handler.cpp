/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "sensor_handler.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>

int SensorHandlerModule::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int SensorHandlerModule::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int SensorHandlerModule::task_spawn(int argc, char *argv[])
{
	// This  function is called upon module start and will spawn the task that will execute the "run" method?
	// TODO: Investigate this parameters
	_task_id = px4_task_spawn_cmd("sensor_handler_task",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	// Log a note that we started the task.
	PX4_INFO("Sensor Handler task started");

	return 0;
}

SensorHandlerModule *SensorHandlerModule::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	SensorHandlerModule *instance = new SensorHandlerModule(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

SensorHandlerModule::SensorHandlerModule(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void SensorHandlerModule::run()
{
	// This is the function that is executed by the task?

	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int sensor_gps_sub = orb_subscribe(ORB_ID(sensor_gps));
	int sensor_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
	int sensor_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	int sensor_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	int cpuload_sub = orb_subscribe(ORB_ID(cpuload));

	// Update the sensors_sub property of the class with the subscription IDs
	this->sensors_sub[0] = sensor_combined_sub; this->sensors_sub[1] = sensor_gps_sub; this->sensors_sub[2] = sensor_baro_sub;
	this->sensors_sub[3] = sensor_gyro_sub; this->sensors_sub[4] = sensor_mag_sub; this->sensors_sub[5] = cpuload_sub;

	// Set up each of the poll structures
	for (int i = 0; i < 6; i++) {
		this->poll_fds[i].fd = sensors_sub[i];
		this->poll_fds[i].events = POLLIN;
	}

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data. If there is new data, pret will be positive?
		int pret = px4_poll(this->poll_fds, (sizeof(this->poll_fds) / sizeof(this->poll_fds[0])), 1000);
		PX4_INFO("Polling with pret = %d", pret);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else {
			// pret > 0 => there is a file descriptor with new data. So we check all FDs
			this->handle_polling_results();
		}

		// TODO: Check what this does
		parameters_update();

		// Sleep 3s to make it more readable
		px4_usleep(3 * 1000000);
	}

	orb_unsubscribe(sensor_combined_sub);
}

void SensorHandlerModule::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int SensorHandlerModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "sensor_handler");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int sensor_handler_module_main(int argc, char *argv[])
{
	return SensorHandlerModule::main(argc, argv);
}

// Custom Class methods

/**
 * @brief This function is called when a new message is received from the subscribed topic
*/
void SensorHandlerModule::handle_polling_results() {		
	// Confirm that the class fields are initialized
	if (this->sensors_sub[0] == -1) {
		PX4_ERR("Invalid behavior. Sensor subscriptions not initialized.");
		return;
	}

	if (this->poll_fds[0].revents & POLLIN) {
		// TODO: Confirm that the data is being copied by reference
		
		struct sensor_combined_s sensor_combined = this->all_sensor_data.sensor_combined;
		orb_copy(ORB_ID(sensor_combined), this->sensors_sub[0], &sensor_combined);
		// TODO: do something with the data...

		// Auxiliary function defined in sensor_combined.h that prints the data provided by the topic
		print_message(ORB_ID(sensor_combined), sensor_combined);

		// PX4_INFO("Task received data at timestamp %lu", sensor_combined.timestamp);
	} 

	if (this->poll_fds[1].revents & POLLIN) {
		struct sensor_gps_s sensor_gps = this->all_sensor_data.sensor_gps;
		orb_copy(ORB_ID(sensor_gps), this->sensors_sub[1], &sensor_gps);

		print_message(ORB_ID(sensor_gps), sensor_gps);
	}
}