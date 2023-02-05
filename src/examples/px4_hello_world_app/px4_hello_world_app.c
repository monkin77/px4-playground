/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file px4_hello_world_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <poll.h>
#include <unistd.h>
#include <stdio.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_hello_world_app_main(int argc, char *argv[]);

int px4_hello_world_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Space!");

	// Testing Sensor subscriptions
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	
	// Limit the update rate to 2 Hz (500 ms interval)
	orb_set_interval(sensor_sub_fd, 500);

	// Advertise attitude topic
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	// One could wait for multiple topics with this technique, just using one here
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd, .events = POLLIN},
		/* There could be more File Descriptors here, in the form:
			{.fd: other_sub_fd, .events: POLLIN}
		*/
	};

	int error_counter = 0;

	int counter = 0;
	while (true) {
		/* wait for sensor update of 1 file descriptor for 1000ms (1s) */
		int poll_ret = px4_poll(fds, 1, 1000);

		// Counter to avoid infinite loop for now
		counter++;
		if (counter > 10) break;

		if (poll_ret == 0) {
			// This means none of our providers is giving us data
			PX4_ERR("Got no data within a second");
		} else if (poll_ret < 0) {
			// This is seriously bad - should be an emergency
			if (error_counter < 10 || error_counter % 50 == 0) {
				// Use a counter to prevent flooding (and slowing us down)
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;
		} else {
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

				// Log the sensor data
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f", 
					(double) raw.accelerometer_m_s2[0],
					(double) raw.accelerometer_m_s2[1],
					(double) raw.accelerometer_m_s2[2]
				);
				
				// Set att and publish this information for other apps
				// The following lines does not have any meaning, it's just an example
				att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);	
			}

			/* There could be more File Descriptors here in the form:
			if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	return OK;
}
