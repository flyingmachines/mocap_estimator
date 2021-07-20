/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * Mocap Estimator for Position and Velocity
 *
 */
#include <px4_posix.h>
#include <px4_tasks.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <px4_config.h>
#include <math.h>
#include <float.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <poll.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>
#include <cmath>

#include <terrain_estimation/terrain_estimator.h>
#include "mocap_estimator_params.h"

#define MIN_VALID_W 0.00001f
#define PUB_INTERVAL 10000	// limit publish rate to 100 Hz
#define EST_BUF_SIZE 250000 / PUB_INTERVAL		// buffer size is 0.5s
#define MAX_WAIT_FOR_BARO_SAMPLE 3000000 // wait 3 secs for the baro to respond

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int mocap_estimator_task; /**< Handle of deamon task / thread */
static bool inav_verbose_mode = false;

static const hrt_abstime vision_topic_timeout = 500000;	// Vision topic timeout = 0.5s
static const hrt_abstime mocap_topic_timeout = 500000;		// Mocap topic timeout = 0.5s
static const hrt_abstime gps_topic_timeout = 500000;		// GPS topic timeout = 0.5s
static const hrt_abstime flow_topic_timeout = 1000000;	// optical flow topic timeout = 1s
static const hrt_abstime lidar_timeout = 150000;	// lidar timeout = 150ms
static const hrt_abstime lidar_valid_timeout = 1000000;	// estimate lidar distance during this time after lidar loss
static const unsigned updates_counter_len = 1000000;
static const float max_flow = 1.0f;	// max flow value that can be used, rad/s

extern "C" __EXPORT int mocap_estimator_main(int argc, char *argv[]);

int mocap_estimator_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static inline int min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

static inline int max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
	if (reason && *reason) {
		PX4_INFO("%s", reason);
	}

	PX4_INFO("usage: position_estimator_inav {start|stop|status} [-v]\n");
}

/**
* Weighted Average Filter for mocap data only.
*
*
*
 */
int mocap_estimator_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		inav_verbose_mode = false;

		if ((argc > 2) && (!strcmp(argv[2], "-v"))) {
			inav_verbose_mode = true;
		}

		thread_should_exit = false;
		mocap_estimator_task = px4_task_spawn_cmd("mocap_estimator",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 4600,
					       mocap_estimator_thread_main,
					       (argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) nullptr);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

/****************************************************************************
 * main
 ****************************************************************************/
int mocap_estimator_thread_main(int argc, char *argv[])
{
	orb_advert_t mavlink_log_pub = nullptr;
	int Nwin = 200;

	float x_est[2] = { 0.0f, 0.0f };	// pos, vel
	float y_est[2] = { 0.0f, 0.0f };	// pos, vel
	float z_est[2] = { 0.0f, 0.0f };	// pos, vel
	float xsum =0.0f, ysum=0.0f, zsum=0.0f;
	float xavg =0.0f, yavg=0.0f, zavg=0.0f;
	float xvk = 0.0f, xvk_1 = 0.0f, xvk_2 = 0.0f;
	float yvk = 0.0f, yvk_1 = 0.0f, yvk_2 = 0.0f;
	float zvk = 0.0f, zvk_1 = 0.0f, zvk_2 = 0.0f;
	float xk = 0.0f, xk_1 = 0.0f, xk_2 = 0.0f;
	float yk = 0.0f, yk_1 = 0.0f, yk_2 = 0.0f;
	float zk = 0.0f, zk_1 = 0.0f, zk_2 = 0.0f;


	float est_buf[EST_BUF_SIZE][3][2];	// estimated position buffer
	memset(est_buf, 0, sizeof(est_buf));
	int buf_ptr = 0;
	int cnt_ptr = 0;

	float x_est_prev[2], y_est_prev[2], z_est_prev[2];
	memset(x_est_prev, 0, sizeof(x_est_prev));
	memset(y_est_prev, 0, sizeof(y_est_prev));
	memset(z_est_prev, 0, sizeof(z_est_prev));

	//bool ref_inited = false;
	//hrt_abstime ref_init_start = 0;
	//const hrt_abstime ref_init_delay = 1000000;	// wait for 1s after 3D fix
	struct map_projection_reference_s ref;
	memset(&ref, 0, sizeof(ref));

	uint16_t attitude_updates = 0;

	hrt_abstime pub_last = hrt_absolute_time();

	hrt_abstime t_prev = 0;
	
	//const int mocap_heading = 2;

	bool mocap_xy_valid = false;		// mocap XY is valid
	bool mocap_z_valid = false;		// mocap Z is valid
	bool window_init = false;

	/* declare and safely initialize all structs */
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_local_position_s pos;
	memset(&pos, 0, sizeof(pos));
	struct vehicle_odometry_s mocap;
	memset(&mocap, 0, sizeof(mocap));

	/* subscribe */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int mocap_position_sub = orb_subscribe(ORB_ID(vehicle_mocap_odometry));
	int vision_position_sub = orb_subscribe(ORB_ID(vehicle_visual_odometry));

	/* advertise */
	orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &pos);
	//orb_advert_t vehicle_global_position_pub = nullptr;

	struct mocap_estimator_params params;
	memset(&params, 0, sizeof(params));
	struct mocap_estimator_param_handles mocest_param_handles;
	/* initialize parameter handles */
	mocap_parameters_init(&mocest_param_handles);

	/* first parameters read at start up */
	struct parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), parameter_update_sub,
		 &param_update); /* read from param topic to clear updated flag */
	/* first parameters update */
	mocap_parameters_update(&mocest_param_handles, &params);

	thread_running = true;
	//hrt_abstime baro_wait_for_sample_time = hrt_absolute_time();

	/* main loop */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = vehicle_attitude_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		int ret = px4_poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(&mavlink_log_pub, "[inav] poll error on init");
			continue;

		} else if (ret > 0) {
			/* act on attitude updates */

			/* vehicle attitude */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			attitude_updates++;

			bool updated;

			/* parameter update */
			orb_check(parameter_update_sub, &updated);

			if (updated) {
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
				mocap_parameters_update(&mocest_param_handles, &params);
			}

			
			//matrix::Dcmf R = matrix::Quatf(att.q);

			/* vehicle mocap position */
			orb_check(mocap_position_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_mocap_odometry), mocap_position_sub, &mocap);

				mocap_xy_valid = PX4_ISFINITE(mocap.x)
						 && (PX4_ISFINITE(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE]) ? sqrtf(fmaxf(
								 mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE],
								 mocap.pose_covariance[mocap.COVARIANCE_MATRIX_Y_VARIANCE])) <= 100.0f : true);
				mocap_z_valid = PX4_ISFINITE(mocap.z)
						&& (PX4_ISFINITE(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE]) ?
						    mocap.pose_covariance[mocap.COVARIANCE_MATRIX_Z_VARIANCE] <= 100.0f : true);



				if(!window_init && mocap_xy_valid && mocap_z_valid){
						xsum += mocap.x;
						ysum += mocap.y;
						zsum += mocap.z;
						cnt_ptr++;
					if(cnt_ptr == Nwin){
						window_init = true;
						xavg = xsum/Nwin;
						yavg = ysum/Nwin;
						zavg = zsum/Nwin;
					}
				}else{
						xavg = params.mc_a_x * mocap.x + (1.0f - params.mc_a_x) * xavg;
						x_est[0] = xavg;
						yavg = params.mc_a_y * mocap.y + (1.0f - params.mc_a_y) * yavg;
						y_est[0] = yavg;
						zavg = params.mc_a_z * mocap.z + (1.0f - params.mc_a_z) * zavg;
						z_est[0] = zavg;
				}

			}

			orb_check(vision_position_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_visual_odometry), vision_position_sub, &mocap);

				mocap_xy_valid = PX4_ISFINITE(mocap.x)
						 && (PX4_ISFINITE(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE]) ? sqrtf(fmaxf(
								 mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE],
								 mocap.pose_covariance[mocap.COVARIANCE_MATRIX_Y_VARIANCE])) <= 100.0f : true);
				mocap_z_valid = PX4_ISFINITE(mocap.z)
						&& (PX4_ISFINITE(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE]) ?
						    mocap.pose_covariance[mocap.COVARIANCE_MATRIX_Z_VARIANCE] <= 100.0f : true);


				if(!window_init && mocap_xy_valid && mocap_z_valid){
						xsum += mocap.x;
						ysum += mocap.y;
						zsum += mocap.z;
						cnt_ptr++;
					if(cnt_ptr == Nwin){
						window_init = true;
						xavg = xsum/Nwin;
						yavg = ysum/Nwin;
						zavg = zsum/Nwin;
					}
				}else{
						xavg = params.mc_a_x * mocap.x + (1.0f - params.mc_a_x) * xavg;
						x_est[0] = xavg;
						yavg = params.mc_a_y * mocap.y + (1.0f - params.mc_a_y) * yavg;
						y_est[0] = yavg;
						zavg = params.mc_a_z * mocap.z + (1.0f - params.mc_a_z) * zavg;
						z_est[0] = zavg;
				}

			}

		}

		//matrix::Dcm<float> R = matrix::Quatf(att.q);

		/* check for timeout on mocap topic */
		if ((mocap_xy_valid || mocap_z_valid) && (t > (mocap.timestamp + mocap_topic_timeout))) {
			mocap_xy_valid = false;
			mocap_z_valid = false;
			//warnx("MOCAP timeout");
			//mavlink_log_info(&mavlink_log_pub, "[inav] MOCAP timeout");
		}

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.02, dt), 0.0002);		// constrain dt from 0.2 to 20 ms
		t_prev = t;

		if (window_init) /* Getting velocity from position data through 2nd order system*/
		{
			float alpha = 2.0f/dt;
			xk = x_est[0];
			xvk = (1.0f/(alpha*alpha + 2.0f*params.mc_zt_vx*params.mc_wn_vx*alpha + params.mc_wn_vx*params.mc_wn_vx))*(params.mc_wn_vx*params.mc_wn_vx*alpha*xk - params.mc_wn_vx*params.mc_wn_vx*alpha*xk_2 - (2.0f*params.mc_wn_vx*params.mc_wn_vx - 2.0f*alpha*alpha)*xvk_1 - (alpha*alpha - 2.0f*params.mc_zt_vx*params.mc_wn_vx*alpha + params.mc_wn_vx*params.mc_wn_vx)*xvk_2);
			x_est[1] = xvk;
			xk_2 = xk_1;
			xk_1 = xk;
			xvk_2 = xvk_1;
			xvk_1 = xvk;

			yk = y_est[0];
			yvk = (1.0f/(alpha*alpha + 2.0f*params.mc_zt_vy*params.mc_wn_vy*alpha + params.mc_wn_vy*params.mc_wn_vy))*(params.mc_wn_vy*params.mc_wn_vy*alpha*yk - params.mc_wn_vy*params.mc_wn_vy*alpha*yk_2 - (2.0f*params.mc_wn_vy*params.mc_wn_vy - 2.0f*alpha*alpha)*yvk_1 - (alpha*alpha - 2.0f*params.mc_zt_vy*params.mc_wn_vy*alpha + params.mc_wn_vy*params.mc_wn_vy)*yvk_2);
			//yvk = (1.0f/(params.mc_tc_y*alpha*alpha + alpha + 1.0f))*(alpha*yk - alpha*yk_2 - (2.0f - 2.0f*params.mc_tc_y*alpha*alpha)*yvk_1 - (params.mc_tc_y*alpha*alpha - alpha + 1.0f)*yvk_2);
			y_est[1] = yvk;
			yk_2 = yk_1;
			yk_1 = yk;
			yvk_2 = yvk_1;
			yvk_1 = yvk;

			zk = z_est[0];
			zvk = (1.0f/(alpha*alpha + 2.0f*params.mc_zt_vz*params.mc_wn_vz*alpha + params.mc_wn_vz*params.mc_wn_vz))*(params.mc_wn_vz*params.mc_wn_vz*alpha*zk - params.mc_wn_vz*params.mc_wn_vz*alpha*zk_2 - (2.0f*params.mc_wn_vz*params.mc_wn_vz - 2.0f*alpha*alpha)*zvk_1 - (alpha*alpha - 2.0f*params.mc_zt_vz*params.mc_wn_vz*alpha + params.mc_wn_vz*params.mc_wn_vz)*zvk_2);
			//zvk = (1.0f/(params.mc_tc_z*alpha*alpha + alpha + 1.0f))*(alpha*zk - alpha*zk_2 - (2.0f - 2.0f*params.mc_tc_z*alpha*alpha)*zvk_1 - (params.mc_tc_z*alpha*alpha - alpha + 1.0f)*zvk_2);
			z_est[1] = zvk;
			zk_2 = zk_1;
			zk_1 = zk;
			zvk_2 = zvk_1;
			zvk_1 = zvk;

		}


		bool can_estimate_xy = mocap_xy_valid ;
		bool can_estimate_z = mocap_z_valid;

		//float w_mocap_p = params.w_mocap_p;

		if (inav_verbose_mode) {}

		if (t > pub_last + PUB_INTERVAL) {
			pub_last = t;

			/* push current estimate to buffer */
			est_buf[buf_ptr][0][0] = x_est[0];
			est_buf[buf_ptr][0][1] = x_est[1];
			est_buf[buf_ptr][1][0] = y_est[0];
			est_buf[buf_ptr][1][1] = y_est[1];
			est_buf[buf_ptr][2][0] = z_est[0];
			est_buf[buf_ptr][2][1] = z_est[1];

			/* push current rotation matrix to buffer */
			//memcpy(R_buf[buf_ptr], &R._data[0][0], sizeof(R._data));

			buf_ptr++;

			if (buf_ptr >= EST_BUF_SIZE) {
				buf_ptr = 0;
			}


			/* publish local position */
			pos.xy_valid = can_estimate_xy;
			pos.v_xy_valid = can_estimate_xy;
			pos.z_valid = can_estimate_z;
			pos.v_z_valid = can_estimate_z;
			//pos.xy_global = pos.xy_valid && use_gps_xy;
			//pos.z_global = pos.z_valid && use_gps_z;
			pos.x = x_est[0];
			pos.vx = x_est[1];
			pos.y = y_est[0];
			pos.vy = y_est[1];
			pos.z = z_est[0];
			pos.vz = z_est[1];
			pos.ax = NAN;
			pos.ay = NAN;
			pos.az = NAN;
			pos.yaw = matrix::Eulerf(matrix::Quatf(att.q)).psi();
			//pos.eph = eph;
			//pos.epv = epv;
			pos.evh = 0.0f;
			pos.evv = 0.0f;
			pos.vxy_max = INFINITY;
			pos.vz_max = INFINITY;
			pos.hagl_min = INFINITY;
			pos.hagl_max = INFINITY;

			// this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
			pos.z_deriv = z_est[1];

			pos.timestamp = t;

			orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &pos);
		}
	}

	warnx("stopped");
	mavlink_log_info(&mavlink_log_pub, "[mocapest] stopped");
	thread_running = false;
	return 0;
}


int mocap_parameters_init(struct mocap_estimator_param_handles *h)
{
	h->mc_a_x = param_find("MOCAP_ALPHA_X");
	h->mc_a_y = param_find("MOCAP_ALPHA_Y");
	h->mc_a_z = param_find("MOCAP_ALPHA_Z");
	h->mc_wn_vx = param_find("MOCAP_WN_VX");
	h->mc_wn_vy = param_find("MOCAP_WN_VY");
	h->mc_wn_vz = param_find("MOCAP_WN_VZ");
	h->mc_zt_vx = param_find("MOCAP_ZT_VX");
	h->mc_zt_vy = param_find("MOCAP_ZT_VY");
	h->mc_zt_vz = param_find("MOCAP_ZT_VZ");

	return 0;
}

int mocap_parameters_update(const struct mocap_estimator_param_handles *h,
			   struct mocap_estimator_params *p)
{
	param_get(h->mc_a_x, &(p->mc_a_x));
	param_get(h->mc_a_y, &(p->mc_a_y));
	param_get(h->mc_a_z, &(p->mc_a_z));
	param_get(h->mc_wn_vx, &(p->mc_wn_vx));
	param_get(h->mc_wn_vy, &(p->mc_wn_vy));
	param_get(h->mc_wn_vz, &(p->mc_wn_vz));
	param_get(h->mc_zt_vx, &(p->mc_zt_vx));
	param_get(h->mc_zt_vy, &(p->mc_zt_vy));
	param_get(h->mc_zt_vz, &(p->mc_zt_vz));
	return 0;
}
