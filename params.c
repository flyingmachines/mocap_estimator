/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

/*
 * @file position_estimator_inav_params.c
 *
 * @author Anton Babushkin <rk3dov@gmail.com>
 *
 * Parameters for position_estimator_inav
 */

#include "mocap_estimator_params.h"

/**
 * Alpha for Mocap Z
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_ALPHA_X, 0.5f);

/**
 * Alpha for Mocap X
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_ALPHA_Y, 0.5f);

/**
 * Alpha for Mocap Y
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_ALPHA_Z, 0.5f);
/**
 * cutoff frequency for Filter for Vx
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_WN_VX, 1.0f);
/**
 * cut-off frequency for Filter for Vy
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_WN_VY, 1.0f);
/**
 * cut-off frequency for Filter for Vz
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_WN_VZ, 1.0f);
/**
 * Damping for Vx
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_ZT_VX, 1.0f);
/**
 * Damping for Vy
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_ZT_VY, 1.0f);
/**
 * Damping for Vz
 * @min 0.0
 * @max 1.0
 * @group Mocap Estimator
 */
PARAM_DEFINE_FLOAT(MOCAP_ZT_VZ, 1.0f);
