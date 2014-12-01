/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™ 
 *
 *  File: cwrap.h
 *  Desc: Provides functions that C objects can link against.
 *  Auth: Iain Peet
 *
 *  Copyright (c) 2010, Clearpath Robotics, Inc. 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com 
 *
 */

#ifndef CWRAP_H_
#define CWRAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>

uint16_t cwrapCrc16(int32_t size, int32_t init_val, uint8_t * data);
void inputSingleByte(uint8_t* new_bytes, uint16_t recv_len, uint8_t* recv_buffer,
										  uint8_t* cur_write_buf, uint8_t* cur_read_buf,
										  uint8_t* char_idx, uint8_t* num_char_left,
										  uint8_t SOH, uint8_t RECV_LEN,
										  uint8_t NUM_BUF);


/* Request message formatting */
int cwrapSubscribePkg(uint8_t* buf, size_t buf_size, uint16_t type, uint16_t freq);
int cwrapVelocityPkg(uint8_t* buf, size_t buf_size, double trans, double rot, double accel);
int cwrapChassisNamePkg(uint8_t* buf, size_t buf_size, char* name);
int cwrapChassisTimePkg(uint8_t* buf, size_t buf_size, uint32_t chassis_time);
int cwrapSafetySystem(uint8_t* buf, size_t buf_size, uint16_t safety_flags);
int cwrapDifferentialSpeedsPkg(uint8_t* buf, size_t buf_size,
	double l_speed, double r_speed, double l_accel, double r_accel);
int cwrapDifferentialCtrlsPkg(uint8_t* buf, size_t buf_size,
	double p, double i, double d, double ffwd, double stic, double i_lim);
int cwrapDifferentialOutputsPkg(uint8_t* buf, size_t buf_size, double l_setpt, double r_setpt);
int cwrapAckermannOutputPkg(uint8_t* buf, size_t buf_size, double steer, double throt, double brake);
int cwrapTurnPkg(uint8_t* buf, size_t buf_size, double trans, double radius, double accel);
int cwrapMaxSpeedPkg(uint8_t* buf, size_t buf_size, double max_fwd, double max_rev);
int cwrapMaxAccelPkg(uint8_t* buf, size_t buf_size, double max_fwd, double max_rev);
int cwrapGearPkg(uint8_t* buf, size_t buf_size, int8_t gear);
int cwrapProcessorResetPkg(uint8_t* buf, size_t buf_size);

/* Data message parsing */
int cwrapGetDifferentialControl(uint8_t* packet, size_t len, 
		double *l_p, double *l_i, double *l_d, double *l_ffwd, double *l_stic, double *l_int_lim,
		double *r_p, double *r_i, double *r_d, double *r_ffwd, double *r_stic, double *r_int_lim);
int cwrapGetDifferentialOutput(uint8_t* packet, size_t len, double *left, double *right);
int cwrapGetDifferentialSpeed(uint8_t* packet, size_t len,
		double *l_speed, double *l_accel, double *r_speed, double *r_accel );
int cwrapGetEncoders(uint8_t* packet, size_t len, int8_t enc_count, double travel[], double speed[]);
int cwrapGetEncodersRaw(uint8_t* packet, size_t len, int8_t enc_count, int32_t ticks[]);
int cwrapGetFirmwareInfo(uint8_t* packet, size_t len, 
		uint8_t* firm_maj, uint8_t* firm_min, uint8_t* proto_maj, uint8_t* proto_min, uint32_t *wr_time);
int cwrapGetMaxAcceleration(uint8_t* packet, size_t len, double *fwd_max, double *rev_max);
int cwrapGetMaxSpeed(uint8_t* packet, size_t len, double *fwd_max, double *rev_max);
int cwrapGetPlatformInfo(uint8_t* packet, size_t len, 
		char *model, uint8_t name_len, int8_t *revision, int32_t *serial);
int cwrapGetPlatformName(uint8_t* packet, size_t len, char *name, uint8_t name_len);
int cwrapGetPlatformRotation(uint8_t* packet, size_t len,
		double *roll_rate, double *pitch_rate, double *yaw_rate);
int cwrapGetPowerSystem(uint8_t* packet, size_t len, 
		uint8_t bat_count, double charge_est[], int16_t cap_est[], uint8_t desc[]);
int cwrapGetSafetySystemStatus(uint8_t* packet, size_t len, uint8_t *flags);
int cwrapGetSystemStatus(uint8_t* packet, size_t len, uint32_t *uptime,
		int8_t voltage_count, double voltage[],
		int8_t current_count, double current[],
		int8_t temp_count, double temp[]);


#ifdef __cplusplus
}	//extern "C"
#endif

#endif // CWRAP_H_

