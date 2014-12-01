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
*  File: cwrap.cpp
*  Desc: Provides functions that C objects can link against.
*  Auth: Ryan Gariepy
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

#include <string.h>

#include "husky_base/horizon_legacy/cwrap.h"

#include "husky_base/horizon_legacy/Message.h"
#include "husky_base/horizon_legacy/Message_data.h"
#include "husky_base/horizon_legacy/Message_request.h"
#include "husky_base/horizon_legacy/Message_cmd.h"
#include "husky_base/horizon_legacy/crc.h"

using namespace clearpath;

#ifdef __cplusplus
extern "C" {
#endif

/*** Request Message Formatting ******************************************************************/

int cwrapSubscribePkg(uint8_t *buf, size_t buf_size, uint16_t type,
    uint16_t freq)
{
  return Request(type, freq).toBytes(buf, buf_size);
}

int cwrapVelocityPkg(uint8_t *buf, size_t buf_size, double trans,
    double rot, double accel)
{
  return SetVelocity(trans, rot, accel).toBytes(buf, buf_size);
}

int cwrapChassisNamePkg(uint8_t *buf, size_t buf_size, char *name)
{
  return SetPlatformName(name).toBytes(buf, buf_size);
}

int cwrapChassisTimePkg(uint8_t *buf, size_t buf_size, uint32_t chassis_time)
{
  return SetPlatformTime(chassis_time).toBytes(buf, buf_size);
}

int cwrapSafetySystem(uint8_t *buf, size_t buf_size, uint16_t safety_flags)
{
  return SetSafetySystem(safety_flags).toBytes(buf, buf_size);
}

int cwrapDifferentialSpeedsPkg(uint8_t *buf, size_t buf_size,
    double l_speed, double r_speed, double l_accel, double r_accel)
{
  return SetDifferentialSpeed(l_speed, r_speed, l_accel, r_accel).toBytes(buf, buf_size);
}

int cwrapDifferentialCtrlsPkg(uint8_t *buf, size_t buf_size,
    double p, double i, double d, double ffwd, double stic, double i_lim)
{
  return SetDifferentialControl(p, i, d, ffwd, stic, i_lim).toBytes(buf, buf_size);
}

int cwrapDifferentialOutputsPkg(uint8_t *buf, size_t buf_size, double l_setpt, double r_setpt)
{
  return SetDifferentialOutput(l_setpt, r_setpt).toBytes(buf, buf_size);
}

int cwrapAckermannOutputPkg(uint8_t *buf, size_t buf_size, double steer, double throt, double brake)
{
  return SetAckermannOutput(steer, throt, brake).toBytes(buf, buf_size);
}

int cwrapTurnPkg(uint8_t *buf, size_t buf_size, double trans, double radius, double accel)
{
  return SetTurn(trans, radius, accel).toBytes(buf, buf_size);
}

int cwrapMaxSpeedPkg(uint8_t *buf, size_t buf_size, double max_fwd, double max_rev)
{
  return SetMaxSpeed(max_fwd, max_rev).toBytes(buf, buf_size);
}

int cwrapMaxAccelPkg(uint8_t *buf, size_t buf_size, double max_fwd, double max_rev)
{
  return SetMaxAccel(max_fwd, max_rev).toBytes(buf, buf_size);
}

int cwrapGearPkg(uint8_t *buf, size_t buf_size, int8_t gear)
{
  return SetGear(gear).toBytes(buf, buf_size);
}

int cwrapProcessorReset(uint8_t *buf, size_t buf_size)
{
  return CmdProcessorReset().toBytes(buf, buf_size);
}


/*** Data Message Parsing ************************************************************************/

/* Data message parsing */
int cwrapGetDifferentialControl(uint8_t *packet, size_t len,
    double *l_p, double *l_i, double *l_d, double *l_ffwd, double *l_stic, double *l_int_lim,
    double *r_p, double *r_i, double *r_d, double *r_ffwd, double *r_stic, double *r_int_lim)
{
  Message *msg = Message::factory(packet, len);
  DataDifferentialControl *ctl = dynamic_cast<DataDifferentialControl *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (ctl == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  *l_p = ctl->getLeftP();
  *l_i = ctl->getLeftI();
  *l_d = ctl->getLeftD();
  *l_ffwd = ctl->getLeftFeedForward();
  *l_stic = ctl->getLeftStiction();
  *l_int_lim = ctl->getLeftIntegralLimit();

  *r_p = ctl->getRightP();
  *r_i = ctl->getRightI();
  *r_d = ctl->getRightD();
  *r_ffwd = ctl->getRightFeedForward();
  *r_stic = ctl->getRightStiction();
  *r_int_lim = ctl->getRightIntegralLimit();

  delete msg;
  return 0;
}

int cwrapGetDifferentialOutput(uint8_t *packet, size_t len, double *left, double *right)
{
  Message *msg = Message::factory(packet, len);
  DataDifferentialOutput *out = dynamic_cast<DataDifferentialOutput *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (out == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  *left = out->getLeft();
  *right = out->getRight();

  delete msg;
  return 0;
}

int cwrapGetDifferentialSpeed(uint8_t *packet, size_t len,
    double *l_speed, double *l_accel, double *r_speed, double *r_accel)
{
  Message *msg = Message::factory(packet, len);
  DataDifferentialSpeed *spd = dynamic_cast<DataDifferentialSpeed *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (spd == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  *l_speed = spd->getLeftSpeed();
  *l_accel = spd->getLeftAccel();
  *r_speed = spd->getRightSpeed();
  *r_accel = spd->getRightAccel();

  delete msg;
  return 0;
}

int cwrapGetEncoders(uint8_t *packet, size_t len, int8_t enc_count, double *travel, double *speed)
{
  Message *msg = Message::factory(packet, len);
  DataEncoders *enc = dynamic_cast<DataEncoders *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (enc == NULL)
  {
    delete msg;
    return -1;
  }

  /* Copy requested data */
  for (int i = 0; i < enc_count; ++i)
  {
    if (i < enc->getCount())
    {
      travel[i] = enc->getTravel(i);
      speed[i] = enc->getSpeed(i);
    }
    else
    {
      travel[i] = 0.0;
      speed[i] = 0.0;
    }
  }

  delete msg;
  return 0;
}

int cwrapGetEncodersRaw(uint8_t *packet, size_t len, int8_t enc_count, int32_t ticks[])
{
  Message *msg = Message::factory(packet, len);
  DataEncodersRaw *enc = dynamic_cast<DataEncodersRaw *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (enc == NULL)
  {
    delete msg;
    return -1;
  }

  /* Copy requested data */
  for (int i = 0; i < enc_count; ++i)
  {
    if (i < enc->getCount())
    {
      ticks[i] = enc->getTicks(i);
    }
    else
    {
      ticks[i] = 0;
    }
  }

  delete msg;
  return 0;
}

int cwrapGetFirmwareInfo(uint8_t *packet, size_t len,
    uint8_t *firm_maj, uint8_t *firm_min, uint8_t *proto_maj, uint8_t *proto_min, uint32_t *wr_time)
{
  Message *msg = Message::factory(packet, len);
  DataFirmwareInfo *fwi = dynamic_cast<DataFirmwareInfo *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (fwi == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  *firm_maj = fwi->getMajorFirmwareVersion();
  *firm_min = fwi->getMinorFirmwareVersion();
  *proto_maj = fwi->getMajorProtocolVersion();
  *proto_min = fwi->getMinorProtocolVersion();
  *wr_time = fwi->getWriteTime().rawTime;

  delete msg;
  return 0;
}

int cwrapGetMaxAcceleration(uint8_t *packet, size_t len, double *fwd_max, double *rev_max)
{
  Message *msg = Message::factory(packet, len);
  DataMaxAcceleration *max = dynamic_cast<DataMaxAcceleration *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (max == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  *fwd_max = max->getForwardMax();
  *rev_max = max->getReverseMax();

  delete msg;
  return 0;
}

int cwrapGetMaxSpeed(uint8_t *packet, size_t len, double *fwd_max, double *rev_max)
{
  Message *msg = Message::factory(packet, len);
  DataMaxSpeed *max = dynamic_cast<DataMaxSpeed *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (max == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  *fwd_max = max->getForwardMax();
  *rev_max = max->getReverseMax();

  delete msg;
  return 0;
}

int cwrapGetPlatformInfo(uint8_t *packet, size_t len,
    char *model, uint8_t name_len, int8_t *revision, int32_t *serial)
{
  Message *msg = Message::factory(packet, len);
  DataPlatformInfo *info = dynamic_cast<DataPlatformInfo *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (info == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  strncpy(model, info->getModel().c_str(), name_len);
  model[name_len - 1] = '\0';
  *revision = info->getRevision();
  *serial = info->getSerial();

  delete msg;
  return 0;
}

int cwrapGetPlatformName(uint8_t *packet, size_t len, char *name, uint8_t name_len)
{
  Message *msg = Message::factory(packet, len);
  DataPlatformName *pla = dynamic_cast<DataPlatformName *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (pla == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  strncpy(name, pla->getName().c_str(), name_len);
  name[name_len - 1] = '\0';

  delete msg;
  return 0;
}

int cwrapGetPlatformRotation(uint8_t *packet, size_t len,
    double *roll_rate, double *pitch_rate, double *yaw_rate)
{
  Message *msg = Message::factory(packet, len);
  DataPlatformRotation *rot = dynamic_cast<DataPlatformRotation *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (rot == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  *roll_rate = rot->getRollRate();
  *pitch_rate = rot->getPitchRate();
  *yaw_rate = rot->getYawRate();

  delete msg;
  return 0;
}

int cwrapGetPowerSystem(uint8_t *packet, size_t len,
    uint8_t bat_count, double charge_est[], int16_t cap_est[], uint8_t desc[])
{
  Message *msg = Message::factory(packet, len);
  DataPowerSystem *pow = dynamic_cast<DataPowerSystem *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (pow == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  for (int i = 0; i < bat_count; ++i)
  {
    if (i < pow->getBatteryCount())
    {
      charge_est[i] = pow->getChargeEstimate(i);
      cap_est[i] = pow->getCapacityEstimate(i);
      desc[i] = pow->getDescription(i).rawDesc;
    }
    else
    {
      charge_est[i] = 0.0;
      cap_est[i] = 0;
      desc[i] = 0;
    }
  }

  delete msg;
  return 0;
}

int cwrapGetSafetySystemStatus(uint8_t *packet, size_t len, uint8_t *flags)
{
  Message *msg = Message::factory(packet, len);
  DataSafetySystemStatus *sss = dynamic_cast<DataSafetySystemStatus *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (sss == NULL)
  {
    delete msg;
    return -1;
  }

  /* Write out data */
  *flags = sss->getFlags();

  delete msg;
  return 0;
}

int cwrapGetSystemStatus(uint8_t *packet, size_t len, uint32_t *uptime,
    int8_t voltage_count, double voltage[],
    int8_t current_count, double current[],
    int8_t temp_count, double temp[])
{
  Message *msg = Message::factory(packet, len);
  DataSystemStatus *sys = dynamic_cast<DataSystemStatus *>(msg);

  /* If packet is of wrong type, dynamic cast will fail and return null */
  if (sys == NULL)
  {
    delete msg;
    return -1;
  }

  /*Write out data */
  *uptime = sys->getUptime();
  for (int i = 0; i < voltage_count; ++i)
  {
    if (i < sys->getVoltagesCount())
    {
      voltage[i] = sys->getVoltage(i);
    }
    else
    {
      voltage[i] = 0.0;
    }
  }
  for (int i = 0; i < current_count; ++i)
  {
    if (i < sys->getCurrentsCount())
    {
      current[i] = sys->getCurrent(i);
    }
    else
    {
      current[i] = 0.0;
    }
  }
  for (int i = 0; i < temp_count; ++i)
  {
    if (i < sys->getTemperaturesCount())
    {
      temp[i] = sys->getTemperature(i);
    }
    else
    {
      temp[i] = 0.0;
    }
  }

  delete msg;
  return 0;
}

/*** Miscellaneous *******************************************************************************/
uint16_t cwrapCrc16(int32_t size, int32_t init_val, uint8_t *data)
{
  return crc16(size, init_val, data);
}

void inputSingleByte(uint8_t *new_bytes, uint16_t recv_len, uint8_t *recv_buffer,
    uint8_t *cur_write_buf, uint8_t *cur_read_buf,
    uint8_t *char_idx, uint8_t *num_char_left,
    uint8_t SOH, uint8_t RECV_LEN,
    uint8_t NUM_BUF)
{

  uint16_t i;
  uint8_t byte_in;
  for (i = 0; i < recv_len; i++)
  {
    byte_in = new_bytes[i];
    // We're at the beginning and have found the SOH. Begin writing to our new buffer
    if (*char_idx == 0 && byte_in == SOH)
    {
      recv_buffer[(*cur_write_buf) * RECV_LEN + (*char_idx)++] = SOH;
      // We're looking for SOH and haven't found it. Continue.
    }
    else if (*char_idx == 0)
    {
      return;
      // First length byte (real length)
    }
    else if (*char_idx == 1)
    {
      recv_buffer[(*cur_write_buf) * RECV_LEN + (*char_idx)++] = byte_in;
      // Verify length
    }
    else if (*char_idx == 2)
    {
      uint8_t len_1 = recv_buffer[(*cur_write_buf) * RECV_LEN + (*char_idx) - 1];
      // Bad length, but we may have started reading a byte early
      // Treat this byte as the first length byte, and read next
      // into idx #2
      if ((len_1 & 0xFF) != (~byte_in & 0xFF) && len_1 == SOH)
      {
        recv_buffer[(*cur_write_buf) * RECV_LEN + 1] = byte_in;
        *char_idx = 2;
        //TRACE_DEBUG("Byte early? %X %X\n\r", len_1, byte_in);
      }
        // Bad length, but we may have started reading two bytes early
        // Treat this byte as the SOH, and read next into idx #1
      else if ((len_1 & 0xFF) != (~byte_in & 0xFF) && byte_in == SOH)
      {
        *char_idx = 1;
        //TRACE_DEBUG("Two bytes early? %X %X\n\r", len_1, byte_in);
      }
        // Bad length. Assume invalid start
      else if ((len_1 & 0xFF) != (~byte_in & 0xFF))
      {
        *char_idx = 0;
        //TRACE_DEBUG("Invalid start? %X %X\n\r", len_1, byte_in);
      }
        // Good length
      else
      {
        *num_char_left = len_1;
        // Avoid buffer overruns
        if (*num_char_left > RECV_LEN - 3)
        {
          *num_char_left = RECV_LEN - 3;
        }
        recv_buffer[(*cur_write_buf) * RECV_LEN + (*char_idx)] = ~(*num_char_left);
        (*char_idx)++;
        //TRACE_DEBUG("Reading %d chars\n\r",num_char_left);
      }
      // Read data
    }
    else
    {
      recv_buffer[(*cur_write_buf) * RECV_LEN + (*char_idx)++] = byte_in;
      (*num_char_left)--;
      // We're done writing. Prepare for next incoming message
      if (*num_char_left == 0)
      {
        //TRACE_DEBUG("New buffer ready: %d chars\n\r",char_idx);
        *char_idx = 0;
        (*cur_write_buf)++;
        // Wrap buffer
        if (*cur_write_buf == NUM_BUF)
        {
          *cur_write_buf = 0;
        }
        // Discard old data to make space for new, if necessary
        if (*cur_write_buf == *cur_read_buf)
        {
          (*cur_read_buf)++;
          if (*cur_read_buf == NUM_BUF)
          {
            *cur_read_buf = 0;
          }
        }
      }
    }
  }
}

#ifdef __cplusplus
}  //extern "C"
#endif
