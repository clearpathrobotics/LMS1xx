/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "lms1xx/LMS1xx.h"
#include "console_bridge/console.h"

#include <time.h>

LMS1xx::LMS1xx() : connected_(false)
{
  //console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
}

LMS1xx::~LMS1xx()
{
}

void LMS1xx::connect(std::string host, int port)
{
  if (!connected_)
  {
    logDebug("Creating non-blocking socket.");
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_)
    {
      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port = htons(port);
      inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

      logDebug("Connecting socket to laser.");
      int ret = ::connect(socket_fd_, (struct sockaddr *) &stSockAddr, sizeof(stSockAddr));

      if (ret == 0)
      {
        connected_ = true;
        logDebug("Connected succeeded.");
      }
    }
  }
}

NTPstatus LMS1xx::getNTPstatus() const
{
  NTPstatus ntpStatus;
  char buf[100];
  uint32_t bytes;

  // get max offset
  sprintf(buf, "%c%s%c", 0x02, "sRN TSCTCmaxoffset", 0x03);
  write(socket_fd_, buf, strlen(buf));
  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len-1] = 0;
  logDebug("RX: %s", buf);
  sscanf((buf + 20), "%X", &bytes);
  ntpStatus.maxOffsetNTP = *((float*)&bytes);

  // get dealy time
  sprintf(buf, "%c%s%c", 0x02, "sRN TSCTCdelay", 0x03);
  write(socket_fd_, buf, strlen(buf));
  len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
  	logWarn("invalid packet recieved");
  buf[len-1] = 0;
  logDebug("RX: %s", buf);
  sscanf((buf + 16), "%X", &bytes);

  ntpStatus.timeDelay = *((float*)&bytes);  

  return ntpStatus;
}

void LMS1xx::setNTPsettings(const NTPcfg &cfg)
{
  char buf[100];
  // send NTP role
  sprintf(buf, "%c%s %d%c", 0x02,
        "sWN TSCRole", cfg.NTProle, 0x03);
  logDebug("TX: %s", buf);
  write(socket_fd_, buf, strlen(buf));
  int len = read(socket_fd_, buf, 100);
  buf[len - 1] = 0;
  // send time synchronization interface 
  sprintf(buf, "%c%s %d%c", 0x02,
        "sWN TSCTCInterface", cfg.timeSyncIfc, 0x03);
  logDebug("TX: %s", buf);
  write(socket_fd_, buf, strlen(buf));
  len = read(socket_fd_, buf, 100);
  buf[len - 1] = 0;
  // send time server IP address c0 a8 0 1
  sprintf(buf, "%c%s %X %X %X %X%c", 0x02,
       "sWN TSCTCSrvAddr", cfg.serverIP[0], cfg.serverIP[1], cfg.serverIP[2], cfg.serverIP[3], 0x03);
  logDebug("TX: %s", buf);
  write(socket_fd_, buf, strlen(buf));
  len = read(socket_fd_, buf, 100);
  buf[len - 1] = 0;
  // send time zone
  sprintf(buf, "%c%s %X%c", 0x02,
          "sWN TSCTCtimezone",cfg.timeZone+12, 0x03);
  logDebug("TX: %s", buf);
  write(socket_fd_, buf, strlen(buf));
  len = read(socket_fd_, buf, 100);
  buf[len - 1] = 0;
  // send update time
  sprintf(buf, "%c%s +%d%c", 0x02,
          "sWN TSCTCupdatetime", cfg.updateTime, 0x03);
  logDebug("TX: %s", buf);
  write(socket_fd_, buf, strlen(buf));
  len = read(socket_fd_, buf, 100);
  buf[len - 1] = 0;
}

void LMS1xx::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
  }
}

bool LMS1xx::isConnected()
{
  return connected_;
}

void LMS1xx::startMeas()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sMN LMCstartmeas", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}

void LMS1xx::stopMeas()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sMN LMCstopmeas", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}

status_t LMS1xx::queryStatus()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sRN STlms", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);

  int ret;
  sscanf((buf + 10), "%d", &ret);

  return (status_t) ret;
}

void LMS1xx::login()
{
  char buf[100];
  int result;
  sprintf(buf, "%c%s%c", 0x02, "sMN SetAccessMode 03 F4724744", 0x03);

  fd_set readset;
  struct timeval timeout;


  do   //loop until data is available to read
  {
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    write(socket_fd_, buf, strlen(buf));

    FD_ZERO(&readset);
    FD_SET(socket_fd_, &readset);
    result = select(socket_fd_ + 1, &readset, NULL, NULL, &timeout);

  }
  while (result <= 0);

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}

scanCfg LMS1xx::getScanCfg() const
{
  scanCfg cfg;
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sRN LMPscancfg", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);

  sscanf(buf + 1, "%*s %*s %X %*d %X %X %X", &cfg.scaningFrequency,
         &cfg.angleResolution, &cfg.startAngle, &cfg.stopAngle);
  return cfg;
}

void LMS1xx::setScanCfg(const scanCfg &cfg)
{
  char buf[100];
  sprintf(buf, "%c%s %X +1 %X %X %X%c", 0x02, "sMN mLMPsetscancfg",
          cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle,
          cfg.stopAngle, 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  buf[len - 1] = 0;
}

void LMS1xx::setScanDataCfg(const scanDataCfg &cfg)
{
  char buf[100];
  sprintf(buf, "%c%s %02X 00 %d %d 0 %02X 00 %d %d 0 %d +%d%c", 0x02,
          "sWN LMDscandatacfg", cfg.outputChannel, cfg.remission ? 1 : 0,
          cfg.resolution, cfg.encoder, cfg.position ? 1 : 0,
          cfg.deviceName ? 1 : 0, cfg.timestamp ? 1 : 0, cfg.outputInterval, 0x03);
  logDebug("TX: %s", buf);
  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  buf[len - 1] = 0;
}

scanOutputRange LMS1xx::getScanOutputRange() const
{
  scanOutputRange outputRange;
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sRN LMPoutputRange", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  sscanf(buf + 1, "%*s %*s %*d %X %X %X", &outputRange.angleResolution,
         &outputRange.startAngle, &outputRange.stopAngle);
  return outputRange;
}

void LMS1xx::scanContinous(int start)
{
  char buf[100];
  sprintf(buf, "%c%s %d%c", 0x02, "sEN LMDscandata", start, 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02)
    logError("invalid packet recieved");

  buf[len] = 0;
  logDebug("RX: %s", buf);
}

bool LMS1xx::getScanData(scanData* scan_data)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(socket_fd_, &rfds);

  // Block a total of up to 100ms waiting for more data from the laser.
  while (1)
  {
    // Would be great to depend on linux's behaviour of updating the timeval, but unfortunately
    // that's non-POSIX (doesn't work on OS X, for example).
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    logDebug("entering select()", tv.tv_usec);
    int retval = select(socket_fd_ + 1, &rfds, NULL, NULL, &tv);
    logDebug("returned %d from select()", retval);
    if (retval)
    {
      buffer_.readFrom(socket_fd_);

      // Will return pointer if a complete message exists in the buffer,
      // otherwise will return null.
      char* buffer_data = buffer_.getNextBuffer();
      
      if (buffer_data)
      {
        parseScanData(buffer_data, scan_data);
        buffer_.popLastBuffer();
        return true;
      }
    }
    else
    {
      // Select timed out or there was an fd error.
      return false;
    }
  }
}


void LMS1xx::parseScanData(char* buffer, scanData* data)
{

  char* tok = strtok_r(buffer, " ",&strtok_saveptr); //Type of command
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Command
  tok = strtok_r(NULL, " ",&strtok_saveptr); //VersionNumber
  tok = strtok_r(NULL, " ",&strtok_saveptr); //DeviceNumber
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Serial number
  tok = strtok_r(NULL, " ",&strtok_saveptr); //DeviceStatus
  tok = strtok_r(NULL, " ",&strtok_saveptr); //MessageCounter
  tok = strtok_r(NULL, " ",&strtok_saveptr); //ScanCounter
  tok = strtok_r(NULL, " ",&strtok_saveptr); //PowerUpDuration
  tok = strtok_r(NULL, " ",&strtok_saveptr); //TransmissionDuration
  uint32_t PowerUpDuration;
  sscanf(tok, "%X", &PowerUpDuration);
  logDebug("PowerUpDuration : %d\n", PowerUpDuration);
  data->msg_startup_usec = PowerUpDuration;

  tok = strtok_r(NULL, " ",&strtok_saveptr); //InputStatus
  tok = strtok_r(NULL, " ",&strtok_saveptr); //OutputStatus
  tok = strtok_r(NULL, " ",&strtok_saveptr); //ReservedByteA
  tok = strtok_r(NULL, " ",&strtok_saveptr); //ScanningFrequency
  tok = strtok_r(NULL, " ",&strtok_saveptr); //MeasurementFrequency
  tok = strtok_r(NULL, " ",&strtok_saveptr);
  tok = strtok_r(NULL, " ",&strtok_saveptr);
  tok = strtok_r(NULL, " ",&strtok_saveptr);
  tok = strtok_r(NULL, " ",&strtok_saveptr); //NumberEncoders
  int NumberEncoders;
  sscanf(tok, "%d", &NumberEncoders);
  for (int i = 0; i < NumberEncoders; i++)
  {
    tok = strtok_r(NULL, " ",&strtok_saveptr); //EncoderPosition
    tok = strtok_r(NULL, " ",&strtok_saveptr); //EncoderSpeed
  }

  tok = strtok_r(NULL, " ",&strtok_saveptr); //NumberChannels16Bit
  int NumberChannels16Bit;
  sscanf(tok, "%d", &NumberChannels16Bit);
  logDebug("NumberChannels16Bit : %d", NumberChannels16Bit);

  for (int i = 0; i < NumberChannels16Bit; i++)
  {
    int type = -1; // 0 DIST1 1 DIST2 2 RSSI1 3 RSSI2
    char content[6];
    tok = strtok_r(NULL, " ",&strtok_saveptr); //MeasuredDataContent
    sscanf(tok, "%s", content);

    if (!strcmp(content, "DIST1"))
    {
      type = 0;
    }
    else if (!strcmp(content, "DIST2"))
    {
      type = 1;
    }
    else if (!strcmp(content, "RSSI1"))
    {
      type = 2;
    }
    else if (!strcmp(content, "RSSI2"))
    {
      type = 3;
    }
    tok = strtok_r(NULL, " ",&strtok_saveptr); //ScalingFactor
    tok = strtok_r(NULL, " ",&strtok_saveptr); //ScalingOffset
    tok = strtok_r(NULL, " ",&strtok_saveptr); //Starting angle
    tok = strtok_r(NULL, " ",&strtok_saveptr); //Angular step width
    tok = strtok_r(NULL, " ",&strtok_saveptr); //NumberData
    int NumberData;
    sscanf(tok, "%X", &NumberData);
    logDebug("NumberData : %d", NumberData);

    if (type == 0)
    {
      data->dist_len1 = NumberData;
    }
    else if (type == 1)
    {
      data->dist_len2 = NumberData;
    }
    else if (type == 2)
    {
      data->rssi_len1 = NumberData;
    }
    else if (type == 3)
    {
      data->rssi_len2 = NumberData;
    }

    for (int i = 0; i < NumberData; i++)
    {
      int dat;
      tok = strtok_r(NULL, " ",&strtok_saveptr); //data
      sscanf(tok, "%X", &dat);

      if (type == 0)
      {
        data->dist1[i] = dat;
      }
      else if (type == 1)
      {
        data->dist2[i] = dat;
      }
      else if (type == 2)
      {
        data->rssi1[i] = dat;
      }
      else if (type == 3)
      {
        data->rssi2[i] = dat;
      }

    }
  }

  tok = strtok_r(NULL, " ",&strtok_saveptr); //NumberChannels8Bit
  int NumberChannels8Bit;
  sscanf(tok, "%d", &NumberChannels8Bit);
  logDebug("NumberChannels8Bit : %d\n", NumberChannels8Bit);

  for (int i = 0; i < NumberChannels8Bit; i++)
  {
    int type = -1;
    char content[6];
    tok = strtok_r(NULL, " ",&strtok_saveptr); //MeasuredDataContent
    sscanf(tok, "%s", content);
    if (!strcmp(content, "DIST1"))
    {
      type = 0;
    }
    else if (!strcmp(content, "DIST2"))
    {
      type = 1;
    }
    else if (!strcmp(content, "RSSI1"))
    {
      type = 2;
    }
    else if (!strcmp(content, "RSSI2"))
    {
      type = 3;
    }
    tok = strtok_r(NULL, " ",&strtok_saveptr); //ScalingFactor
    tok = strtok_r(NULL, " ",&strtok_saveptr); //ScalingOffset
    tok = strtok_r(NULL, " ",&strtok_saveptr); //Starting angle
    tok = strtok_r(NULL, " ",&strtok_saveptr); //Angular step width
    tok = strtok_r(NULL, " ",&strtok_saveptr); //NumberData
    int NumberData;
    sscanf(tok, "%X", &NumberData);
    logDebug("NumberData : %d\n", NumberData);

    if (type == 0)
    {
      data->dist_len1 = NumberData;
    }
    else if (type == 1)
    {
      data->dist_len2 = NumberData;
    }
    else if (type == 2)
    {
      data->rssi_len1 = NumberData;
    }
    else if (type == 3)
    {
      data->rssi_len2 = NumberData;
    }
    for (int i = 0; i < NumberData; i++)
    {
      int dat;
      tok = strtok_r(NULL, " ",&strtok_saveptr); //data
      sscanf(tok, "%X", &dat);

      if (type == 0)
      {
        data->dist1[i] = dat;
      }
      else if (type == 1)
      {
        data->dist2[i] = dat;
      }
      else if (type == 2)
      {
        data->rssi1[i] = dat;
      }
      else if (type == 3)
      {
        data->rssi2[i] = dat;
      }
    }
  }

  tok = strtok_r(NULL, " ",&strtok_saveptr); //PositionDataEnable
  int PositionDataEnable;
  sscanf(tok, "%d", &PositionDataEnable);
  logDebug("NumberChannels8Bit : %d\n", PositionDataEnable);
  if(PositionDataEnable == 1){
  tok = strtok_r(NULL, " ",&strtok_saveptr); //X position
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Y position
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Z position
  tok = strtok_r(NULL, " ",&strtok_saveptr); //X rotation
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Y rotation
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Z rotation
  tok = strtok_r(NULL, " ",&strtok_saveptr); //rotations type
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Transmits the name of device
  }

  tok = strtok_r(NULL, " ",&strtok_saveptr); //NameEnable
  int NameEnable;
  sscanf(tok, "%d", &NameEnable);
  logDebug("NameEnable : %d\n", NameEnable);
  if(NameEnable == 1){
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Length of name
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Device name in characters
  }

  tok = strtok_r(NULL, " ",&strtok_saveptr); //CommentEnable
  int CommentEnable;
  sscanf(tok, "%d", &CommentEnable);
  logDebug("CommentEnable : %d\n", CommentEnable);
  if(CommentEnable == 1){
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Length of comment
  tok = strtok_r(NULL, " ",&strtok_saveptr); //Transmits a comment in characters
  }

  tok = strtok_r(NULL, " ",&strtok_saveptr); //TimeEnable
  int TimeEnable;
  sscanf(tok, "%d", &TimeEnable);
  logDebug("TimeEnable : %d\n", TimeEnable);
  if(TimeEnable == 1){
  struct tm msg_time = {0};
  //Year
  uint16_t year; 
  tok = strtok_r(NULL, " ",&strtok_saveptr); 
  sscanf(tok, "%hX", &year);
  msg_time.tm_year = (int) year - 1900;
  logDebug("NTP year: %hX\n", year);
  logDebug("NTP year: %d\n", msg_time.tm_year);
  // Month
  uint8_t int_8_buf; 
  tok = strtok_r(NULL, " ",&strtok_saveptr); 
  sscanf(tok, "%hhX", &int_8_buf);
  msg_time.tm_mon = (int) int_8_buf - 1;
  logDebug("NTP month: %hhX\n", int_8_buf);

  // Day
  tok = strtok_r(NULL, " ",&strtok_saveptr); 
  sscanf(tok, "%hhX", &int_8_buf);
  msg_time.tm_mday = (int) int_8_buf;
  logDebug("NTP day: %hhX\n", int_8_buf);

  // Hour
  tok = strtok_r(NULL, " ",&strtok_saveptr); 
  sscanf(tok, "%hhX", &int_8_buf);
  msg_time.tm_hour = (int) int_8_buf;
  logDebug("NTP hour: %hhX\n", int_8_buf);

  // Minute
  tok = strtok_r(NULL, " ",&strtok_saveptr); 
  sscanf(tok, "%hhX", &int_8_buf);
  msg_time.tm_min = (int) int_8_buf;
  logDebug("NTP minute: %hhX\n", int_8_buf);

  // Second
  tok = strtok_r(NULL, " ",&strtok_saveptr); 
  sscanf(tok, "%hhX", &int_8_buf);
  msg_time.tm_sec = (int) int_8_buf;
  data->msg_sec = mktime(&msg_time);
  logDebug("NTP sec: %lld\n", (long long) data->msg_sec);

  // Microsecond
  uint32_t microsecond; 
  tok = strtok_r(NULL, " ",&strtok_saveptr); 
  sscanf(tok, "%X", &microsecond);
  data->msg_usec = microsecond ;
  logDebug("NTP usec: %X\n", microsecond);  
  }
  

}

void LMS1xx::saveConfig()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sMN mEEwriteall", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}

void LMS1xx::startDevice()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sMN Run", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}
