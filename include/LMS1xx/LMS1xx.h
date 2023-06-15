/*
 * LMS1xx.h
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

#ifndef LMS1XX_H_
#define LMS1XX_H_

#include "LMS1xx/lms_buffer.h"
#include "LMS1xx/lms_structs.h"

#include <string>
#include <stdint.h>

#include "rclcpp/logging.hpp"


typedef enum
{
  undefined = 0,
  initialisation = 1,
  configuration = 2,
  idle = 3,
  rotated = 4,
  in_preparation = 5,
  ready = 6,
  ready_for_measurement = 7
} status_t;

/*!
* @class LMS1xx
* @brief Class responsible for communicating with LMS1xx device.
*
* @author Konrad Banachowicz
*/

class LMS1xx
{
public:
  LMS1xx();
  virtual ~LMS1xx();

  /*!
  * @brief Connect to LMS1xx.
  * @param host LMS1xx host name or ip address.
  * @param port LMS1xx port number.
  */
  void connect(std::string host, int port = 2111);

  /*!
  * @brief Disconnect from LMS1xx device.
  */
  void disconnect();

  /*!
  * @brief Get status of connection.
  * @returns connected or not.
  */
  bool isConnected();

  /*!
  * @brief Start measurements.
  * After receiving this command LMS1xx unit starts spinning laser and measuring.
  */
  void startMeas();

  /*!
  * @brief Stop measurements.
  * After receiving this command LMS1xx unit stop spinning laser and measuring.
  */
  void stopMeas();

  /*!
  * @brief Get current status of LMS1xx device.
  * @returns status of LMS1xx device.
  */
  status_t queryStatus();

  /*!
  * @brief Log into LMS1xx unit.
  * Increase privilege level, giving ability to change device configuration.
  */
  void login();

  /*!
  * @brief Get current scan configuration.
  * Get scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @returns scanCfg structure.
  */
  scanCfg getScanCfg() const;

  /*!
  * @brief Set scan configuration.
  * Get scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @param cfg structure containing scan configuration.
  */
  void setScanCfg(const scanCfg &cfg);

  /*!
  * @brief Set scan data configuration.
  * Set format of scan message returned by device.
  * @param cfg structure containing scan data configuration.
  */
  void setScanDataCfg(const scanDataCfg &cfg);

  /*!
  * @brief Get current output range configuration.
  * Get output range configuration :
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @returns scanOutputRange structure.
  */
  scanOutputRange getScanOutputRange() const;

  /*!
  * @brief Start or stop continuous data acquisition.
  * After reception of this command device start or stop continuous data stream containing scan messages.
  * @param start 1 : start 0 : stop
  */
  void scanContinous(int start);

  /*!
  * @brief Receive single scan message.
  * @return true if scan was read successfully, false if error or timeout. False implies that higher level
  *         logic should take correct action such as reopening the connection.
  */
  bool getScanData(scanData* scan_data);

  /*!
  * @brief Save data permanently.
  * Parameters are saved in the EEPROM of the LMS and will also be available after the device is switched off and on again.
  *
  */
  void saveConfig();

  /*!
  * @brief The device is returned to the measurement mode after configuration.
  *
  */
  void startDevice();

protected:
  /*!
  * @brief Receive single scan message.
  * @param data pointer to scanData buffer structure.
  */
  static void parseScanData(char* buf, scanData* data);

  bool connected_;
  rclcpp::Logger logger_;
  LMSBuffer buffer_;
  int socket_fd_;
};

#endif /* LMS1XX_H_ */

