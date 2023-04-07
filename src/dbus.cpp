/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 2019/10/30.
//

#include "rm_dbus/dbus.h"

#include <fcntl.h> /* File control definitions */
#include <ros/ros.h>
#include <unistd.h>

#include <cstring>

#define termios asmtermios
#include <asm/termios.h>
#undef termios

#include <termios.h>

extern "C" {
extern int ioctl(int __fd, unsigned long int __request, ...) throw();
}

void DBus::init(const char* serial, double max_vel) {
  int fd = open(serial, O_RDWR | O_NOCTTY | O_SYNC);

  struct termios2 options {};
  ioctl(fd, TCGETS2, &options);

  if (fd == -1) {
    ROS_ERROR("[rt_dbus] Unable to open dbus\n");
  }

  // Even parity(8E1):
  options.c_cflag &= ~CBAUD;
  options.c_cflag |= BOTHER;

  options.c_cflag |= PARENB;
  options.c_cflag &= ~PARODD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  options.c_ispeed = 100000;
  options.c_ospeed = 100000;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~IGNBRK;  // disable break processing

  /* set input mode (non−canonical, no echo,...) */
  options.c_lflag = 0;
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;

  options.c_oflag = 0;                  // no remapping, no delays
  options.c_cflag |= (CLOCAL | CREAD);  // ignore modem controls, enable reading
  ioctl(fd, TCSETS2, &options);

  port_ = fd;

  MAX_VEL = max_vel;
}

rm_dbus::raw_dbus DBus::read() {
  uint8_t read_byte;
  int timeout = 0;  // time out of one package
  int count = 0;    // count of bit of one package
  while (timeout < 10 && count < 18) {
    // Read a byte //
    size_t n = ::read(port_, &read_byte, sizeof(read_byte));
    if (n == 0) {
      timeout++;
    } else if (n == 1) {
      buff_[count++] = read_byte;
    }
  }

  if (count < 18) {
    if (count > 0) {
      is_update_ = false;
      printf("Error on receiving\n");
    }
  } else {
    unpack();
    is_update_ = true;
  }

  return raw_data;
}

void DBus::unpack() {
  raw_data.ch0 = (buff_[0] | buff_[1] << 8) & 0x07FF;
  raw_data.ch1 = (buff_[1] >> 3 | buff_[2] << 5) & 0x07FF;
  raw_data.ch2 = (buff_[2] >> 6 | buff_[3] << 2 | buff_[4] << 10) & 0x07FF;
  raw_data.ch3 = (buff_[4] >> 1 | buff_[5] << 7) & 0x07FF;

  raw_data.s1 = ((buff_[5] >> 4) & 0x000C) >> 2;
  raw_data.s2 = (buff_[5] >> 4) & 0x0003;

  raw_data.x = buff_[6] | (buff_[7] << 8);
  raw_data.y = buff_[8] | (buff_[9] << 8);
  raw_data.z = buff_[10] | (buff_[11] << 8);
  raw_data.l = buff_[12];
  raw_data.r = buff_[13];
  raw_data.key = buff_[14] | buff_[15] << 8;  // key board code
  raw_data.wheel = (buff_[16] | buff_[17] << 8);
}

void DBus::clear_data_() {
  d_bus_data_.ch0 = 0;
  d_bus_data_.ch1 = 0;
  d_bus_data_.ch2 = 0;
  d_bus_data_.ch3 = 0;
  d_bus_data_.wheel = 0;
  d_bus_data_.x = 0;
  d_bus_data_.y = 0;
  d_bus_data_.z = 0;
  d_bus_data_.l = 0;
  d_bus_data_.r = 0;
  d_bus_data_.key = 0;
}

int DBus::preData() {
  if (raw_data.s1 == 3) {
    if (!is_update_) {
      clear_data_();
      return 1;
    }

    d_bus_data_.ch0 = raw_data.ch0 - 1024;
    d_bus_data_.ch1 = raw_data.ch1 - 1024;
    d_bus_data_.ch2 = raw_data.ch2 - 1024;
    d_bus_data_.ch3 = raw_data.ch3 - 1024;
    d_bus_data_.wheel = raw_data.wheel - 1024;

    if ((abs(d_bus_data_.ch0) > 660) || (abs(d_bus_data_.ch1) > 660) ||
        (abs(d_bus_data_.ch2) > 660) || (abs(d_bus_data_.ch3) > 660)) {
      clear_data_();
      return 1;
    }

    /* prevent remote control zero deviation */
    if (d_bus_data_.ch0 <= 10 && d_bus_data_.ch0 >= -10) d_bus_data_.ch0 = 0;
    if (d_bus_data_.ch1 <= 10 && d_bus_data_.ch1 >= -10) d_bus_data_.ch1 = 0;
    if (d_bus_data_.ch2 <= 10 && d_bus_data_.ch2 >= -10) d_bus_data_.ch2 = 0;
    if (d_bus_data_.ch3 <= 10 && d_bus_data_.ch3 >= -10) d_bus_data_.ch3 = 0;

    d_bus_data_.s1 = raw_data.s1;
    d_bus_data_.s2 = raw_data.s2;

    d_bus_data_.x = raw_data.x;
    d_bus_data_.y = raw_data.y;
    d_bus_data_.z = raw_data.z;
    d_bus_data_.l = raw_data.l;
    d_bus_data_.r = raw_data.r;
    d_bus_data_.key = raw_data.key;

    return 0;
  } else if (raw_data.s1 == 1) {
    clear_data_();
    return 1;
  } else
    return 2;
}

geometry_msgs::Twist DBus::getVel() const {
  geometry_msgs::Twist ret;

  ret.angular.z = d_bus_data_.ch0 / 660.0 * MAX_VEL;
  ret.linear.x = d_bus_data_.ch2 / 660.0 * MAX_VEL;
  ret.linear.y = d_bus_data_.ch3 / 660.0 * MAX_VEL;

  return ret;

  // d_bus_data->ch_r_x = static_cast<double>(d_bus_data_.ch0 / 660.0);
  // d_bus_data->ch_r_y = static_cast<double>(d_bus_data_.ch1 / 660.0);
  // d_bus_data->ch_l_x = static_cast<double>(d_bus_data_.ch2 / 660.0);
  // d_bus_data->ch_l_y = static_cast<double>(d_bus_data_.ch3 / 660.0);
  // d_bus_data->m_x = static_cast<double>(d_bus_data_.x / 1600.0);
  // d_bus_data->m_y = static_cast<double>(d_bus_data_.y / 1600.0);
  // d_bus_data->m_z = static_cast<double>(d_bus_data_.z / 1600.0);
  // d_bus_data->wheel = static_cast<double>(d_bus_data_.wheel / 660.0);

  // d_bus_data->s_l = d_bus_data_.s2;
  // d_bus_data->s_r = d_bus_data_.s1;
  // d_bus_data->p_l = d_bus_data_.l;
  // d_bus_data->p_r = d_bus_data_.r;

  // d_bus_data->key_w = d_bus_data_.key & 0x01 ? true : false;
  // d_bus_data->key_s = d_bus_data_.key & 0x02 ? true : false;
  // d_bus_data->key_a = d_bus_data_.key & 0x04 ? true : false;
  // d_bus_data->key_d = d_bus_data_.key & 0x08 ? true : false;
  // d_bus_data->key_shift = d_bus_data_.key & 0x10 ? true : false;
  // d_bus_data->key_ctrl = d_bus_data_.key & 0x20 ? true : false;
  // d_bus_data->key_q = d_bus_data_.key & 0x40 ? true : false;
  // d_bus_data->key_e = d_bus_data_.key & 0x80 ? true : false;
  // d_bus_data->key_r = (d_bus_data_.key >> 8) & 0x01 ? true : false;
  // d_bus_data->key_f = (d_bus_data_.key >> 8) & 0x02 ? true : false;
  // d_bus_data->key_g = (d_bus_data_.key >> 8) & 0x04 ? true : false;
  // d_bus_data->key_z = (d_bus_data_.key >> 8) & 0x08 ? true : false;
  // d_bus_data->key_x = (d_bus_data_.key >> 8) & 0x10 ? true : false;
  // d_bus_data->key_c = (d_bus_data_.key >> 8) & 0x20 ? true : false;
  // d_bus_data->key_v = (d_bus_data_.key >> 8) & 0x40 ? true : false;
  // d_bus_data->key_b = (d_bus_data_.key >> 8) & 0x80 ? true : false;

  // d_bus_data->stamp = ros::Time::now();
}
