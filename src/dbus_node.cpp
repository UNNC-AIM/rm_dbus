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
// Modified by mxdh on 2023/4/17
//

#include "rm_dbus/dbus_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm_dbus");
  DBusNode dbus_node;
  ros::Rate loop_rate(80);
  while (ros::ok()) {
    dbus_node.run();
    loop_rate.sleep();
  }
  return 0;
}

DBusNode::DBusNode() {
  raw_data_pub_ = nh_.advertise<rm_dbus::raw_dbus>("raw_data", 1);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
  nh_.param<double>("max_vel", MAX_VEL,3);

  dbus_.init(serial_port_.data(), MAX_VEL);
}

void DBusNode::run() {
  raw_data_pub_.publish(dbus_.read());
  if (dbus_.preData() < 2) cmd_vel_pub_.publish(dbus_.getVel());
}
