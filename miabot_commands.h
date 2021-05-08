/*  miabot_commands.h
 *  Miabot Plugin for Player
 *  Copyright (C) - Joseph Baxter {joseph.lee.baxter@gmail.com}
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _MIABOT_COMMANDS_H
#define _MIABOT_COMMANDS_H

#include <string>

#include <libplayercore/playercore.h>
#include "miabot.h"
#include <bluetooth/bluetooth.h>



using namespace std;

void bluetooth_dongle(string bluetooth_mac);
void bluetooth(string bluetooth_router, string bluetooth_port);
int rfcomm_connect_dongle(bdaddr_t *src, bdaddr_t *dst, uint8_t channel);
int rfcomm_connect(string bluetooth_router, string bluetooth_port);
string hexbyte2str(int n);
string hexbyte2strB(int n);
int str2hexdig(char s);
int str2hexbyte(string s);
double ticks2m(int ticks);
int m2speed(double m);
int m2ticks(double m);
int deg2ticks(double d);
void stop();
void encoder();
void defaults();
int bin2int();
void seq(int f, int t, int r, bool b, bool l, bool forw);
int value2index(int value);
void stopped();
double time();
void p(int ping);
void rec();
void DoScan(player_miabot_data_t* d, int photodiode, int sonar, bool buffer);
void ToggleSonar(bool b, int ping);
void parseSonar();
void odom(string inp_odom);
void set_odometry(float m_x, float m_y, float rad_theta);
void ToggleOdom(bool b, int enc);
void updateOdom(player_miabot_data_t* d);
void HandlePositionCommand(player_position2d_cmd_vel_t position2d_cmd);
void HandlePositionCommand_pos(player_position2d_cmd_pos_t position2d_cmd);
void CloseGripper(int close_arg);
void OpenGripper(int open_arg);
void UpGripper(int up_arg);
void DownGripper(int down_arg);
void compass(player_miabot_data_t* d);
void updateGripper(player_miabot_data_t* d);
int rgb(int rmin, int rmax,int gmin, int gmax,int bmin, int bmax);
void blob(player_miabot_data_t* d);
void camTrack(int set_cam,int num_blobs, int redmin,int redmax, int redmin1,int redmax1, int redmin2,int redmax2,int redmin3,int redmax3, int redmin4,int redmax4, int redmin5,int redmax5,int redmin6,int redmax6, int redmin7,int redmax7, int greenmin,int greenmax, int greenmin1,int greenmax1,int greenmin2,int greenmax2, int greenmin3,int greenmax3, int greenmin4,int greenmax4,int greenmin5,int greenmax5, int greenmin6,int greenmax6, int greenmin7,int greenmax7, int bluemin,int bluemax,int bluemin1,int bluemax1, int bluemin2,int bluemax2, int bluemin3,int bluemax3,int bluemin4,int bluemax4, int bluemin5,int bluemax5, int bluemin6,int bluemax6,int bluemin7,int bluemax7);
void camStopTracking();
void parseBlob(string inp_blob);
void Track();
void camera_settings(int white,int adj,int filt);
void readData(int wait);

void voltage(string bluetooth_router, string bluetooth_port);
void TogglePill(bool set, int rate);
#endif
