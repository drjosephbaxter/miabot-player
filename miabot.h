/*  miabot.h
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

#ifndef _MIABOTDEVICE_H
#define _MIABOTDEVICE_H


#include <libplayercore/playercore.h>

#include <string>
#include "miabot_params.h"

using namespace std;


#define CAM_IMAGE_WIDTH	176 
#define CAM_IMAGE_HEIGHT 144
#define MAX_BLOBS 8  

typedef struct
{ 
                       
  int redmin;                   
  int greenmin;
  int bluemin;
                     
  int redmax;                   
  int greenmax;
  int bluemax;
} color_config;

typedef struct player_miabot_data{

  player_position2d_data_t position2d;
  player_sonar_data_t sonar; 
  player_sonar_data_t photodiode; 
  player_gripper_data_t gripper;
  player_gripper_data_t lift;
  player_position2d_data_t compass;
  player_blobfinder_data_t blobfinder;

}__attribute__((packed)) player_miabot_data_t;
     
class Miabot : public Driver
{

 public:
   
  Miabot(ConfigFile* cf, int section);
  ~Miabot (void);

  virtual int Setup();
  virtual int Shutdown();
  virtual int Subscribe(player_devaddr_t id);
  virtual int Unsubscribe(player_devaddr_t id); 
  virtual void Main(); 

  virtual int ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, 
                               void * data);
  
  
 private:

  player_miabot_data_t miabot_data;
 
  player_devaddr_t tracker_id;
  player_devaddr_t position2d_id;
  player_devaddr_t sonar_id;  
  player_devaddr_t gripper_id;
  player_devaddr_t lift_id;
  player_devaddr_t compass_id;
  player_devaddr_t photodiode_id;
  player_devaddr_t blobfinder_id;

  int position2d_subscriptions;
  int compass_subscriptions;
  int sonar_subscriptions;
  int gripper_subscriptions;
  int lift_subscriptions;
  int photodiode_subscriptions;
  int blobfinder_subscriptions;
  int set_cam;
  bool s_cmap;
  string bluetooth_mac;
  string bluetooth_router;
  string bluetooth_port;
  int bluetooth_type;
  color_config color[MAX_BLOBS];
  int num_blobs;
  int ping;
  int prate, erate, swait; //ping rate, enc rate, socket wait
  int white, adj, filt; // camera settings
  
  bool tracker; // tacker plugin  

  int open_arg, close_arg; // gripper settings

  int up_arg, down_arg; // lift settings
  
  Device* position;
  // current odom pose
  double position_x, position_y, position_a;

  bool buffer; // buffer last sonar echo received
  
  //methods
  
  void update_everything(player_miabot_data_t* d);
  
  int HandleCommand(player_msghdr * hdr, void * data);
  int HandleConfig(QueuePointer & resp_queue, player_msghdr * hdr, 
                               void * data);
  
  void reset_odometry();
  void ToggleMotorPower(bool val);
  int SetupPosition();
  int ShutdownPosition();
  void ProcessPositionData(player_position2d_data_t* data);
  int HandleGripperCommand(player_msghdr * hdr, void * data);
  int HandleLiftCommand(player_msghdr * hdr, void * data);

};

#endif
