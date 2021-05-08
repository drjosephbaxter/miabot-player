/*  miabot.cc
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

/**
 * @brief Miabot Pro  

 This driver offer access to the various Miabot-mediated devices, logically
splitting up the devices' functionality.

@par Compile-time dependencies

- tracker.so if you are intending to use the global tracking system installed in the robot village.

- lbluetooth, enables bluetooth communications over rfcomm

@par Requires

If the overhead tracking system is in use the following required device interfaces are implmented:

- "tracker" @ref interface_position2d
  - This interface returns global tracking data (if equipped)

@par Provides

The Miabot driver provides the following device interfaces, some of
them named:

- "odometry" @ref interface_position2d
  - This interface returns odometry data, and accepts velocity commands.

- "compass" @ref interface_position2d
  - This interface returns compass data (if equipped).

- @ref interface_sonar
  - Returns data from sonar arrays (if equipped)

- "photodiode" @ref interface_sonar
  - This interface returns photodiode data (if equipped).

- @ref interface_gripper
  - Controls gripper (if equipped)

- "lift" @ref interface_gripper
  - Controls lift motion of the gripper (if equipped)

- @ref interface_blobfinder
  - Controls a AVRcam (if equipped).

@par Supported configuration requests

- "odometry" @ref interface_position2d :
  - PLAYER_POSITION_SET_ODOM_REQ
  - PLAYER_POSITION_MOTOR_POWER_REQ
  - PLAYER_POSITION_RESET_ODOM_REQ
  - PLAYER_POSITION_GET_GEOM_REQ
- @ref interface_sonar :
  - PLAYER_SONAR_GET_GEOM_REQ
- "photodiode" @ref interface_sonar :
  - PLAYER_SONAR_GET_GEOM_REQ
- @ref interface_blobfinder :
  - PLAYER_BLOBFINDER_SET_COLOR_REQ
  - PLAYER_BLOBFINDER_SET_IMAGER_PARAMS_REQ
- @ref interface_gripper :
  - PLAYER_GRIPPER_GET_GEOM_REQ
- "lift" @ref interface_gripper :
  - PLAYER_GRIPPER_GET_GEOM_REQ

@par Configuration file options

- tracking system settings :
  - tracker - plugin enabled (1) or plugin disabled (0)

- connection settings :
  - type - tcp/ip (0) or rfcomm (1)
  - mac "xx:xx:xx:xx:xx:xx" - The mac address of the miabot your are trying to connect to (standard plugin only)
  - router "0.0.0.0" - The ip address of your bluetooth router (tcp/ip plugin only)
  - port "0" - The port assigned to the robot you wish to connect to (tcp/ip plugin only)

- sensor/actuator settings :
  - ping - bitmask of sonar/photodiode arrays to ping
  - ping_rate - sonar/photodiode ping rate (ms)
  - enc_rate - motor encoder tick rate (ms)
  - bt_wait - bluetooth sockt max wait time (microseconds)
  - open_arg - how far the gripper opens - (0) maximum
  - close_arg - how far the gripper closes - (185) demo object width
  - down_arg - how far the lifter drops	 - (85) horizontal
  - up_arg - how far the lifter lifts - (150) maximum
  - send_cam - switch to send colour map or not at boot (1/0) note: dev version only
  - auto_white - avrcam auto-white balance (1/0)
  - auto_adj - avrcam auto-gain (1/0)
  - light_filt - avrcam fluorescent light filter (1/0)
  - num_blobs - number of different colour blobs for the avrcam to track (0-8)
  - redmin# - minimum red range for rgb value of blob to track (# - id of blob (0-7))
  - redmax# - maximum red range for rgb value of blob to track (# - id of blob (0-7))
  - greenmin# - minimum green range for rgb value of blob to track (# - id of blob (0-7))
  - greenmax# - minimum green range for rgb value of blob to track (# - id of blob (0-7))
  - bluemin# - minimum blue range for rgb value of blob to track (# - id of blob (0-7))
  - bluemax# - minimum blue range for rgb value of blob to track (# - id of blob (0-7)) 

@par Example

@verbatim
driver
(
  name "miabot"
  provides ["position:0" "sonar:0" "blobfinder:0"]
)
@endverbatim

@author Joseph Baxter

 */
/*
#if HAVE_CONFIG_H
  #include <config.h>
#endif
*/

#include <string>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>

#include <libplayercore/playercore.h>
#include <libplayerxdr/playerxdr.h>

#include "miabot.h"
#include "miabot_commands.h"
#include "miabot_params.h"

using namespace std;

#define DEFAULT_MAC "00:00:00:00:00"
#define DEFAULT_TYPE 0
#define DEFAULT_ROUTER "village1" 
#define DEFAULT_PORT "5000"
//  config
Driver* Miabot_Init(ConfigFile* cf, int section)
{
  return((Driver*)(new Miabot(cf, section)));
}

//  add driver
void Miabot_Register(DriverTable* table)
{
  table->AddDriver("miabot", Miabot_Init);
}

// process messages
int Miabot::ProcessMessage(QueuePointer & resp_queue,player_msghdr * hdr,void * data)
{
  // Check for capabilities requests first
  HANDLE_CAPABILITY_REQUEST (position2d_id, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILTIES_REQ);
  HANDLE_CAPABILITY_REQUEST (gripper_id, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILTIES_REQ);
   // Position2d caps
  HANDLE_CAPABILITY_REQUEST (position2d_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL);
  HANDLE_CAPABILITY_REQUEST (position2d_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_POS);
// Gripper caps
  HANDLE_CAPABILITY_REQUEST (gripper_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_OPEN);
  HANDLE_CAPABILITY_REQUEST (gripper_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_CLOSE);
// lift caps
  HANDLE_CAPABILITY_REQUEST (lift_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_OPEN);
  HANDLE_CAPABILITY_REQUEST (lift_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_CLOSE);
  
  // Look for configuration requests
  // Is it new odometry data?
 
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
			   PLAYER_POSITION2D_DATA_STATE,
			   this->tracker_id))
    {
      this->ProcessPositionData((player_position2d_data_t*)data);
      return(0);
    } 
    
  else if(hdr->type == PLAYER_MSGTYPE_REQ)
    return(this->HandleConfig(resp_queue,hdr,data));
  else if(hdr->type == PLAYER_MSGTYPE_CMD)
    return(this->HandleCommand(hdr,data));
  
  else
    return(-1);
}
   
//  construtor
Miabot::Miabot(ConfigFile* cf, int section) : Driver(cf, section,true,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
  memset(&this->tracker_id, 0, sizeof(player_devaddr_t));
  memset(&this->position2d_id, 0, sizeof(player_devaddr_t));
  memset(&this->sonar_id, 0, sizeof(player_devaddr_t));
  memset(&this->gripper_id, 0, sizeof(player_devaddr_t));
  memset(&this->lift_id, 0, sizeof(player_devaddr_t));
  memset(&this->compass_id, 0, sizeof(player_devaddr_t));
  memset(&this->photodiode_id, 0, sizeof(player_devaddr_t));
  memset(&this->blobfinder_id, 0, sizeof(player_devaddr_t));
  
  this->position2d_subscriptions = 0;
  this->compass_subscriptions = 0;
  this->sonar_subscriptions = 0;
  this->gripper_subscriptions = 0;
  this->lift_subscriptions = 0;
  this->photodiode_subscriptions = 0;
  this->blobfinder_subscriptions = 0;


  this->tracker = cf->ReadInt(section, "tracker", 0);

  if(this->tracker){ 
    // Must have a position device to get localisation
    if (cf->ReadDeviceAddr(&this->tracker_id, section, "requires",
			   PLAYER_POSITION2D_CODE, -1, "tracker") != 0)
      {
	this->SetError(-1);
	return;
      }
  }
  //  position2d
  if(cf->ReadDeviceAddr(&(this->position2d_id), section, "provides",
                      PLAYER_POSITION2D_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->position2d_id) != 0)
    {
      this->SetError(-1);    
      return;
    }
    // Stop position messages in the queue from being overwritten
    //    this->InQueue->AddReplaceRule (this->position2d_id, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_POS, false);
  }else
    {
      PLAYER_WARN("Position2d interface not created for miabot driver");
    }
  // compass
  if(cf->ReadDeviceAddr(&(this->compass_id), section, "provides", 
                      PLAYER_POSITION2D_CODE, -1, "compass") == 0)
    {
      if(this->AddInterface(this->compass_id)!= 0)
	{
	  this->SetError(-1);
	  return;
	}
    }else
      {
	PLAYER_WARN("compass interface not created for miabot driver");
      }
  
  //  sonar
  
  if(cf->ReadDeviceAddr(&(this->sonar_id), section, "provides", 
			PLAYER_SONAR_CODE, -1, NULL) == 0)
    {
      if(this->AddInterface(this->sonar_id) != 0)
	{
	  this->SetError(-1);    
	  return;
	}
      
      
    }else
      {
	PLAYER_WARN("sonar interface not created for miabot driver");
      }
  
  // photodiode
  if(cf->ReadDeviceAddr(&(this->photodiode_id), section, "provides", 
			PLAYER_SONAR_CODE, -1, "photodiode") == 0)
    {
      if(this->AddInterface(this->photodiode_id) != 0)
	{
	  this->SetError(-1);    
	  return;
	}
      
      
    }else
      {
	PLAYER_WARN("photodiode interface not created for miabot driver");
      }
  // lift 
  if(cf->ReadDeviceAddr(&(this->lift_id), section, "provides", 
                      PLAYER_GRIPPER_CODE, -1, "lift") == 0)
  {
    if(this->AddInterface(this->lift_id) != 0)
      {
	this->SetError(-1);
	return;
      }
  }else
    {
      PLAYER_WARN("gripper interface not created for miabot driver");
    }
  // gripper 
  if(cf->ReadDeviceAddr(&(this->gripper_id), section, "provides", 
                      PLAYER_GRIPPER_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->gripper_id) != 0)
      {
	this->SetError(-1);
	return;
      }
  }else
    {
      PLAYER_WARN("gripper interface not created for miabot driver");
    }
  
  // blobfinder
  if(cf->ReadDeviceAddr(&(this->blobfinder_id), section, "provides", 
                      PLAYER_BLOBFINDER_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->blobfinder_id) != 0)
    {
      this->SetError(-1);
      return;
    }

  
  }else
  {
      PLAYER_WARN("blobfinder interface not created for miabot driver");
  }

  /**
   * @par communications method
- router - tcp/ip (0) 
- dongle -  rfcomm (1)
  */
  this->bluetooth_type = cf->ReadInt(section, "type", DEFAULT_TYPE);
  if(this->bluetooth_type==0){
    this->bluetooth_router = cf->ReadString(section, "router", DEFAULT_ROUTER);
    this->bluetooth_port = cf->ReadString(section, "port", DEFAULT_PORT);
  }else{
    this->bluetooth_mac = cf->ReadString(section, "mac", DEFAULT_MAC);
  }
  initialize_miabot_params();

  if(this->bluetooth_type==0){
    bluetooth(bluetooth_router,bluetooth_port);
  }else{
    bluetooth_dongle(bluetooth_mac);
  }
  //check volts
  voltage(this->bluetooth_router,this->bluetooth_port);
 
  //set drive defaults (hi speed ramps / anti squeel)
  defaults(); 

  //set ping number from bitmap
  this->ping = cf->ReadInt(section, "ping", 0);
  p(ping);

  this->prate = cf->ReadInt(section, "ping_rate", 0);
 
  printf("Set sonar ping rate to %d ms.\n",this->prate);

  //this->erate = cf->ReadInt(section, "enc_rate", 0);

  //printf("Set enc rate to %d ms.\n",this->erate);

  this->swait = cf->ReadInt(section, "bt_wait", 0);

  printf("Set socket wait time %d micoseconds.\n",this->swait);

  this->num_blobs = cf->ReadInt(section, "num_blobs", 0);

  this->white = cf->ReadInt(section, "auto_white", 0);
  
  this->adj = cf->ReadInt(section, "auto_adj", 0);
  
  this->filt = cf->ReadInt(section, "light_filt", 0);

  this->set_cam = cf->ReadInt(section, "set_cam", 0);

  if(this->set_cam==1){

    if(this->white==1){
      printf("Auto White-Balance Enabled.\n");
    }else{
      printf("Auto White-Balance Disabled.\n");
    }
    if(this->filt==1){
      printf("Auto Light Filter Enabled.\n");
    }else{
      printf("Auto Light Filter Disabled.\n");
    }
    if(this->adj==1){
      printf("Auto Adjust Enabled.\n");
    }else{
      printf("Auto Adjust Disabled.\n");
    }
    camera_settings(this->white,this->adj,this->filt);
  }
   
  this->color[0].redmin = cf->ReadInt(section, "redmin0", 0);
  this->color[0].redmax = cf->ReadInt(section, "redmax0", 0);
  this->color[0].greenmin = cf->ReadInt(section, "greenmin0", 0);
  this->color[0].greenmax = cf->ReadInt(section, "greenmax0", 0);
  this->color[0].bluemin = cf->ReadInt(section, "bluemin0", 0);
  this->color[0].bluemax = cf->ReadInt(section, "bluemax0", 0);
  this->color[1].redmin = cf->ReadInt(section, "redmin1", 0);
  this->color[1].redmax = cf->ReadInt(section, "redmax1", 0);
  this->color[1].greenmin = cf->ReadInt(section, "greenmin1", 0);
  this->color[1].greenmax = cf->ReadInt(section, "greenmax1", 0);
  this->color[1].bluemin = cf->ReadInt(section, "bluemin1", 0);
  this->color[1].bluemax = cf->ReadInt(section, "bluemax1", 0);
  this->color[2].redmin = cf->ReadInt(section, "redmin2", 0);
  this->color[2].redmax = cf->ReadInt(section, "redmax2", 0);
  this->color[2].greenmin = cf->ReadInt(section, "greenmin2", 0);
  this->color[2].greenmax = cf->ReadInt(section, "greenmax2", 0);
  this->color[2].bluemin = cf->ReadInt(section, "bluemin2", 0);
  this->color[2].bluemax = cf->ReadInt(section, "bluemax2", 0);
  this->color[3].redmin = cf->ReadInt(section, "redmin3", 0);
  this->color[3].redmax = cf->ReadInt(section, "redmax3", 0);
  this->color[3].greenmin = cf->ReadInt(section, "greenmin3", 0);
  this->color[3].greenmax = cf->ReadInt(section, "greenmax3", 0);
  this->color[3].bluemin = cf->ReadInt(section, "bluemin3", 0);
  this->color[3].bluemax = cf->ReadInt(section, "bluemax3", 0);
  this->color[4].redmin = cf->ReadInt(section, "redmin4", 0);
  this->color[4].redmax = cf->ReadInt(section, "redmax4", 0);
  this->color[4].greenmin = cf->ReadInt(section, "greenmin4", 0);
  this->color[4].greenmax = cf->ReadInt(section, "greenmax4", 0);
  this->color[4].bluemin = cf->ReadInt(section, "bluemin4", 0);
  this->color[4].bluemax = cf->ReadInt(section, "bluemax4", 0);
  this->color[5].redmin = cf->ReadInt(section, "redmin5", 0);
  this->color[5].redmax = cf->ReadInt(section, "redmax5", 0);
  this->color[5].greenmin = cf->ReadInt(section, "greenmin5", 0);
  this->color[5].greenmax = cf->ReadInt(section, "greenmax5", 0);
  this->color[5].bluemin = cf->ReadInt(section, "bluemin5", 0);
  this->color[5].bluemax = cf->ReadInt(section, "bluemax5", 0);
  this->color[6].redmin = cf->ReadInt(section, "redmin6", 0);
  this->color[6].redmax = cf->ReadInt(section, "redmax6", 0);
  this->color[6].greenmin = cf->ReadInt(section, "greenmin6", 0);
  this->color[6].greenmax = cf->ReadInt(section, "greenmax6", 0);
  this->color[6].bluemin = cf->ReadInt(section, "bluemin6", 0);
  this->color[6].bluemax = cf->ReadInt(section, "bluemax6", 0);
  this->color[7].redmin = cf->ReadInt(section, "redmin7", 0);
  this->color[7].redmax = cf->ReadInt(section, "redmax7", 0);
  this->color[7].greenmin = cf->ReadInt(section, "greenmin7", 0);
  this->color[7].greenmax = cf->ReadInt(section, "greenmax7", 0);
  this->color[7].bluemin = cf->ReadInt(section, "bluemin7", 0);
  this->color[7].bluemax = cf->ReadInt(section, "bluemax7", 0);
  
  this->s_cmap = true;
  
  camTrack(this->set_cam,this->num_blobs,this->color[0].redmin,this->color[0].redmax,this->color[1].redmin,this->color[1].redmax,this->color[2].redmin,this->color[2].redmax,this->color[3].redmin,this->color[3].redmax,this->color[4].redmin,this->color[4].redmax,this->color[5].redmin,this->color[5].redmax,this->color[6].redmin,this->color[6].redmax,this->color[7].redmin,this->color[7].redmax,this->color[0].greenmin,this->color[0].greenmax,this->color[1].greenmin,this->color[1].greenmax,this->color[2].greenmin,this->color[2].greenmax,this->color[3].greenmin,this->color[3].greenmax,this->color[4].greenmin,this->color[4].greenmax,this->color[5].greenmin,this->color[5].greenmax,this->color[6].greenmin,this->color[6].greenmax,this->color[7].greenmin,this->color[7].greenmax,this->color[0].bluemin,this->color[0].bluemax,this->color[1].bluemin,this->color[1].bluemax,this->color[2].bluemin,this->color[2].bluemax,this->color[3].bluemin,this->color[3].bluemax,this->color[4].bluemin,this->color[4].bluemax,this->color[5].bluemin,this->color[5].bluemax,this->color[6].bluemin,this->color[6].bluemax,this->color[7].bluemin,this->color[7].bluemax);  
  
  this->s_cmap = false;


  this->open_arg = cf->ReadInt(section, "open_arg", 0);
  
  this->close_arg = cf->ReadInt(section, "close_arg", 255);

  this->up_arg = cf->ReadInt(section, "up_arg", 150);
  
  this->down_arg = cf->ReadInt(section, "down_arg", 85);

  this->buffer = cf->ReadInt(section, "buffer_sonar", 0);
}
//  setup 
int Miabot::Setup(){

  /* now spawn reading thread */

  if(this->tracker){
    SetupPosition(); 
  }
  this->StartThread();
  
  return (0);
}
  
//  shutdown
int Miabot::Shutdown(){
  //make sure robot has stopped everything
  if(this->blobfinder_subscriptions){
    camStopTracking();
  }
  //ToggleOdom(false,erate);
  if(this->sonar_subscriptions){
    ToggleSonar(false,prate);
  }
  this->ToggleMotorPower(false);
  if(this->tracker){
 	 ShutdownPosition();
 }
  this->StopThread();
  return (0);
}

int Miabot::Subscribe(player_devaddr_t id){
  int setupResult;

  // do the subscription
  if((setupResult = Driver::Subscribe(id)) == 0)
    {
      // also increment the appropriate subscription counter

      if(Device::MatchDeviceAddress(id, this->position2d_id))
	this->position2d_subscriptions++;
     
      else if(Device::MatchDeviceAddress(id, this->compass_id))
	this->compass_subscriptions++;
      else if(Device::MatchDeviceAddress(id, this->sonar_id))
	this->sonar_subscriptions++;
      else if(Device::MatchDeviceAddress(id, this->gripper_id))
      	this->gripper_subscriptions++;
      else if(Device::MatchDeviceAddress(id, this->lift_id))
      	this->lift_subscriptions++;
      else if(Device::MatchDeviceAddress(id, this->photodiode_id))
	this->photodiode_subscriptions++;
      else if(Device::MatchDeviceAddress(id, this->blobfinder_id))
	this->blobfinder_subscriptions++;
    }


  return(setupResult);
  
}

int Miabot::Unsubscribe(player_devaddr_t id){
  int shutdownResult;
  
  // do the unsubscription
  if((shutdownResult = Driver::Unsubscribe(id)) == 0)
    {
      // also decrement the appropriate subscription counter
      if(Device::MatchDeviceAddress(id, this->position2d_id)){
	this->position2d_subscriptions--;
	assert(this->position2d_subscriptions >= 0);
   
      }else if(Device::MatchDeviceAddress(id, this->compass_id)){
	this->compass_subscriptions--;
	assert(this->compass_subscriptions >= 0);
      }else if(Device::MatchDeviceAddress(id, this->sonar_id)){
	this->sonar_subscriptions--;
	assert(this->sonar_subscriptions >= 0);
      }else if(Device::MatchDeviceAddress(id, this->gripper_id)){
	this->gripper_subscriptions--;
	assert(this->gripper_subscriptions >= 0);
      }else if(Device::MatchDeviceAddress(id, this->lift_id)){
	this->lift_subscriptions--;
	assert(this->lift_subscriptions >= 0);
      }else if(Device::MatchDeviceAddress(id, this->photodiode_id)){
	this->photodiode_subscriptions--;
	assert(this->photodiode_subscriptions >= 0);
      }else if(Device::MatchDeviceAddress(id, this->blobfinder_id)){
	this->blobfinder_subscriptions--;
	assert(this->blobfinder_subscriptions >= 0);
      }
    }
  
  return(shutdownResult);
}


// main
void Miabot::Main(){
 
  int last_position2d_subscrcount = 0;
  int last_blobfinder_subscrcount = 0;
  int last_sonar_subscrcount=0;
  int last_gripper_subscrcount=0;
  
  while(1){
    pthread_testcancel();
this->Lock();
    //position2d
    //////////////////////////////////////////////////////////////////////////////
    if(!last_position2d_subscrcount && this->position2d_subscriptions){
      printf("STARTING MOTORS...\n");
      
      //make sure robot doesn't go anywhere
      this->ToggleMotorPower(false);  
     
    }else if(last_position2d_subscrcount && !(this->position2d_subscriptions)){
      //make sure robot doesn't go anywhere
      printf("STOPPING MOTORS...\n");
      
      this->ToggleMotorPower(false);
      
    }
    /*if(this->position2d_subscriptions){
      // check if stalled
      stopped();
      }*/

    last_position2d_subscrcount = this->position2d_subscriptions;
    
    //sonar
    //////////////////////////////////////////////////////////////////////////////
    if(!last_sonar_subscrcount && this->sonar_subscriptions){
      ToggleSonar(true,prate);
    }else if(last_sonar_subscrcount && !(this->sonar_subscriptions)){
      ToggleSonar(false,prate);
    }
    last_sonar_subscrcount = this->sonar_subscriptions;
    
    //gripper
    /////////////////////////////////////////////////////////////////////////////
    if(!last_gripper_subscrcount && this->gripper_subscriptions){
      TogglePill(true,prate);
    }else if(last_gripper_subscrcount && !(this->gripper_subscriptions)){
      TogglePill(false,prate);
    }
    last_gripper_subscrcount = this->gripper_subscriptions;
    
    //blobfinder
    ////////////////////////////////////////////////////////////////////////////
    if(!last_blobfinder_subscrcount && this->blobfinder_subscriptions){
     
      //start tracking
      // have to issuse track() twice --- WHY??????
    
	Track();
	Track();
            
    }else if(last_blobfinder_subscrcount && !(this->blobfinder_subscriptions)){
      //turn tracking off
        
      camStopTracking();
    }
    
    last_blobfinder_subscrcount = this->blobfinder_subscriptions;
           

    // handle pending messages
    if(!this->InQueue->Empty()){
      ProcessMessages();
    }
       
this->Unlock();
    // Get data from robot 
    
   
    memset(&(this->miabot_data),0,sizeof(player_miabot_data_t));
    update_everything(&miabot_data);
  


    pthread_testcancel();
    
    this->Publish(this->position2d_id,
		  PLAYER_MSGTYPE_DATA,
		  PLAYER_POSITION2D_DATA_STATE,
		  (void*)&(this->miabot_data.position2d),
		  sizeof(player_position2d_data_t),
		  NULL);
    this->Publish(this->sonar_id,
		  PLAYER_MSGTYPE_DATA,
		  PLAYER_SONAR_DATA_RANGES,
		  (void*)&(this->miabot_data.sonar),
		  sizeof(player_sonar_data_t),
		  NULL);
    this->Publish(this->compass_id,
		  PLAYER_MSGTYPE_DATA,
		  PLAYER_POSITION2D_DATA_STATE,
		  (void*)&(this->miabot_data.compass), 
		  sizeof(player_position2d_data_t),
		  NULL);
    this->Publish(this->gripper_id,
		  PLAYER_MSGTYPE_DATA,
		  PLAYER_GRIPPER_DATA_STATE,
		  (void*)&(this->miabot_data.gripper), 
		  sizeof(player_gripper_data_t),
		  NULL);
    this->Publish(this->lift_id,
		  PLAYER_MSGTYPE_DATA,
		  PLAYER_GRIPPER_DATA_STATE,
		  (void*)&(this->miabot_data.lift), 
		  sizeof(player_gripper_data_t),
		  NULL);
    this->Publish(this->photodiode_id,
		  PLAYER_MSGTYPE_DATA,
		  PLAYER_SONAR_DATA_RANGES,
		  (void*)&(this->miabot_data.photodiode), 
		  sizeof(player_sonar_data_t),
		  NULL);
    this->Publish(this->blobfinder_id,
		  PLAYER_MSGTYPE_DATA, 
		  PLAYER_BLOBFINDER_DATA_BLOBS,
		  (void*)&(this->miabot_data.blobfinder), 
		  sizeof(player_blobfinder_data_t), NULL);
    
    pthread_testcancel();   
   
  }
  pthread_exit(NULL);    
   
}



void Miabot::update_everything(player_miabot_data_t* d){
 
  

  if(this->s_cmap == false){ // only look for data if miabot not busy sending colour map image params
   
    readData(swait);
    // update odometry readings
    if(this->position2d_subscriptions){
      pthread_testcancel();
      updateOdom(d); // just fills velocities and stall
      d->position2d.pos.px = this->position_x;
      d->position2d.pos.py = this->position_y;
      d->position2d.pos.pa = this->position_a;
      pthread_testcancel();
      
    }
    
    // update compass reading
    /*if(this->compass_subscriptions){
      pthread_testcancel();
      compass(d);
      pthread_testcancel();
      
      
      }*/
    // update sonar/photodiode readings
    if(this->sonar_subscriptions || this->photodiode_subscriptions){
      pthread_testcancel();
      DoScan(d,this->photodiode_subscriptions,this->sonar_subscriptions,buffer);
      pthread_testcancel();
    }
    
    //update gripper readings
    if(this->gripper_subscriptions && this->lift_subscriptions){
      pthread_testcancel();
      updateGripper(d);
      pthread_testcancel();
    }
    
    // update blobfinder readings
    if(this->blobfinder_subscriptions){       
      pthread_testcancel();
      blob(d);
      pthread_testcancel();   
    }
  }
}
// Command handler functions
int Miabot::HandleCommand(player_msghdr * hdr, void* data)
{
  int retVal = -1;

  if(Message::MatchMessage(hdr,
			   PLAYER_MSGTYPE_CMD,
			   PLAYER_POSITION2D_CMD_VEL,
			   this->position2d_id))
    {
      
      // get and send the latest motor command
      
      player_position2d_cmd_vel_t position2d_cmd;
      position2d_cmd = *(player_position2d_cmd_vel_t*)data;
      HandlePositionCommand(position2d_cmd);
      retVal = 0;
    }else if(Message::MatchMessage(hdr,
				   PLAYER_MSGTYPE_CMD,
				   PLAYER_POSITION2D_CMD_POS,
				   this->position2d_id))
      {
	
	// get and send the latest motor command
	
	player_position2d_cmd_pos_t position2d_cmd;
	position2d_cmd = *(player_position2d_cmd_pos_t*)data;
	HandlePositionCommand_pos(position2d_cmd);
	retVal = 0;
      
  
      }else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_CMD, -1, gripper_id))
	{
	  retVal = HandleGripperCommand (hdr, data);
	  
	
	}else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_CMD, -1, lift_id))
	{
	  retVal = HandleLiftCommand (hdr, data);
	  
	}
  
  return retVal;
}


int Miabot::HandleGripperCommand(player_msghdr * hdr, void * data)
{ 

if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_OPEN, this->gripper_id))
    {
      OpenGripper(this->open_arg);
      return 0;
    }
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_CLOSE, this->gripper_id))
    {
      CloseGripper(this->close_arg);
      return 0;
    }
}

int Miabot::HandleLiftCommand(player_msghdr * hdr, void * data)
{ 

if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_OPEN, this->lift_id))
    {
      UpGripper(this->up_arg);
      return 0;
    }
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_CLOSE, this->lift_id))
    {
      DownGripper(this->down_arg);
      return 0;
    }
}

// Config handler functions

int Miabot::HandleConfig(QueuePointer & resp_queue, player_msghdr * hdr,void * data)
{
 
  //sonar
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_SONAR_REQ_GET_GEOM,this->sonar_id)){
    if(hdr->size != 0)
      {
	PLAYER_WARN("Arg get sonar geom is wrong size; ignoring");
	return(-1);
      }
    
    player_sonar_geom_t sonar_geom;
    
    sonar_geom.poses_count = MiabotRobotParams[0].SonarNum;
    sonar_geom.poses = new player_pose3d_t[sonar_geom.poses_count];
    for (int i = 0; i < MiabotRobotParams[0].SonarNum; i++)
      {
	sonar_pose_t sonar_pose = MiabotRobotParams[0].sonar_pose[i];
	sonar_geom.poses[i].px = sonar_pose.x; 
	sonar_geom.poses[i].py = sonar_pose.y; 
	sonar_geom.poses[i].pyaw = sonar_pose.th;   
	
      }
    
    this->Publish(this->sonar_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype, &sonar_geom,sizeof(sonar_geom),NULL);
    delete sonar_geom.poses;
    return 0;



    //photodiode
    }else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_SONAR_REQ_GET_GEOM,this->photodiode_id)){
    if(hdr->size != 0)
      {
	PLAYER_WARN("Arg get photodiode geom is wrong size; ignoring");
	return(-1);
      }
    
    player_sonar_geom_t photodiode_geom;
    
    photodiode_geom.poses_count = MiabotRobotParams[0].SonarNum;
    photodiode_geom.poses = new player_pose3d_t[photodiode_geom.poses_count];
    for (int i = 0; i < MiabotRobotParams[0].SonarNum; i++)
      {
	sonar_pose_t photodiode_pose = MiabotRobotParams[0].sonar_pose[i];
	photodiode_geom.poses[i].px = photodiode_pose.x; 
	photodiode_geom.poses[i].py = photodiode_pose.y; 
	photodiode_geom.poses[i].pyaw = photodiode_pose.th;   
	
	}
    
    this->Publish(this->photodiode_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype, &photodiode_geom,sizeof(photodiode_geom),NULL);
    delete photodiode_geom.poses;
    return 0;



    //position

    //SET ODOM
    }else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_SET_ODOM,this->position2d_id)){
    if(hdr->size != sizeof(player_position2d_set_odom_req_t))
      {
	PLAYER_WARN("Arg to odometry set requests wrong size; ignoring");
	return(-1);
      }
    
    
    player_position2d_set_odom_req* set_odom_req = (player_position2d_set_odom_req*) data;
    
    puts("set odometry");
    set_odometry(set_odom_req->pose.px,set_odom_req->pose.py,set_odom_req->pose.pa);
    
    this->Publish(this->position2d_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype);
    return 0;

    //RESET ODOM
  }else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_RESET_ODOM,this->position2d_id)){
    if(hdr->size != 0)
      {
	PLAYER_WARN("Arg to reset position request is wrong size; ignoring");
	return(-1);
      }
    puts("reset odometry");
    this->reset_odometry();
    
    this->Publish(this->position2d_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype);
    return 0;

    //GET GEOM
  }else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_GET_GEOM,this->position2d_id)){
    if(hdr->size != 0)
      {
	PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
	return(-1);
      }
    puts("get geometry");
    player_position2d_geom_t position_geom;
   
    position_geom.pose.px = 0;
    position_geom.pose.py = 0;
    position_geom.pose.pyaw = 0;
   
    position_geom.size.sl = MiabotRobotParams[0].RobotLength;
    position_geom.size.sw = MiabotRobotParams[0].RobotWidth;
  
    this->Publish(this->position2d_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype, &position_geom,sizeof(position_geom));
   
    return 0;
 
    //REQ POWER
  }else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,PLAYER_POSITION2D_REQ_MOTOR_POWER,this->position2d_id))
  {
    /* motor state change request
     *   1 = enable motors
     *   0 = disable motors (default)
     */
    puts("set power");
    if(hdr->size != sizeof(player_position2d_power_config_t))
    {
      PLAYER_WARN("Arg to motor state change request wrong size; ignoring");
      return(-1);
    }
    player_position2d_power_config_t* power_config =
            (player_position2d_power_config_t*)data;
    this->ToggleMotorPower(power_config->state);

    this->Publish(this->position2d_id, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
    return 0;
  



      //gripper 
    
  }else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_GRIPPER_REQ_GET_GEOM,this->gripper_id)){
  
    if(hdr->size != 0)
      {
	PLAYER_WARN("Arg get gripper geom is wrong size; ignoring");
      
	return(-1);
      }
    player_gripper_geom_t gripper_geom;
    
    
    gripper_geom.pose.px = MiabotRobotParams[0].gripper_pose[0];
    gripper_geom.pose.py = MiabotRobotParams[0].gripper_pose[1];
    gripper_geom.pose.pz = MiabotRobotParams[0].gripper_pose[2]; 
    gripper_geom.outer_size.sl = MiabotRobotParams[0].gripperl;
    gripper_geom.outer_size.sw = MiabotRobotParams[0].gripperw;
    gripper_geom.outer_size.sh = MiabotRobotParams[0].gripperh;
    gripper_geom.inner_size.sl = gripper_geom.outer_size.sl - 0.015;
    gripper_geom.inner_size.sw = gripper_geom.outer_size.sw;
    gripper_geom.inner_size.sh = gripper_geom.outer_size.sh;
  
    this->Publish(this->gripper_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype, &gripper_geom,sizeof(gripper_geom));
    return 0;
    
     //lift

  }else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_GRIPPER_REQ_GET_GEOM,this->lift_id)){
  
    if(hdr->size != 0)
      {
	PLAYER_WARN("Arg get gripper geom is wrong size; ignoring");
      
	return(-1);
      }
    player_gripper_geom_t gripper_geom;
    
    
    gripper_geom.pose.px = MiabotRobotParams[0].gripper_pose[0];
    gripper_geom.pose.py = MiabotRobotParams[0].gripper_pose[1];
    gripper_geom.pose.pz = MiabotRobotParams[0].gripper_pose[2]; 
    gripper_geom.outer_size.sl = MiabotRobotParams[0].gripperl;
    gripper_geom.outer_size.sw = MiabotRobotParams[0].gripperw;
    gripper_geom.outer_size.sh = MiabotRobotParams[0].gripperh;
    gripper_geom.inner_size.sl = gripper_geom.outer_size.sl - 0.015;
    gripper_geom.inner_size.sw = gripper_geom.outer_size.sw;
    gripper_geom.inner_size.sh = gripper_geom.outer_size.sh;
  
    this->Publish(this->lift_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype, &gripper_geom,sizeof(gripper_geom));
    return 0;

    //compass
  }else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_GET_GEOM,this->compass_id)){
    if(hdr->size != 0)
      {
	PLAYER_WARN("Arg get robot (compass) geom is wrong size; ignoring");
	return(-1);
      }
    
    player_position2d_geom_t compass_geom;
    compass_geom.pose.px = 0;
    compass_geom.pose.py = 0;
    compass_geom.pose.pyaw = 0;
      
    compass_geom.size.sl = MiabotRobotParams[0].RobotLength;
    compass_geom.size.sw = MiabotRobotParams[0].RobotWidth;
    
    this->Publish(this->compass_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype, &compass_geom,sizeof(compass_geom));
    return 0;
    



    //blobfinder    
  }else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,PLAYER_BLOBFINDER_REQ_SET_COLOR,this->blobfinder_id)){
    // Set the tracking color (RGB max/min values)
    
    if(hdr->size != sizeof(player_blobfinder_color_config_t))
      {
	puts("Arg to blobfinder color request wrong size; ignoring");
	return(-1);
      }

    //stop tracking
    
  
    camStopTracking();
    
    puts("sending colour map");
    this->s_cmap = true;

    camTrack(this->set_cam,this->num_blobs,this->color[0].redmin,this->color[0].redmax,this->color[1].redmin,this->color[1].redmax,this->color[2].redmin,this->color[2].redmax,this->color[3].redmin,this->color[3].redmax,this->color[4].redmin,this->color[4].redmax,this->color[5].redmin,this->color[5].redmax,this->color[6].redmin,this->color[6].redmax,this->color[7].redmin,this->color[7].redmax,this->color[0].greenmin,this->color[0].greenmax,this->color[1].greenmin,this->color[1].greenmax,this->color[2].greenmin,this->color[2].greenmax,this->color[3].greenmin,this->color[3].greenmax,this->color[4].greenmin,this->color[4].greenmax,this->color[5].greenmin,this->color[5].greenmax,this->color[6].greenmin,this->color[6].greenmax,this->color[7].greenmin,this->color[7].greenmax,this->color[0].bluemin,this->color[0].bluemax,this->color[1].bluemin,this->color[1].bluemax,this->color[2].bluemin,this->color[2].bluemax,this->color[3].bluemin,this->color[3].bluemax,this->color[4].bluemin,this->color[4].bluemax,this->color[5].bluemin,this->color[5].bluemax,this->color[6].bluemin,this->color[6].bluemax,this->color[7].bluemin,this->color[7].bluemax);  
    
    this->s_cmap = false;

    Track();
    
    this->Publish(this->blobfinder_id, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_BLOBFINDER_REQ_SET_COLOR);
    return 0;
   
  }else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,PLAYER_BLOBFINDER_REQ_SET_IMAGER_PARAMS,this->blobfinder_id)){
    // Set the imager control params
    if(hdr->size != sizeof(player_blobfinder_imager_config_t))
    {
      puts("Arg to blobfinder imager request wrong size; ignoring");
      return(-1);
    }
    player_blobfinder_imager_config_t* imager_config = (player_blobfinder_imager_config_t*)data;

    this->s_cmap = true;

    camStopTracking(); // Stop the current tracking.

   
    if(imager_config->autogain == 0){ //AUTO ADJ OFF
      this->adj = 0;
    }else{
      this->adj = 1;  //AUTO ADJ ON
    }
    if(imager_config->colormode == 0){ //RGB/AWB OFF
      this->white = 0;

    }else if(imager_config->colormode == 1){ //RGB/AWB ON
      this->white = 1;
    }
    
    camera_settings(this->white,this->adj,this->filt);
    printf("Blobfinder imager parameters updated.\n");
       
    this->s_cmap = false;

    Track();  // Restart tracking

    this->Publish(this->blobfinder_id, resp_queue,PLAYER_MSGTYPE_RESP_ACK,PLAYER_BLOBFINDER_REQ_SET_IMAGER_PARAMS);
    return(0);

  }else{
    PLAYER_WARN("unknown config request to miabot driver");
    return(-1);
  }
  //restart tracking
 
 
  Track();
 
}


void Miabot::reset_odometry() {
  printf("RESETING ODOM...\n");
  // clear encoders
  encoder();

}

/* toggle motors off, according to val */
void Miabot::ToggleMotorPower(bool val)
{
  if(val == false){
    stop();
  }
}

// Set up the underlying position device.
int
Miabot::SetupPosition()
{
  
  // Subscribe to the position device.
  if(!(this->position = deviceTable->GetDevice(this->tracker_id)))
  {
    PLAYER_ERROR("unable to locate suitable position device");
    return(-1);
  }
  if(this->position->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to position device");
    return(-1);
  }
  Message* msg;
  player_position2d_geom_t* geom;
  // Get the robot's geometry
  if(!(msg = this->position->Request(this->InQueue,
                                     PLAYER_MSGTYPE_REQ,
                                     PLAYER_POSITION2D_REQ_GET_GEOM,
                                     NULL, 0, NULL, false)) ||
     (msg->GetHeader()->size != sizeof(player_position2d_geom_t)))
  {
    PLAYER_ERROR("failed to get geometry of underlying position device");
    if(msg)
      delete msg;
    return(-1);
  }

  geom = (player_position2d_geom_t*)msg->GetPayload();
 
  return 0;
}

int
Miabot::ShutdownPosition()
{
  return(this->position->Unsubscribe(this->InQueue));
}

void
Miabot::ProcessPositionData(player_position2d_data_t* data)
{
  this->position_x = data->pos.px;
  this->position_y = data->pos.py;
  this->position_a = data->pos.pa;
}

Miabot::~Miabot (void)
{
  player_position2d_data_t_cleanup(&miabot_data.position2d);
  player_sonar_data_t_cleanup (&miabot_data.sonar);
  player_sonar_data_t_cleanup (&miabot_data.photodiode);
  player_gripper_data_t_cleanup (&miabot_data.gripper);
  player_gripper_data_t_cleanup (&miabot_data.lift);
  player_blobfinder_data_t_cleanup (&miabot_data.blobfinder);
  player_position2d_data_t_cleanup (&miabot_data.compass);
  

 
}

// Extra stuff for building a shared object.x

/* need the extern to avoid C++ name-mangling  */
extern "C" {
  int player_driver_init(DriverTable* table)
  {
    puts("miabot driver initializing");
    Miabot_Register(table);
    puts("miabot driver done");
    return(0);
  }
}

