/*  miabot_commands.cc
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
/*
 * miabot_commands.cc
 * Commands and Parser functions for the Miabot Pro
 */

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <netdb.h>

#include <pthread.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include "miabot_commands.h"
#include "miabot_params.h"
#include "miabot.h"
#include <time.h>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <time.h>

#include <libplayercore/playercore.h>

#define motor_max_speed 0.8 // 0.8 = 20% of max (4 m/s) for odom acc
 
using namespace std;

// general
int global_sk;
string sResponse;
int bt_wait = 260000; 
string inp_command;

// odom
int odomLength = 27;
player_pose2d_t pose;
double m_odo_x, m_odo_y, rad_odo_theta;
bool stall;

// sonar
int nSonars;
int sonarLength = 15;
string sResponseArray[8];
double last_ranges[8];

// gripper
bool sent_gripper_cmd;
int gripper;
bool openg = true,closeg = false,upg = true,downg = false;
int gripperState, liftState, beamState;;

// blobfinder
int colour1,colour2,colour3,colour4,colour5,colour6,colour7,colour8;
int blobcount;
int blobcolor[8];
int blobmx[8], blobmy[8];	// Centroid
int blobx1[8],blobx2[8], bloby1[8], bloby2[8];	// Bounding box
int blobarea[8], blobrange[8];	// Area and confidence
static player_blobfinder_blob_t avrcamBlob;
double vx, va;

///////////////////////////////////////////////////////////////
// bluetooth functions
void bluetooth(string bluetooth_router, string bluetooth_port){


if((global_sk = rfcomm_connect(bluetooth_router, bluetooth_port)) < 0){
    exit(1);
  }

}
/**
* Socket Connection. TCP/IP over RFCOMM.
*/
int rfcomm_connect(string bluetooth_router, string bluetooth_port){

  struct sockaddr_in addr;
  struct hostent *hostInfo;
  int sock;
  unsigned short int serverPort;

  hostInfo = gethostbyname(bluetooth_router.c_str());
  if (hostInfo == NULL) {
    printf("problem interpreting host: %s\n", bluetooth_router.c_str());
    return -1;
  }

  
  if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0){
    printf("\nsocket error\n");
        
    return -1;
  }
  
   
  addr.sin_family = hostInfo->h_addrtype;
  memcpy((char *) &addr.sin_addr.s_addr,hostInfo->h_addr_list[0], hostInfo->h_length);
  serverPort = atoi(bluetooth_port.c_str());
  addr.sin_port = htons(serverPort);

  if(connect(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    
    printf("\nconnection error\n");
    close(sock);
    return -1;
  }

 
  return sock;
  
}
void bluetooth_dongle(string bluetooth_mac){

  bdaddr_t btaddr;

  str2ba(bluetooth_mac.c_str(),&btaddr);
  if((global_sk = rfcomm_connect_dongle(BDADDR_ANY,&btaddr,1)) < 0){
    exit(1);
  }
}

int rfcomm_connect_dongle(bdaddr_t *src, bdaddr_t *dst, uint8_t channel){
  
  struct sockaddr_rc addr;
  int sock;

  if((sock = socket(PF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM)) < 0){
    printf("\nsocket error\n");
        
    return -1;
  }
  addr.rc_family = AF_BLUETOOTH;
  bacpy(&addr.rc_bdaddr, src);

  addr.rc_channel = 0;
  
  //if(bind(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    
   // printf("\nbind error\n");
   // close(sock);    
   // return -1;
   // }

  addr.rc_family = AF_BLUETOOTH;
  bacpy(&addr.rc_bdaddr, dst);
    
  addr.rc_channel = channel;
 
  if(connect(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    
    printf("\nconnection error\n");
    close(sock);
    return -1;
  }

  
  return sock;
}

// handle command functions

void HandlePositionCommand(player_position2d_cmd_vel_t position2d_cmd){
 
  int leftvel, rightvel;
  double speedDemand, turnRateDemand, rotational_term, DiffConvFactor, dleftvel, drightvel;

  DiffConvFactor = -6.67; // Ratio of angular velocity to wheel velocity difference N.B. this value may not be correct (only did a quick sample)
  
  vx = speedDemand = position2d_cmd.vel.px;
  va = turnRateDemand = position2d_cmd.vel.pa;
  
  // convert xspeed and yawspeed into wheelspeeds
  rotational_term = turnRateDemand / DiffConvFactor;        
  dleftvel = speedDemand + rotational_term;
  drightvel = speedDemand - rotational_term;
  
  // Apply wheel speed bounds 
  if(dleftvel > motor_max_speed){
    
    dleftvel = motor_max_speed;
    puts("Left wheel velocity threshholded!");
  }else if(dleftvel < -motor_max_speed){
    
    dleftvel = -motor_max_speed;
    puts("Left wheel velocity threshholded!");
  }
  
  if(drightvel > motor_max_speed){
    
    drightvel = motor_max_speed;
    puts("Right wheel velocity threshholded!");
    
  }else if(drightvel < -motor_max_speed){
    
    drightvel = -motor_max_speed;
    puts("Right wheel velocity threshholded!");
    
  }
    
  // convert m/s to speed vaule
  leftvel = m2speed(dleftvel);
  rightvel = m2speed(drightvel);
       
  
  // send the speed command
  ostringstream speedCommandBuf;
  string speedCommand = "[=";
  speedCommandBuf << (int)leftvel << "l," << (int)rightvel;
  speedCommand.append(speedCommandBuf.str());
  speedCommandBuf.clear();
  speedCommand.append("r]");

  if(send(global_sk,speedCommand.c_str(),speedCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }  
  

}

void HandlePositionCommand_pos(player_position2d_cmd_pos_t position2d_cmd){
 
  double x, y, z,a,b,c,A,hyp;
  int d, r, yaw,Z;
  string go;
  bool left,l,forward;
  int angle;

  angle = deg2ticks(RTOD(pose.pa));

  x = position2d_cmd.pos.px;
  y = position2d_cmd.pos.py;
  z = position2d_cmd.pos.pa;
 
  hyp = sqrt(((pose.py-y)*(pose.py-y))+((pose.px-x)*(pose.px-x))); // pythag
  c = hyp;
  b = 1;
  a = sqrt(((pose.py-y)*(pose.py-y))+((pose.px-x+1)*(pose.px-x+1))); // pythag

  if(x > pose.px){
    forward = true;
    if(pose.py-y < 0){
      A = 360 - RTOD(acos(((b*b)+(c*c)-(a*a))/(2*b*c))); // cosine
    }else{
      A = RTOD(acos(((b*b)+(c*c)-(a*a))/(2*b*c))); // cosine
    }
  }else{
    forward = false;
    if(pose.py-y < 0){
      A = 180 - RTOD(acos(((b*b)+(c*c)-(a*a))/(2*b*c))); // cosine
    }else{
      A = RTOD(acos(((b*b)+(c*c)-(a*a))/(2*b*c))) - 180; // cosine
    }
  }

  d = m2ticks(hyp);
  r = deg2ticks(A);
  if(r > angle){
    r = r-angle;
  }else if(r < angle){
    r = deg2ticks(360)-(angle-r);
  }

  Z = deg2ticks(RTOD(z));
 
  // keep in bounds
  while(r<0)
    r+=deg2ticks(360);
  while(r>deg2ticks(360))
    r-=deg2ticks(360);
   while(Z<0)
    Z=deg2ticks(360);
  while(Z>deg2ticks(360))
    Z-=deg2ticks(360);
  
  // rotate to desired yaw
  if(r == Z){
    yaw = 0;
  }else if(Z==0){
    if(r<deg2ticks(180)){
         
      yaw = r;
      left = true;
 
    }else{
      yaw = deg2ticks(360)-r;
      left = false;
    }
  }else if(r < Z){
    yaw = Z-r;
    left = false;
  }else if(r > Z){
    
    yaw = r-Z;
    left = true;
  }

  if(r > deg2ticks(180)){
    // run sequence (has to be sequnce else all commands arent done)
    
    r = deg2ticks(360)-r;
    l = false;
 
   }else{
 
    // run sequence (has to be sequnce else all commands arent done)
    l = true;
   
  }

  seq(d,r,yaw,l,left,forward);

  go = "[~]"; // do sequence
  
  if(send(global_sk,go.c_str(),go.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }
 
}


void OpenGripper(int open_arg){

  gripper = open_arg;

  string gripperCommand;
  ostringstream gripperCommandBuf;
   
  gripperCommand = "[P ";
  gripperCommandBuf << open_arg << "]";
  gripperCommand.append(gripperCommandBuf.str());
  gripperCommandBuf.clear();
  
  openg = true;
  closeg = false;
  
  gripperState = 1;

   
  if(send(global_sk,gripperCommand.c_str(),gripperCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  } 

}

void CloseGripper(int close_arg){

  gripper = close_arg;

  string gripperCommand;
  ostringstream gripperCommandBuf;
 
     
  gripperCommand = "[P ";
  gripperCommandBuf << close_arg << "]";
  gripperCommand.append(gripperCommandBuf.str());
  gripperCommandBuf.clear();
  
  openg = false;
  closeg = true;
  
  gripperState = 2;
  
  if(send(global_sk,gripperCommand.c_str(),gripperCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }
  
}


void UpGripper(int up_arg){

  string gripperCommand;
  ostringstream gripperCommandBuf;
   
  gripperCommand = "[P ";
  gripperCommandBuf << gripper << " " << up_arg << "]";
  gripperCommand.append(gripperCommandBuf.str());
  gripperCommandBuf.clear();

  upg = true;
  downg = false;

  liftState = 1;  
 
  if(send(global_sk,gripperCommand.c_str(),gripperCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  } 

}

void DownGripper(int down_arg){

  string gripperCommand;
  ostringstream gripperCommandBuf;

  gripperCommand = "[P ";
  gripperCommandBuf << gripper << " " << down_arg << "]";
  gripperCommand.append(gripperCommandBuf.str());
  gripperCommandBuf.clear();
  
  upg = false;
  downg = true;

  liftState = 2;
  
  if(send(global_sk,gripperCommand.c_str(),gripperCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  } 

}

// odometry functions

void odom(string inp_odom){
  
  
  string tickLeft, tickRight;
  int xl, xr;
  double m_xl, m_xr;
  double dist;
  double axel = 0.07;
  bool flagL, flagR, b = true;
  stringstream sbufL, sbufR;
  if(inp_odom.length()>= odomLength){
    if(inp_odom.find_first_of("+-")!=string::npos){
      
      tickLeft = inp_odom.substr(inp_odom.find_first_of("+-"),11);
      
      if(tickLeft.at(0)=='+'){
	flagL = true;
      }else{
	flagL = false;
      }
      if(tickLeft.find_first_of("123456789")!=string::npos){
	tickLeft = tickLeft.substr(tickLeft.find_first_of("123456789"));
      }else{
	tickLeft = "0";
      }
   
      
      inp_odom = inp_odom.substr(inp_odom.find_first_of("+-")+11);
      tickRight = inp_odom.substr(inp_odom.find_first_of("+-"),11);
      
      if(tickRight.at(0)=='+'){
	flagR = true;
      }else{
	flagR = false;
      }
      if(tickRight.find_first_of("123456789")!=string::npos){
	tickRight = tickRight.substr(tickRight.find_first_of("123456789"));
      }else{
	tickRight = "0";
      }
      
      
    }else{
      b = false;
    }
  }else{
    b = false;
  }

  if(b){
    
    sbufL << tickLeft;
    if(flagL){
      sbufL >> xl;
    }else{
      sbufL >> xl;
      xl = -(xl);
    }
    sbufL.clear();
  
    sbufR << tickRight;
    if(flagR){
      sbufR >> xr;
    }else{
      sbufR >> xr;
      xr = -(xr);
    }
    sbufR.clear();
    

    m_xl = ticks2m(xl);
    m_xr = ticks2m(xr);
    
    // http://rossum.sourceforge.net/papers/DiffSteer/#d6
  
    dist = (m_xr+m_xl)/2;
    pose.pa = (m_xr-m_xl)/(2*axel);
  
    // keep pose.pa in bounds
    while(pose.pa<0)
      pose.pa+=2*M_PI;
    while(pose.pa>2*M_PI)
    pose.pa-=2*M_PI;
    
    pose.px = dist * cos(pose.pa);
    pose.py = dist * sin(pose.pa);

  }//else{
    //printf("No Encoder values Recieved...\n");
    
  //}
    
  
  
}
   
// camera functions

void camera_settings(int white, int adj, int filt){
  string settings;

  settings = "[iA0=0,43,52,2D,"; // CR 45

  if(filt==1){
    settings.append("07,");
  }else{
    settings.append("03,");
  }

  settings.append("13,");

  if(adj==1){
    settings.append("01,");
  }else{
    settings.append("00,");
  }

  settings.append("12,");

  if(white==1){
    settings.append("2C,0D]");
  }else{
    settings.append("28,0D]");
  }
  //printf("%s\n",settings.c_str());
  if(send(global_sk,settings.c_str(),settings.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }  
  rec();
  //printf("%s\n",sResponse.c_str());
  
}

// send camera colour map
void camTrack(int set_cam,int num_blobs, int redmin,int redmax, int redmin1,int redmax1, int redmin2,int redmax2,int redmin3,int redmax3, int redmin4,int redmax4, int redmin5,int redmax5,int redmin6,int redmax6, int redmin7,int redmax7, int greenmin,int greenmax, int greenmin1,int greenmax1,int greenmin2,int greenmax2, int greenmin3,int greenmax3, int greenmin4,int greenmax4,int greenmin5,int greenmax5, int greenmin6,int greenmax6, int greenmin7,int greenmax7, int bluemin,int bluemax,int bluemin1,int bluemax1, int bluemin2,int bluemax2, int bluemin3,int bluemax3,int bluemin4,int bluemax4, int bluemin5,int bluemax5, int bluemin6,int bluemax6,int bluemin7,int bluemax7)
{
  
  string sred[16] = {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"};
  string sgreen[16] = {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"};
  string sblue[16] = {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"};
  int red[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int green[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int blue[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  int r1,r2,g1,g2,b1,b2,r1a,r2a,g1a,g2a,b1a,b2a,r1b,r2b,g1b,g2b,b1b,b2b,r1c,r2c,g1c,g2c,b1c,b2c,r1d,r2d,g1d,g2d,b1d,b2d,r1e,r2e,g1e,g2e,b1e,b2e,r1f,r2f,g1f,g2f,b1f,b2f,r1g,r2g,g1g,g2g,b1g,b2g;

  if(num_blobs==8){
    printf("TRACKING EIGHT COLOURS...\n");
  
    colour1 = rgb(redmin,redmax,greenmin,greenmax,bluemin,bluemax); 
    colour2 = rgb(redmin1,redmax1,greenmin1,greenmax1,bluemin1,bluemax1); 
    colour3 = rgb(redmin2,redmax2,greenmin2,greenmax2,bluemin2,bluemax2);
    colour4 = rgb(redmin3,redmax3,greenmin3,greenmax3,bluemin3,bluemax3);
    colour5 = rgb(redmin4,redmax4,greenmin4,greenmax4,bluemin4,bluemax4);
    colour6 = rgb(redmin5,redmax5,greenmin5,greenmax5,bluemin5,bluemax5);
    colour7 = rgb(redmin6,redmax6,greenmin6,greenmax6,bluemin6,bluemax6);
    colour8 = rgb(redmin7,redmax7,greenmin7,greenmax7,bluemin7,bluemax7);

    printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",colour1,colour2,colour3,colour4,colour5,colour6,colour7,colour8);

    r1 = value2index(redmin);
    r2 = value2index(redmax);
    g1 = value2index(greenmin);
    g2 = value2index(greenmax);
    b1 = value2index(bluemin);
    b2 = value2index(bluemax);
    r1a = value2index(redmin1);
    r2a = value2index(redmax1);
    g1a = value2index(greenmin1);
    g2a = value2index(greenmax1);
    b1a = value2index(bluemin1);
    b2a = value2index(bluemax1);
    r1b = value2index(redmin2);
    r2b = value2index(redmax2);
    g1b = value2index(greenmin2);
    g2b = value2index(greenmax2);
    b1b = value2index(bluemin2);
    b2b = value2index(bluemax2);
    r1c = value2index(redmin3);
    r2c = value2index(redmax3);
    g1c = value2index(greenmin3);
    g2c = value2index(greenmax3);
    b1c = value2index(bluemin3);
    b2c = value2index(bluemax3);
    r1d = value2index(redmin4);
    r2d = value2index(redmax4);
    g1d = value2index(greenmin4);
    g2d = value2index(greenmax4);
    b1d = value2index(bluemin4);
    b2d = value2index(bluemax4);
    r1e = value2index(redmin5);
    r2e = value2index(redmax5);
    g1e = value2index(greenmin5);
    g2e = value2index(greenmax5);
    b1e = value2index(bluemin5);
    b2e = value2index(bluemax5);
    r1f = value2index(redmin6);
    r2f = value2index(redmax6);
    g1f = value2index(greenmin6);
    g2f = value2index(greenmax6);
    b1f = value2index(bluemin6);
    b2f = value2index(bluemax6);
    r1g = value2index(redmin7);
    r2g = value2index(redmax7);
    g1g = value2index(greenmin7);
    g2g = value2index(greenmax7);
    b1g = value2index(bluemin7);
    b2g = value2index(bluemax7);

    for(int i = r1; i <= r2; i++){
      red[i] = 128;
    }
    for(int i = g1; i <= g2; i++){
      green[i] = 128;
    }
    for(int i = b1; i <= b2; i++){
      blue[i] = 128;
    }
    for(int i = r1a; i <= r2a; i++){
      red[i] += 64;
    }
    for(int i = g1a; i <= g2a; i++){
      green[i] += 64;
    }
    for(int i = b1a; i <= b2a; i++){
      blue[i] += 64;
    }
    for(int i = r1b; i <= r2b; i++){
      red[i] += 32;
    }
    for(int i = g1b; i <= g2b; i++){
      green[i] += 32;
    }
    for(int i = b1b; i <= b2b; i++){
      blue[i] += 32;
    }
    for(int i = r1c; i <= r2c; i++){
      red[i] += 16;
    }
    for(int i = g1c; i <= g2c; i++){
      green[i] += 16;
    }
    for(int i = b1c; i <= b2c; i++){
      blue[i] += 16;
    }
    for(int i = r1d; i <= r2d; i++){
      red[i] += 8;
    }
    for(int i = g1d; i <= g2d; i++){
      green[i] += 8;
    }
    for(int i = b1d; i <= b2d; i++){
      blue[i] += 8;
    }
    for(int i = r1e; i <= r2e; i++){
      red[i] += 4;
    }
    for(int i = g1e; i <= g2e; i++){
      green[i] += 4;
    }
    for(int i = b1e; i <= b2e; i++){
      blue[i] += 4;
    }
    for(int i = r1f; i <= r2f; i++){
      red[i] += 2;
    }
    for(int i = g1f; i <= g2f; i++){
      green[i] += 2;
    }
    for(int i = b1f; i <= b2f; i++){
      blue[i] += 2;
    }
    for(int i = r1g; i <= r2g; i++){
      red[i] += 1;
    }
    for(int i = g1g; i <= g2g; i++){
      green[i] += 1;
    }
    for(int i = b1g; i <= b2g; i++){
      blue[i] += 1;
    }

    for(int i = 0; i < 16; i++){
      sred[i] = hexbyte2strB(red[i]);
      sgreen[i] = hexbyte2strB(green[i]);
      sblue[i] = hexbyte2strB(blue[i]);
    }

  }else if(num_blobs==7){
    printf("TRACKING SEVEN COLOURS...\n");
  
    colour1 = rgb(redmin,redmax,greenmin,greenmax,bluemin,bluemax); 
    colour2 = rgb(redmin1,redmax1,greenmin1,greenmax1,bluemin1,bluemax1); 
    colour3 = rgb(redmin2,redmax2,greenmin2,greenmax2,bluemin2,bluemax2);
    colour4 = rgb(redmin3,redmax3,greenmin3,greenmax3,bluemin3,bluemax3);
    colour5 = rgb(redmin4,redmax4,greenmin4,greenmax4,bluemin4,bluemax4);
    colour6 = rgb(redmin5,redmax5,greenmin5,greenmax5,bluemin5,bluemax5);
    colour7 = rgb(redmin6,redmax6,greenmin6,greenmax6,bluemin6,bluemax6);

    printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",colour1,colour2,colour3,colour4,colour5,colour6,colour7);

    r1 = value2index(redmin);
    r2 = value2index(redmax);
    g1 = value2index(greenmin);
    g2 = value2index(greenmax);
    b1 = value2index(bluemin);
    b2 = value2index(bluemax);
    r1a = value2index(redmin1);
    r2a = value2index(redmax1);
    g1a = value2index(greenmin1);
    g2a = value2index(greenmax1);
    b1a = value2index(bluemin1);
    b2a = value2index(bluemax1);
    r1b = value2index(redmin2);
    r2b = value2index(redmax2);
    g1b = value2index(greenmin2);
    g2b = value2index(greenmax2);
    b1b = value2index(bluemin2);
    b2b = value2index(bluemax2);
    r1c = value2index(redmin3);
    r2c = value2index(redmax3);
    g1c = value2index(greenmin3);
    g2c = value2index(greenmax3);
    b1c = value2index(bluemin3);
    b2c = value2index(bluemax3);
    r1d = value2index(redmin4);
    r2d = value2index(redmax4);
    g1d = value2index(greenmin4);
    g2d = value2index(greenmax4);
    b1d = value2index(bluemin4);
    b2d = value2index(bluemax4);
    r1e = value2index(redmin5);
    r2e = value2index(redmax5);
    g1e = value2index(greenmin5);
    g2e = value2index(greenmax5);
    b1e = value2index(bluemin5);
    b2e = value2index(bluemax5);
    r1f = value2index(redmin6);
    r2f = value2index(redmax6);
    g1f = value2index(greenmin6);
    g2f = value2index(greenmax6);
    b1f = value2index(bluemin6);
    b2f = value2index(bluemax6);

    for(int i = r1; i <= r2; i++){
      red[i] = 128;
    }
    for(int i = g1; i <= g2; i++){
      green[i] = 128;
    }
    for(int i = b1; i <= b2; i++){
      blue[i] = 128;
    }
    for(int i = r1a; i <= r2a; i++){
      red[i] += 64;
    }
    for(int i = g1a; i <= g2a; i++){
      green[i] += 64;
    }
    for(int i = b1a; i <= b2a; i++){
      blue[i] += 64;
    }
    for(int i = r1b; i <= r2b; i++){
      red[i] += 32;
    }
    for(int i = g1b; i <= g2b; i++){
      green[i] += 32;
    }
    for(int i = b1b; i <= b2b; i++){
      blue[i] += 32;
    }
    for(int i = r1c; i <= r2c; i++){
      red[i] += 16;
    }
    for(int i = g1c; i <= g2c; i++){
      green[i] += 16;
    }
    for(int i = b1c; i <= b2c; i++){
      blue[i] += 16;
    }
    for(int i = r1d; i <= r2d; i++){
      red[i] += 8;
    }
    for(int i = g1d; i <= g2d; i++){
      green[i] += 8;
    }
    for(int i = b1d; i <= b2d; i++){
      blue[i] += 8;
    }
    for(int i = r1e; i <= r2e; i++){
      red[i] += 4;
    }
    for(int i = g1e; i <= g2e; i++){
      green[i] += 4;
    }
    for(int i = b1e; i <= b2e; i++){
      blue[i] += 4;
    }
    for(int i = r1f; i <= r2f; i++){
      red[i] += 2;
    }
    for(int i = g1f; i <= g2f; i++){
      green[i] += 2;
    }
    for(int i = b1f; i <= b2f; i++){
      blue[i] += 2;
    }
   
    for(int i = 0; i < 16; i++){
      sred[i] = hexbyte2strB(red[i]);
      sgreen[i] = hexbyte2strB(green[i]);
      sblue[i] = hexbyte2strB(blue[i]);
    }

  }else if(num_blobs==6){
    printf("TRACKING SIX COLOURS...\n");
  
    colour1 = rgb(redmin,redmax,greenmin,greenmax,bluemin,bluemax); 
    colour2 = rgb(redmin1,redmax1,greenmin1,greenmax1,bluemin1,bluemax1); 
    colour3 = rgb(redmin2,redmax2,greenmin2,greenmax2,bluemin2,bluemax2);
    colour4 = rgb(redmin3,redmax3,greenmin3,greenmax3,bluemin3,bluemax3);
    colour5 = rgb(redmin4,redmax4,greenmin4,greenmax4,bluemin4,bluemax4);
    colour6 = rgb(redmin5,redmax5,greenmin5,greenmax5,bluemin5,bluemax5);

    printf("%d\t%d\t%d\t%d\t%d\t%d\n",colour1,colour2,colour3,colour4,colour5,colour6);

    r1 = value2index(redmin);
    r2 = value2index(redmax);
    g1 = value2index(greenmin);
    g2 = value2index(greenmax);
    b1 = value2index(bluemin);
    b2 = value2index(bluemax);
    r1a = value2index(redmin1);
    r2a = value2index(redmax1);
    g1a = value2index(greenmin1);
    g2a = value2index(greenmax1);
    b1a = value2index(bluemin1);
    b2a = value2index(bluemax1);
    r1b = value2index(redmin2);
    r2b = value2index(redmax2);
    g1b = value2index(greenmin2);
    g2b = value2index(greenmax2);
    b1b = value2index(bluemin2);
    b2b = value2index(bluemax2);
    r1c = value2index(redmin3);
    r2c = value2index(redmax3);
    g1c = value2index(greenmin3);
    g2c = value2index(greenmax3);
    b1c = value2index(bluemin3);
    b2c = value2index(bluemax3);
    r1d = value2index(redmin4);
    r2d = value2index(redmax4);
    g1d = value2index(greenmin4);
    g2d = value2index(greenmax4);
    b1d = value2index(bluemin4);
    b2d = value2index(bluemax4);
    r1e = value2index(redmin5);
    r2e = value2index(redmax5);
    g1e = value2index(greenmin5);
    g2e = value2index(greenmax5);
    b1e = value2index(bluemin5);
    b2e = value2index(bluemax5);
    
    for(int i = r1; i <= r2; i++){
      red[i] = 128;
    }
    for(int i = g1; i <= g2; i++){
      green[i] = 128;
    }
    for(int i = b1; i <= b2; i++){
      blue[i] = 128;
    }
    for(int i = r1a; i <= r2a; i++){
      red[i] += 64;
    }
    for(int i = g1a; i <= g2a; i++){
      green[i] += 64;
    }
    for(int i = b1a; i <= b2a; i++){
      blue[i] += 64;
    }
    for(int i = r1b; i <= r2b; i++){
      red[i] += 32;
    }
    for(int i = g1b; i <= g2b; i++){
      green[i] += 32;
    }
    for(int i = b1b; i <= b2b; i++){
      blue[i] += 32;
    }
    for(int i = r1c; i <= r2c; i++){
      red[i] += 16;
    }
    for(int i = g1c; i <= g2c; i++){
      green[i] += 16;
    }
    for(int i = b1c; i <= b2c; i++){
      blue[i] += 16;
    }
    for(int i = r1d; i <= r2d; i++){
      red[i] += 8;
    }
    for(int i = g1d; i <= g2d; i++){
      green[i] += 8;
    }
    for(int i = b1d; i <= b2d; i++){
      blue[i] += 8;
    }
    for(int i = r1e; i <= r2e; i++){
      red[i] += 4;
    }
    for(int i = g1e; i <= g2e; i++){
      green[i] += 4;
    }
    for(int i = b1e; i <= b2e; i++){
      blue[i] += 4;
    }
   
    for(int i = 0; i < 16; i++){
      sred[i] = hexbyte2strB(red[i]);
      sgreen[i] = hexbyte2strB(green[i]);
      sblue[i] = hexbyte2strB(blue[i]);
    }

  }else if(num_blobs==5){
    printf("TRACKING FIVE COLOURS...\n");


    colour1 = rgb(redmin,redmax,greenmin,greenmax,bluemin,bluemax); 
    colour2 = rgb(redmin1,redmax1,greenmin1,greenmax1,bluemin1,bluemax1); 
    colour3 = rgb(redmin2,redmax2,greenmin2,greenmax2,bluemin2,bluemax2);
    colour4 = rgb(redmin3,redmax3,greenmin3,greenmax3,bluemin3,bluemax3);
    colour5 = rgb(redmin4,redmax4,greenmin4,greenmax4,bluemin4,bluemax4);

    printf("%d\t%d\t%d\t%d\t%d\n",colour1,colour2,colour3,colour4,colour5);

    r1 = value2index(redmin);
    r2 = value2index(redmax);
    g1 = value2index(greenmin);
    g2 = value2index(greenmax);
    b1 = value2index(bluemin);
    b2 = value2index(bluemax);
    r1a = value2index(redmin1);
    r2a = value2index(redmax1);
    g1a = value2index(greenmin1);
    g2a = value2index(greenmax1);
    b1a = value2index(bluemin1);
    b2a = value2index(bluemax1);
    r1b = value2index(redmin2);
    r2b = value2index(redmax2);
    g1b = value2index(greenmin2);
    g2b = value2index(greenmax2);
    b1b = value2index(bluemin2);
    b2b = value2index(bluemax2);
    r1c = value2index(redmin3);
    r2c = value2index(redmax3);
    g1c = value2index(greenmin3);
    g2c = value2index(greenmax3);
    b1c = value2index(bluemin3);
    b2c = value2index(bluemax3);
    r1d = value2index(redmin4);
    r2d = value2index(redmax4);
    g1d = value2index(greenmin4);
    g2d = value2index(greenmax4);
    b1d = value2index(bluemin4);
    b2d = value2index(bluemax4);

    for(int i = r1; i <= r2; i++){
      red[i] = 128;
    }
    for(int i = g1; i <= g2; i++){
      green[i] = 128;
    }
    for(int i = b1; i <= b2; i++){
      blue[i] = 128;
    }
    for(int i = r1a; i <= r2a; i++){
      red[i] += 64;
    }
    for(int i = g1a; i <= g2a; i++){
      green[i] += 64;
    }
    for(int i = b1a; i <= b2a; i++){
      blue[i] += 64;
    }
    for(int i = r1b; i <= r2b; i++){
      red[i] += 32;
    }
    for(int i = g1b; i <= g2b; i++){
      green[i] += 32;
    }
    for(int i = b1b; i <= b2b; i++){
      blue[i] += 32;
    }
    for(int i = r1c; i <= r2c; i++){
      red[i] += 16;
    }
    for(int i = g1c; i <= g2c; i++){
      green[i] += 16;
    }
    for(int i = b1c; i <= b2c; i++){
      blue[i] += 16;
    }
    for(int i = r1d; i <= r2d; i++){
      red[i] += 8;
    }
    for(int i = g1d; i <= g2d; i++){
      green[i] += 8;
    }
    for(int i = b1d; i <= b2d; i++){
      blue[i] += 8;
    }

    for(int i = 0; i < 16; i++){
      sred[i] = hexbyte2strB(red[i]);
      sgreen[i] = hexbyte2strB(green[i]);
      sblue[i] = hexbyte2strB(blue[i]);
    }

  }else if(num_blobs==4){
    printf("TRACKING FOUR COLOURS...\n");
 
    colour1 = rgb(redmin,redmax,greenmin,greenmax,bluemin,bluemax); 
    colour2 = rgb(redmin1,redmax1,greenmin1,greenmax1,bluemin1,bluemax1); 
    colour3 = rgb(redmin2,redmax2,greenmin2,greenmax2,bluemin2,bluemax2);
    colour4 = rgb(redmin3,redmax3,greenmin3,greenmax3,bluemin3,bluemax3);

    printf("%d\t%d\t%d\t%d\n",colour1,colour2,colour3,colour4);

    r1 = value2index(redmin);
    r2 = value2index(redmax);
    g1 = value2index(greenmin);
    g2 = value2index(greenmax);
    b1 = value2index(bluemin);
    b2 = value2index(bluemax);
    r1a = value2index(redmin1);
    r2a = value2index(redmax1);
    g1a = value2index(greenmin1);
    g2a = value2index(greenmax1);
    b1a = value2index(bluemin1);
    b2a = value2index(bluemax1);
    r1b = value2index(redmin2);
    r2b = value2index(redmax2);
    g1b = value2index(greenmin2);
    g2b = value2index(greenmax2);
    b1b = value2index(bluemin2);
    b2b = value2index(bluemax2);
    r1c = value2index(redmin3);
    r2c = value2index(redmax3);
    g1c = value2index(greenmin3);
    g2c = value2index(greenmax3);
    b1c = value2index(bluemin3);
    b2c = value2index(bluemax3);

    for(int i = r1; i <= r2; i++){
      red[i] = 128;
    }
    for(int i = g1; i <= g2; i++){
      green[i] = 128;
    }
    for(int i = b1; i <= b2; i++){
      blue[i] = 128;
    }
    for(int i = r1a; i <= r2a; i++){
      red[i] += 64;
    }
    for(int i = g1a; i <= g2a; i++){
      green[i] += 64;
    }
    for(int i = b1a; i <= b2a; i++){
      blue[i] += 64;
    }
    for(int i = r1b; i <= r2b; i++){
      red[i] += 32;
    }
    for(int i = g1b; i <= g2b; i++){
      green[i] += 32;
    }
    for(int i = b1b; i <= b2b; i++){
      blue[i] += 32;
    }
    for(int i = r1c; i <= r2c; i++){
      red[i] += 16;
    }
    for(int i = g1c; i <= g2c; i++){
      green[i] += 16;
    }
    for(int i = b1c; i <= b2c; i++){
      blue[i] += 16;
    }

    for(int i = 0; i < 16; i++){
      sred[i] = hexbyte2strB(red[i]);
      sgreen[i] = hexbyte2strB(green[i]);
      sblue[i] = hexbyte2strB(blue[i]);
    }

  }else if(num_blobs==3){
    printf("TRACKING THREE COLOURS...\n");
   
    colour1 = rgb(redmin,redmax,greenmin,greenmax,bluemin,bluemax); 
    colour2 = rgb(redmin1,redmax1,greenmin1,greenmax1,bluemin1,bluemax1); 
    colour3 = rgb(redmin2,redmax2,greenmin2,greenmax2,bluemin2,bluemax2);
 
    printf("%d\t%d\t%d\n",colour1,colour2,colour3);
    
    r1 = value2index(redmin);
    r2 = value2index(redmax);
    g1 = value2index(greenmin);
    g2 = value2index(greenmax);
    b1 = value2index(bluemin);
    b2 = value2index(bluemax);
    r1a = value2index(redmin1);
    r2a = value2index(redmax1);
    g1a = value2index(greenmin1);
    g2a = value2index(greenmax1);
    b1a = value2index(bluemin1);
    b2a = value2index(bluemax1);
    r1b = value2index(redmin2);
    r2b = value2index(redmax2);
    g1b = value2index(greenmin2);
    g2b = value2index(greenmax2);
    b1b = value2index(bluemin2);
    b2b = value2index(bluemax2);

    for(int i = r1; i <= r2; i++){
      red[i] = 128;
    }
    for(int i = g1; i <= g2; i++){
      green[i] = 128;
    }
    for(int i = b1; i <= b2; i++){
      blue[i] = 128;
    }
    for(int i = r1a; i <= r2a; i++){
      red[i] += 64;
    }
    for(int i = g1a; i <= g2a; i++){
      green[i] += 64;
    }
    for(int i = b1a; i <= b2a; i++){
      blue[i] += 64;
    }
    for(int i = r1b; i <= r2b; i++){
      red[i] += 32;
    }
    for(int i = g1b; i <= g2b; i++){
      green[i] += 32;
    }
    for(int i = b1b; i <= b2b; i++){
      blue[i] += 32;
    }

    for(int i = 0; i < 16; i++){
      sred[i] = hexbyte2strB(red[i]);
      sgreen[i] = hexbyte2strB(green[i]);
      sblue[i] = hexbyte2strB(blue[i]);
    }

  }else if(num_blobs==2){
    printf("TRACKING TWO COLOURS...\n");
    
    colour1 = rgb(redmin,redmax,greenmin,greenmax,bluemin,bluemax); 
    colour2 = rgb(redmin1,redmax1,greenmin1,greenmax1,bluemin1,bluemax1); 
    printf("%d\t%d\n",colour1,colour2);

    r1 = value2index(redmin);
    r2 = value2index(redmax);
    g1 = value2index(greenmin);
    g2 = value2index(greenmax);
    b1 = value2index(bluemin);
    b2 = value2index(bluemax);
    r1a = value2index(redmin1);
    r2a = value2index(redmax1);
    g1a = value2index(greenmin1);
    g2a = value2index(greenmax1);
    b1a = value2index(bluemin1);
    b2a = value2index(bluemax1);

    for(int i = r1; i <= r2; i++){
      red[i] = 128;
    }
    for(int i = g1; i <= g2; i++){
      green[i] = 128;
    }
    for(int i = b1; i <= b2; i++){
      blue[i] = 128;
    }
    for(int i = r1a; i <= r2a; i++){
      red[i] += 64;
    }
    for(int i = g1a; i <= g2a; i++){
      green[i] += 64;
    }
    for(int i = b1a; i <= b2a; i++){
      blue[i] += 64;
    }
   
    for(int i = 0; i < 16; i++){
      sred[i] = hexbyte2strB(red[i]);
      sgreen[i] = hexbyte2strB(green[i]);
      sblue[i] = hexbyte2strB(blue[i]);
    }

  }else if(num_blobs==1){   
    printf("TRACKING ONE COLOUR...\n");
    
    colour1 = rgb(redmin,redmax,greenmin,greenmax,bluemin,bluemax); 
    printf("%d\n",colour1);
    r1 = value2index(redmin);
    r2 = value2index(redmax);
    g1 = value2index(greenmin);
    g2 = value2index(greenmax);
    b1 = value2index(bluemin);
    b2 = value2index(bluemax);

    for(int i = r1; i <= r2; i++){
      red[i] = 128;
    }
    for(int i = g1; i <= g2; i++){
      green[i] = 128;
    }
    for(int i = b1; i <= b2; i++){
      blue[i] = 128;
    }

    for(int i = 0; i < 16; i++){
      sred[i] = hexbyte2strB(red[i]);
      sgreen[i] = hexbyte2strB(green[i]);
      sblue[i] = hexbyte2strB(blue[i]);
    }
  }

  ostringstream rgb,rgb2,rgb3,rgb4,rgb5,rgb6,rgb7,rgb8,rgb9,rgb10,rgb11,rgb12,rgb13,rgb14,rgb15,rgb16,rgb17,rgb18,rgb19,rgb20,rgb21,rgb22,rgb23,rgb24;  
  
  rgb << sred[0] << "20," << sred[1] << "20";
  rgb2 << sred[2] << "20," << sred[3] << "20";
  rgb3 << sred[4] << "20," << sred[5] << "20";
  rgb4 << sred[6] << "20,"<< sred[7] << "20";
  rgb5 << sred[8] << "20," << sred[9] << "20";
  rgb6 << sred[10] << "20," << sred[11] << "20";
  rgb7 << sred[12] << "20,"<< sred[13] << "20";
  rgb8 << sred[14] << "20," <<  sred[15] << "20";
  rgb9 << sgreen[0] << "20," << sgreen[1] << "20";
  rgb10 << sgreen[2] << "20," << sgreen[3] << "20";
  rgb11 << sgreen[4] << "20," << sgreen[5] << "20";
  rgb12 << sgreen[6] << "20," << sgreen[7] << "20";
  rgb13 << sgreen[8] << "20," << sgreen[9] << "20";
  rgb14 << sgreen[10] << "20," << sgreen[11] << "20";
  rgb15 << sgreen[12] << "20," << sgreen[13] << "20";
  rgb16 << sgreen[14] << "20," << sgreen[15] << "20";
  rgb17 << sblue[0] << "20," << sblue[1] << "20";
  rgb18 << sblue[2] << "20," << sblue[3] << "20";
  rgb19 << sblue[4] << "20," << sblue[5] << "20";
  rgb20 << sblue[6] << "20," << sblue[7] << "20";
  rgb21 << sblue[8] << "20," << sblue[9] << "20";
  rgb22 << sblue[10] << "20," << sblue[11] << "20";
  rgb23 << sblue[12] << "20," << sblue[13] << "20";
  rgb24 << sblue[14] << "20," << sblue[15]; 
  string cameraCommand;
  
  if(set_cam==1){   
    printf("SENDING COLOUR MAP...\n");
    // has to be split up due to i2c max message length
    // SM
    cameraCommand = "[iA0=0,53,4D,20,";
    cameraCommand.append(rgb.str());
    cameraCommand.append("]");
    
    rgb.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec(); // have to wait for command to complete (yes for all 24)

    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb2.str());
    cameraCommand.append("]");
    rgb2.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb3.str());
    cameraCommand.append("]");
    rgb3.str("");
  
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb4.str());
    cameraCommand.append("]");
    rgb4.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb5.str());
    cameraCommand.append("]");
    rgb5.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
   
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb6.str());
    cameraCommand.append("]");
    rgb6.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb7.str());
    cameraCommand.append("]");
    rgb7.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb8.str());
    cameraCommand.append("]");
    rgb8.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb9.str());
    cameraCommand.append("]");
    rgb9.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb10.str());
    cameraCommand.append("]");
    rgb10.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
  }  
    rec();

    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb11.str());
    cameraCommand.append("]");
    rgb11.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb12.str());
    cameraCommand.append("]");
    rgb12.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb13.str());
    cameraCommand.append("]");
    rgb13.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb14.str());
    cameraCommand.append("]");
    rgb14.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb15.str());
    cameraCommand.append("]");
    rgb15.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb16.str());
    cameraCommand.append("]");
    rgb16.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb17.str());
    cameraCommand.append("]");
    rgb17.str("");
    
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb18.str());
    cameraCommand.append("]");
    rgb18.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb19.str());
    cameraCommand.append("]");
    rgb19.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb20.str());
    cameraCommand.append("]");
    rgb20.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb21.str());
    cameraCommand.append("]");
    rgb21.str("");

    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb22.str());
    cameraCommand.append("]");
    rgb22.str("");
    
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb23.str());
    cameraCommand.append("]");
    rgb23.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    
    cameraCommand = "[iA0=0,";
    cameraCommand.append(rgb24.str());
    cameraCommand.append("0D]");
    rgb24.str("");
 
    if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  
    rec();
    printf("DONE!\n");
  }
}

// start camera tracking
void Track()
{
  string cameraCommand;
  printf("START TRACKING...\n");

   
  cameraCommand = "[iA0=0,45,54,0D]"; //ET

  if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  } 
  
}


// stop camera tracking (same command as start)
void camStopTracking()
{
  
  printf("STOP TRACKING...\n");
   
  string cameraCommand;
  
  cameraCommand = "[iA0=0,44,54,0D]"; //DT 

  if(send(global_sk,cameraCommand.c_str(),cameraCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }   
  
}

// toggle sonar

void ToggleSonar(bool b, int ping){
  string sonarCommand;
  if(b){
   
    printf("START SONAR...\n");
    sonarCommand = "[S="; // ping every 250ms
    stringstream ss;
    ss << ping;// use set rate in config file
    sonarCommand.append(ss.str());
    sonarCommand.append("]");
  }else{
    printf("STOP SONAR...\n");
    sonarCommand = "[S]"; // stop ping
  }

  if(send(global_sk,sonarCommand.c_str(),sonarCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }   
}

// toggle odom

void ToggleOdom(bool b, int enc){
  string odomCommand;
  if(b){
   
    printf("START ODOM...\n");
    odomCommand = "[:="; 
    stringstream os;
    os << enc;// use set rate in config file
    odomCommand.append(os.str());
    odomCommand.append("]");
  }else{
    printf("STOP ODOM...\n");
    odomCommand = "[:]"; // stop read
  }

  if(send(global_sk,odomCommand.c_str(),odomCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }   
}

// update functions

void DoScan(player_miabot_data_t* d, int photodiode, int sonar, bool buffer){
  
  int p = photodiode;
  int s = sonar;
  string sCmd;

  d->sonar.ranges_count = nSonars;
  d->photodiode.ranges_count = nSonars;
  delete [] d->sonar.ranges;
  d->sonar.ranges = new float[d->sonar.ranges_count];
  delete [] d->photodiode.ranges;
  d->photodiode.ranges = new float[d->photodiode.ranges_count];
  for(int i = 0; i < d->sonar.ranges_count; i++){
    
    int x, y;
    
    if(p){
      y = str2hexbyte(sResponseArray[i].substr(0,2)); 
      
      d->photodiode.ranges[i] = y; //0-255 (dark-light)
	
    }
    if(s){
      x = str2hexbyte(sResponseArray[i].substr(3,2));
      x = x * 256 + str2hexbyte(sResponseArray[i].substr(6,2));
      
      // only grabs first echo
      
      
      d->sonar.ranges[i] = x*0.01; //cm2m
      if(buffer){
	if(d->sonar.ranges[i] < 0.03){
	  if(last_ranges[i] < 0.1){
	    d->sonar.ranges[i] = 0.03; // use min limit
	  }else{
	    d->sonar.ranges[i] = last_ranges[i]; // use stored reading
	  }
	}
	last_ranges[i] = d->sonar.ranges[i]; // store reading
	
      }
    }
          
  }

   
}

void compass(player_miabot_data_t* d){

  /*NOTE: results obtained from the compass are questionable due to interference from the motors, error correction on TODO list*/


  memset(&(d->compass),0,sizeof(d->compass));

  string compassCommand;
  
  compassCommand = "[iC0=2]";

  if(send(global_sk,compassCommand.c_str(),compassCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  } 

  //rec();

  compassCommand = "[iC0?2]";

  if(send(global_sk,compassCommand.c_str(),compassCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  } 
  
  //rec();
  
  if(sResponse.substr(0,2) == "i:"){
  	
    sResponse = sResponse.substr(2);      //strip off front
    
    int X;
    string a,b;
    
    a = sResponse.substr(0,2);
    b = sResponse.substr(3,2);
    
   
    X = str2hexbyte(a);
    X = X * 256 + str2hexbyte(b);
    X/=10; // 0-3599 represents 0-359.9 degs
         
       
    d->compass.pos.pa = DTOR(X);
    
  }else{
    printf("No compass data received...\n");
    d->compass.pos.pa = 0;
  }


}

void updateGripper(player_miabot_data_t* d){
 
  d->gripper.state = gripperState;
  d->lift.state = liftState;
  d->gripper.beams = beamState;
}

void updateOdom(player_miabot_data_t* d){
 
  player_pose2d_t new_vel;
  d->position2d.vel.px = vx;
  d->position2d.vel.pa = va;
  d->position2d.stall = stall;
  /*&double t;
    
  t = time();
      
  // velocity approximation based on distance travelled
  new_vel.px = ((pose.px - m_odo_x)/t)/(1/t);
  new_vel.py = ((pose.py - m_odo_y)/t)/(1/t);
  new_vel.pa = ((pose.pa - rad_odo_theta)/t)/(1/t);
    
  d->position2d.pos.px = pose.px;
  d->position2d.pos.py = pose.py;
  d->position2d.pos.pa = pose.pa;
  d->position2d.vel.px = new_vel.px;
  d->position2d.vel.pa = new_vel.pa;
  d->position2d.stall = stall;

  rad_odo_theta = pose.pa;
  m_odo_x = pose.px;
  m_odo_y = pose.py;*/
 
  //printf("%f\t%f\t%f\n",d->position2d.pos.px,d->position2d.pos.py, d->position2d.pos.pa);
}

// update blob data
void blob(player_miabot_data_t* d){
  
  d->blobfinder.blobs = &avrcamBlob;
  memset(d->blobfinder.blobs,0,
         sizeof(player_blobfinder_blob_t)*MAX_BLOBS);
  d->blobfinder.width = CAM_IMAGE_WIDTH;
  d->blobfinder.height = CAM_IMAGE_HEIGHT;

  if (blobcount >= 1)	
    {
      d->blobfinder.blobs_count = blobcount;
   
      for(int c = 0; c < blobcount; c++){
	if (!d->blobfinder.blobs)
	  d->blobfinder.blobs = new player_blobfinder_blob_t[c];
	  d->blobfinder.blobs[c].color = blobcolor[c];
	  d->blobfinder.blobs[c].left = blobx1[c];
	  d->blobfinder.blobs[c].top = bloby1[c];
	  d->blobfinder.blobs[c].right = blobx2[c];
	  d->blobfinder.blobs[c].bottom = bloby2[c];
	  d->blobfinder.blobs[c].x = blobmx[c];
	  d->blobfinder.blobs[c].y = blobmy[c];
	  d->blobfinder.blobs[c].area = blobarea[c];
	  d->blobfinder.blobs[c].range = 0; // not given
	  
      }
    }else{
      d->blobfinder.blobs_count = 0;
      
    }
}

// util functions

double ticks2m(int ticks){
  // 1 encode pulse ~= 0.04mm
  double pulses2m = 0.00004;
  double m;
  m = ticks*pulses2m;
 
  return m;
}

int m2ticks(double m){
  // 1 encode pulse ~= 0.04mm
  double m2pulses = 25000;
  int t;
  t = (int)(m*m2pulses);
 
  return t;
}

int deg2ticks(double d){
  // 1 encoder pulse ~= 14.82 degrees
  double deg2pulses = 14.82;
  int t;
  t = (int)(d*deg2pulses);
 
  return t;
}

int m2speed(double m){
  // 1m/s ~= 500
  int m2pulses = 500;
  int t;
  t = (int)(m*m2pulses);
 
  return t;
}

void stop(){

  string stop = "[s]";
  if(send(global_sk,stop.c_str(),stop.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
}

void p(int ping){
  printf("SETTING PING NUMBER...\n");
  stringstream pstream;
  pstream << (int)ping;
  string num = "[.rG1=";
  num.append(pstream.str());
  pstream.clear();
  num.append("]");
  if(send(global_sk,num.c_str(),num.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
 
  int s;
  int c = 0;
  s = ping;
  
  if(s >= 128){
    s-=128;
    c++;
  }
  if(s >= 64){
    s-=64;
    c++;
  }
  if(s >= 32){
    s-=32;
    c++;
    }
  if(s >= 16){
    s-=16;
    c++;
  }
  if(s >= 8){
    s-=8;
    c++;
  }
  if(s >= 4){
    s-=4;
    c++;
  }
  if(s >= 2){
    s-=2;
    c++;
  }
  if(s >= 1){
    s-=1;
    c++;
  }
  nSonars = c;
 
  printf("Using %d Sonar(s)\n",nSonars);
}

void defaults(){
  printf("SETTING DEFAULTS...\n");
  // ramp + anti squeel
  string defaults = "[.rT=10]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  defaults = "[.rI=10]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  defaults = "[.qX=2]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  defaults = "[.qT=10]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  // I2C retry settings 
  defaults = "[.iT=0]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  defaults = "[.iN=0]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  // use photodiode data
  defaults = "[.rB1=1]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  // set number of sonar bytes read
  defaults = "[.rBN=3]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  defaults = "[.ebP=3]";
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }
  rec();
  printf("%s\n",sResponse.c_str());
  // set sonar gain to max
  defaults = "[i00=1,0]";
  
  if(send(global_sk,defaults.c_str(),defaults.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }
  rec();
  if(sResponse.at(0)=='i'){
    printf("Setting Sonar Gain to Max\n");
  }
}

void encoder(){
  //zero encoders
  string encoder = "[;0,0]";
  if(send(global_sk,encoder.c_str(),encoder.length(),0) < 0){
    close(global_sk);
    puts("\nError: cannot send\n");
  }  
  //rec();
  //printf("%s\n",sResponse.c_str());
 
 
}

void set_odometry(float m_x, float m_y, float rad_theta) {
  m_odo_x=m_x;
  m_odo_y=m_y;
  rad_odo_theta=rad_theta;
}

string hexbyte2str(int n){ 
  
  string s;
  char buffer[2];
  sprintf(buffer,"%x",n);
  stringstream ss;
  ss << buffer;
  s = ss.str();
  ss.clear();
  return s;
}

string hexbyte2strB(int n){ 
  string s;

  if(n!=0){
    char buffer[3];
    char buffer2[2];
    stringstream ss;
    sprintf(buffer,"%d",n);
    for(int i = 0; i < 3; i++){
      if(buffer[i]!='\0'){
	sprintf(buffer2,"%x",buffer[i]);
	ss << buffer2 << ",";
      }
    }
    
    s = ss.str();
    ss.clear();
  }else{
    s = "30,";
  }
  return s;
}

int str2hexdig(char s){
  
  int n;
  n = (int)s;
  if((n >= (int)('a')) && (n <= (int)('f'))) {
    n = n + (int)('A') - (int)('a');
  }
  if((n >= (int)('0')) && (n <= (int)('9'))){
    n = n - (int)('0');
  }else if((n >= (int)('A')) && (n <= (int)('F'))) {
    n = n - (int)('A') + 10;
  }else{
    n = 0;
  }

  
  return n;
}

int str2hexbyte(string s){
  
  int n;
  n = str2hexdig(s.at(0));
  n = n * 16 + str2hexdig(s.at(1));
 
  return n;
}

// sequence forward, turn, stop
void seq(int f, int t, int r, bool b, bool l, bool forw){
  string go;
  stringstream forward, turn, rotate;
   
  go = "[$]"; // clear sequence
  
  if(send(global_sk,go.c_str(),go.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }  

  if(t != 0){
    if(b){ // right
      go = "[+d>"; // set turn
      turn << t << "]";
      go.append(turn.str());
      turn.clear();
      
      if(send(global_sk,go.c_str(),go.length(),0) < 0){
	close(global_sk);
	printf("\nError: cannot send\n");
      }  
      
      go = "[+>]"; // turn
      if(send(global_sk,go.c_str(),go.length(),0) < 0){
	close(global_sk);
	printf("\nError: cannot send\n");
      }
      
    }else{ // left
      go = "[+d<"; // set turn
      turn << t << "]";
      go.append(turn.str());
      turn.clear();
      
      if(send(global_sk,go.c_str(),go.length(),0) < 0){
	close(global_sk);
	printf("\nError: cannot send\n");
      }  
      
      go = "[+<]"; // turn
      if(send(global_sk,go.c_str(),go.length(),0) < 0){
	close(global_sk);
	printf("\nError: cannot send\n");
      }
    }
  }
  if(forw){ // forward
    go = "[+d^"; // set forward
    forward << f << "]";
    go.append(forward.str());
    forward.clear();
    
    if(send(global_sk,go.c_str(),go.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  

    
    go = "[+^]"; // forward
    if(send(global_sk,go.c_str(),go.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    } 
  }else{ // backward
    go = "[+dv"; // set backward
    forward << f << "]";
    go.append(forward.str());
    forward.clear();
    
    if(send(global_sk,go.c_str(),go.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }  

    
    go = "[+v]"; // backward
    if(send(global_sk,go.c_str(),go.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    } 
  }
   
  if(r != 0){
    if(l){ // left
      go = "[+d<"; // set rotate
      rotate << r << "]";
      go.append(rotate.str());
      rotate.clear();
      
      if(send(global_sk,go.c_str(),go.length(),0) < 0){
	close(global_sk);
	printf("\nError: cannot send\n");
      }  
      
      go = "[+<]"; // turn
      if(send(global_sk,go.c_str(),go.length(),0) < 0){
	close(global_sk);
	printf("\nError: cannot send\n");
      }
    }else{ // right
      go = "[+d>"; // set rotate
      rotate << r << "]";
      go.append(rotate.str());
      rotate.clear();
      
      if(send(global_sk,go.c_str(),go.length(),0) < 0){
	close(global_sk);
	printf("\nError: cannot send\n");
      }  
      
      go = "[+>]"; // turn
      if(send(global_sk,go.c_str(),go.length(),0) < 0){
	close(global_sk);
	printf("\nError: cannot send\n");
      }
    }
  }
  
}

//check if motors are active
void stopped(){
  bool v;
  string check;
  check = "[?]"; // check
  if(send(global_sk,check.c_str(),check.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  }
  //rec();
  
}


// min/max value to index value
int value2index(int value){

  int v;

  switch(value){

  case 0:
    v = 0;
    break;
  case 16:
    v = 1;
    break;
  case 32:
    v = 2;
    break;
  case 48:
    v = 3;
    break;
  case 64:
    v = 4;
    break;
  case 80:
    v = 5;
    break;
  case 96:
    v = 6;
    break;
  case 112:
    v = 7;
    break;
  case 128:
    v = 8;
    break;
  case 144:
    v = 9;
    break;
  case 160:
    v = 10;
    break;
  case 176:
    v = 11;
    break;
  case 192:
    v = 12;
    break;
  case 208:
    v = 13;
    break;
  case 224:
    v = 14;
    break;
  case 240:
    v = 15;
    

  }

  return v;
}

//time past in secs

double time(){

  double c, t, oc;
  c = clock();
  t = (c - oc)/CLOCKS_PER_SEC;
  oc = c;

  return t;
}

// rgb value

int rgb(int rmin, int rmax, int gmin, int gmax, int bmin, int bmax){

  // a descriptive color for the blob
  int colour;

  unsigned char red   = (rmin + rmax)/2; 
  unsigned char green = (gmin + gmax)/2;
  unsigned char blue  = (bmin + bmax)/2;
  
  colour = red << 16;
  colour += green << 8;
  colour += blue;
  
  return colour;
}

// socket recv functions

void rec(){
  
  // standard command response recv
  
  unsigned int start,finish;
 
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = bt_wait;
  
  //char buffer[RFCOMM_DEFAULT_MTU];
  char buffer[256];
  ostringstream sbuf; 
  
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(global_sk, &readfds); 
  
  // is there something to recv
  
  if(FD_ISSET(global_sk, &readfds)){
    
    
    while(select(global_sk+1, &readfds, NULL, NULL, &tv)){
      sbuf.clear();
      memset(buffer,0x0,sizeof(buffer)); 
      
      if(recv(global_sk,buffer,sizeof(buffer),0)<0){
	close(global_sk);
	printf("\nError: cannot rec\n");
	
	
      }else{
	
	sbuf << buffer;
	
      }
      
    }	
    
    inp_command = sbuf.str();
    
    start = inp_command.find('<')+1;
    finish = inp_command.find('>',start)-start;
    sResponse = inp_command.substr(start,finish);
    
  
  }else if(FD_ISSET(global_sk, &readfds)<0){
    printf("\nError: cannot select socket\n");
    
  }
}

void parseBlob(string inp_blob){

  //printf("%s\n",inp_blob.c_str());
  int length;
 

    //rip out the crap
    
    if(inp_blob.find('\n')!=string::npos){
      for(int j = 0; j < inp_blob.length(); j++){
	
	if(inp_blob.at(j)=='\n'){
	  inp_blob.erase(j,2);
	}
      }
    }
  
    if(inp_blob.find("<")!=string::npos){
      for(int j = 0; j < inp_blob.length(); j++){
	if(inp_blob.at(j)=='<'){
	  inp_blob.erase(j,1);
	}
      }
    }

    if(inp_blob.find("R")!=string::npos){
      for(int j = 0; j < inp_blob.length(); j++){
	if(inp_blob.at(j)=='R'){
	  inp_blob.erase(j,1);
	}
      }
    }

    if(inp_blob.find(">")!=string::npos){
      for(int j = 0; j < inp_blob.length(); j++){
	if(inp_blob.at(j)=='>'){
	  if(inp_blob.at(j-2)!='F' && inp_blob.at(j-1)!='F'){
	    inp_blob.erase(j,1);
	  }
	}
      }
    }
   
    if(inp_blob.find(" %c ")!=string::npos){
      for(int j = 0; j < inp_blob.length(); j++){
	if(j+2 <= inp_blob.length()){
	  if(inp_blob.at(j)==' ' && inp_blob.at(j+2)==' '){
	    inp_blob.erase(j,2);
	  }
	}
      }
    }
    
    //printf("%s\n",inp_blob.c_str());
    int count;
    int f;
    
    // could be a while loop instead of an if statement
    // would process all blob dataq received
    // tracking would not be as stable
    // slower
    if(inp_blob.find("0A")!=string::npos){ // start of blob maybe
      f = inp_blob.find("0A");
      if(inp_blob.find('>',f)!=string::npos){ // no > no data
	
	inp_blob = inp_blob.substr(f+3);
	count = (int)str2hexbyte(inp_blob.substr(0,2).c_str());
      
	// number of blobs
	blobcount = count;
	inp_blob = inp_blob.substr(3);
	length = blobcount*15;

	//printf("%s\n",inp_blob.c_str());
	
	if(inp_blob.length()>=length+3){

	  if(inp_blob.substr(length,3)=="FF>"){ //confirm end of blob

	    inp_blob = inp_blob.substr(0,length+3);
	    //printf("%s\n",inp_blob.c_str());	    
	    for(int c = 0; c < blobcount; c++){
	    // color
	      blobcolor[c] = (int)str2hexbyte(inp_blob.substr(0,2).c_str());
	      //find rgb from colour value
	      
	      switch(blobcolor[c]){
	      case 0:
		blobcolor[c]=colour1;
		break;
	      case 1:
		blobcolor[c]=colour2;
		break;
	      case 2:
		blobcolor[c]=colour3;
		break;
	      case 3:
		blobcolor[c]=colour4;
		break;
	      case 4:
		blobcolor[c]=colour5;
		break;
	      case 5:
		blobcolor[c]=colour6;
	      break;
	      case 6:
		blobcolor[c]=colour7;
		break;
	      case 7:
		blobcolor[c]=colour8;
		break;
		
	      }
	    
	      inp_blob = inp_blob.substr(3);
	      // left 
	      blobx1[c] = (int)str2hexbyte(inp_blob.substr(0,2).c_str());
	    
	      inp_blob = inp_blob.substr(3);
	      // top 
	      bloby1[c] = (int)str2hexbyte(inp_blob.substr(0,2).c_str());
	      
	      inp_blob = inp_blob.substr(3);
	      // right 
	      blobx2[c] = (int)str2hexbyte(inp_blob.substr(0,2).c_str());
	      
	      inp_blob = inp_blob.substr(3);
	      // bottom
	      bloby2[c] = (int)str2hexbyte(inp_blob.substr(0,2).c_str());
	      
	      inp_blob = inp_blob.substr(3);
	     
	      // x
	      blobmx[c]=(int)((blobx2[c]+blobx1[c])/2);
	      // y
	      blobmy[c]=(int)((bloby2[c]+bloby1[c])/2);
	      // area
	      blobarea[c] = (int)((blobx2[c]-blobx1[c]+1)*(bloby2[c]-bloby1[c]+1));
	      
	    }
	  }
	}
      }
    }
}

void parseSonar(string inp_sonar){
 
 
  int start, finish;
  string temp[nSonars];
  string address;
  for(int i = 0; i < nSonars; i++){    
    
    if(i == 0){
      address = "<SE0";
    }else if(i == 1){
      address = "<SE2";
    }else if(i == 2){
      address = "<SE4";
    }else if(i == 3){
      address = "<SE6";
    }else if(i == 4){
      address = "<SE8";
    }else if(i == 5){
      address = "<SEA";
    }else if(i == 6){
      address = "<SEC";
    }else if(i == 7){
      address = "<SEE";
    }
    if(inp_sonar.find(address)!=string::npos){
      start = inp_sonar.find(address)+1;
      
      if(inp_sonar.find('>',start)!=string::npos){
	finish = inp_sonar.find('>',start); 
      
	temp[i] = inp_sonar.substr(start,(finish-start));
	
	if(temp[i].length() < sonarLength-2){
	  temp[i] = address.substr(1);
	  temp[i].append(": 00 00 00");
	  
	}
      }else{
	temp[i] = address.substr(1);
	temp[i].append(": 00 00 00");
      }
    }else{
      temp[i] = address.substr(1);
      temp[i].append(": 00 00 00");
    }
  
   
    //printf("%s\n",temp[i].c_str());
  }
 
  int iSonar;
  
  for(int i = 0; i < nSonars; i++){  
    
    iSonar = str2hexbyte(temp[i].substr(1,2));
      
    iSonar = (iSonar - 224) / 2; 
      
    sResponseArray[iSonar] = temp[i];
    sResponseArray[iSonar] = sResponseArray[iSonar].substr(5);
    //printf("%s\n",sResponseArray[iSonar].c_str());
    
  }
 
}

void readData(int wait){
  
  string data = "";
  string inp_sonar = "";
  string inp_blob = "";
  string inp_odom = "";
  string inp_stall = "";
  string beamValue = "";
  int start, finish;

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = wait; //use set in config file
  
  //char buffer[RFCOMM_DEFAULT_MTU];
  char buffer[256];
  ostringstream sbuf; 
  
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(global_sk, &readfds); 
  
  // is there something to recv
  
  if(FD_ISSET(global_sk, &readfds)){
    
    
    while(select(global_sk+1, &readfds, NULL, NULL, &tv)){
      sbuf.clear();
      memset(buffer,0x0,sizeof(buffer)); 
      
      if(recv(global_sk,buffer,sizeof(buffer),0)<0){
	close(global_sk);
	printf("\nError: cannot rec\n");
	
	
      }else{
	
	sbuf << buffer;
	
      }
      
    }	
    
    data = sbuf.str();
 
     
    //printf("%s\n",data.c_str());
   
    string f = "<S"; //sonar
   
    string b = "<R 0A"; //blob

    string e = "<: "; //encoder

    string s = "<?"; //motor

    char end = '>';

    string i = "<i>"; //ack

    string v = "<V0";

    if(data.find(i)!=string::npos){
      while(data.find(i)!=string::npos){
	data.erase(data.find(i),data.find(i)+2);
      }
    }
    
    /*if(data.find(s)!=string::npos){
      while(data.find(s)!=string::npos){
	inp_stall = data.substr(data.find(s),5);
	data.erase(data.find(s),5);
      }
      //printf("%s\n",inp_stall.c_str());
      if(inp_stall.at(inp_stall.find(s)+3)=='1'){
	stall = false;
      }else if(inp_stall.at(inp_stall.find(s)+3)=='0'){
	stall = true;
      }
      }*/

    // look for encoder
    /*if(data.find(e)!=string::npos){
      while(data.find(e)!=string::npos){
	

	inp_odom.append(data.substr(data.find(e),odomLength));
	data.erase(data.find(e),odomLength);
	
      }
      
      //printf("%s\n",inp_odom.c_str());
      odom(inp_odom);
    }else{
      odom("");
      }*/
    // look for sonar
    if(data.find(f)!=string::npos){
      while(data.find(f)!=string::npos){
	
	
	inp_sonar.append(data.substr(data.find(f),sonarLength));
	data.erase(data.find(f),sonarLength);
	
      }
   
      //printf("%s\n",inp_sonar.c_str());
      parseSonar(inp_sonar);
    }else{
     
      parseSonar("");

    }

    //look for pill state
    if(data.find(v)!=string::npos){
      while(data.find(v)!=string::npos){
	
	start = data.find('=')+1;
	finish = data.find('>',start)-start;
	beamValue = data.substr(start,finish);

	data.erase(data.find(v),8);
	
      }
   
      if(beamValue == "3FF"){
	beamState = 0; // no pressure
      }else{
	beamState = 1; // pressure
      }
    }else{
     
      beamState = 2; //error

    }
 
    // look for blob
    if(data.find(b)!=string::npos){
      while(data.find(b)!=string::npos){
	inp_blob.append(data.substr(data.find(b),data.find(data.find(b),end)));
	data.erase(data.find(b),data.find(data.find(b),end)-data.find(b));
      }
      //printf("%s\n",inp_blob.c_str());
      parseBlob(inp_blob);
    }else{
      blobcount = 0;
    }

    
  }else if(FD_ISSET(global_sk, &readfds)<0){
    printf("\nError: cannot select socket\n");
    
  }
}

//voltage check

string volt_rec(){
  
  // standard command response recv
  string voltSvalue;
  unsigned int start,finish;
 
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 260000;
  
  //char buffer[RFCOMM_DEFAULT_MTU];
  char buffer[256];
  ostringstream sbuf; 
  
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(global_sk, &readfds); 
  
  // is there something to recv
  
  if(FD_ISSET(global_sk, &readfds)){
    
    
    while(select(global_sk+1, &readfds, NULL, NULL, &tv)){
      sbuf.clear();
      memset(buffer,0x0,sizeof(buffer)); 
      
      if(recv(global_sk,buffer,sizeof(buffer),0)<0){
	close(global_sk);
	printf("\nError: cannot rec\n");
	
	
      }else{
	
	sbuf << buffer;
	
      }
      
    }	
    
    voltSvalue = sbuf.str();
    
    start = voltSvalue.find('=')+1;
    finish = voltSvalue.find('>',start)-start;
    voltSvalue = voltSvalue.substr(start,finish);
    
    return voltSvalue;
  }else if(FD_ISSET(global_sk, &readfds)<0){
    printf("\nError: cannot select socket\n");
    string error = "0";
    return error;
  }
}

void voltage(string bluetooth_router, string bluetooth_port)
{
  string voltCommand = "[V7]";
  int voltValue;
  double voltCell;
  string voltSTRINGvalue;
  
  if(send(global_sk,voltCommand.c_str(),voltCommand.length(),0) < 0){
    close(global_sk);
    printf("\nError: cannot send\n");
  } 

  voltSTRINGvalue = volt_rec();
  printf("###############\nVOLTAGE CHECK:\nV7-hex:%s\n",voltSTRINGvalue.c_str());
  voltValue = strtol(voltSTRINGvalue.c_str(), NULL, 16);
  printf("V7:%d\n",voltValue);
  voltCell = voltValue*0.0035;
  printf("volt total:%f\nvolt per cell: %f\n###############\n",voltCell*6,voltCell);

  if(voltCell < 1.0){
    printf("WARNING!!! volatge per cell below 1.0v!!!\n");
  }

 }


void TogglePill(bool set, int rate){
  string beamCommand;
    
  if(set==true){   
    beamCommand = "[V0=";
    stringstream bc;
    bc << rate;// use set rate in config file
    beamCommand.append(bc.str());
    beamCommand.append("]");
    if(send(global_sk,beamCommand.c_str(),beamCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }
  }else if(set==false){
    beamCommand = "[V=0]";
    if(send(global_sk,beamCommand.c_str(),beamCommand.length(),0) < 0){
      close(global_sk);
      printf("\nError: cannot send\n");
    }
  }

}
