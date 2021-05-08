/* miabot_params.cc
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
 * miabot_params.cc
 * Parameter file for the Miabot Pro.
 */

#include "miabot_params.h"

// in mm
RobotParams_t miabot_params = 
  {
    0.106, 
    0.075,     
    0.050,     
    0.075,
    8,
    {
      {0.0375,0,0}, //S0
      {-0.0375,0,3.14159265}, //S1
      {0,-0.0375,4.712388975}, //S2
      {0,0.0375,1.570796325}, //S3
      {0.0375,-0.0375,5.497787137}, //S4
      {0.0375,0.0375,0.785398162}, //S5
      {-0.0375,-0.0375,3.926990812}, //S6
      {-0.0375,0.0375,2.356194487},  //S7
    }, 
    {0.075,0,0},
    0.035,
    0.065,
    0.015,
  };

RobotParams_t MiabotRobotParams[PLAYER_NUM_ROBOT_TYPES];

void initialize_miabot_params(void)
{
  MiabotRobotParams[0] =  miabot_params;
 
}
