/*  miabot_params.h
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
#ifndef _MIABOT_PARAMS_H
#define _MIABOT_PARAMS_H

void initialize_miabot_params(void);

#define PLAYER_NUM_ROBOT_TYPES 1

typedef struct
{
  double x;
  double y;
  double th;
} sonar_pose_t;

typedef struct
{
 
  double RobotDiagonal; // 
  double RobotLength; //
  double RobotRadius; // 
  double RobotWidth; // 
  int SonarNum; // 
  sonar_pose_t sonar_pose[8]; //
  double gripper_pose[3]; //
  double gripperw;//
  double gripperl;//
  double gripperh;//
} RobotParams_t;

extern RobotParams_t MiabotRobotParams[];

#endif
