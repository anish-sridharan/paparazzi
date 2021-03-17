/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "mav_exercise.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot_static.h"
#include <stdio.h>
#include <time.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

/*
# ifndef h_i
# define h_i 20.f
# endif

# ifndef d_i
# define d_i 0.3f
# endif
*/
//uint8_t increase_nav_heading(float incrementDegrees);
//uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);

//uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  HOLD
};

// define and initialise global variables
float oa_color_count_frac = 0.18f;
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
//float moveDistance = 2;                 // waypoint displacement [m]
float oob_haeding_increment = 5.f;      // heading angle increment if out of bounds [deg]
float avoidance_heading_direction = 1.f;  // heading change direction for avoidance [rad/s]

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free
float heading_increment = 20.f;          // heading angle increment [deg]
float div_1 = 0.f;
float div_thresh = 0.0001f;
float oag_max_speed = 5.f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]


// needed to receive output from a separate module running on a parallel process

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#ifndef FLOW_OPTICFLOW_ID
# define FLOW_OPTICFLOW_ID ABI_BROADCAST
# endif

#ifndef FLOOR_VISUAL_DETECTION_ID
# define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
# endif

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width,
                               int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra) {
  color_count = quality;
}


static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id,
                               uint32_t __attribute__((unused)) now_ts, int16_t __attribute__((unused)) flow_x,int16_t __attribute__((unused)) flow_y,
                               int16_t __attribute__((unused)) flow_der_x,
                               int16_t __attribute__((unused)) flow_der_y,
                               float __attribute__((unused)) noise_measurement, float div_size) {
  div_1 = div_size;
}


void mav_exercise_init(void) {
  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &optical_flow_ev, optical_flow_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

void mav_exercise_periodic(void) {
  // only evaluate our state machine if we are flying
  
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return; 
  }
  /*
  if (!autopilot_in_flight()) {
    return;
  }
  */

  // compute current color thresholds
  // front_camera defined in airframe xml, with the video_capture module
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  PRINT("Divergence: %f threshold: %f state: %d \n", div_1, div_thresh, navigation_state);

  
  // update our safe confidence using color threshold
  if (color_count < color_count_threshold) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
 

  switch (navigation_state) {
    case SAFE:
	//setGuided();
      
     if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state = OUT_OF_BOUNDS;
      } 
      else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      }
       else {
        guidance_h_set_guided_body_vel(1, 0);
      }
      /*

      //moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
     
     if (!InsideObstacleZone(GetPosX(), GetPosY() )) {
	//setGuided();
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0) {
        navigation_state = OBSTACLE_FOUND;
      } else {
        //moveWaypointForward(WP_GOAL, moveDistance);
       
        guidance_h_set_guided_body_vel(1, 0);
      }
     */
     
      break;
    case OBSTACLE_FOUND:
      // TODO Change behavior
     
      
      guidance_h_set_guided_body_vel(0, 0);
      // stop as soon as obstacle is found
      /*
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);
      */
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;
    case SEARCH_FOR_SAFE_HEADING:
      //increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe
      
      //guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi + RadOfDeg(30.f));

      guidance_h_set_guided_heading_rate(oag_heading_rate);
     

      // make sure we have a couple of good readings before declaring the way safe
      
      if (obstacle_free_confidence >= 2){
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE; 
       }
      
      /*
      if (obstacle_free_confidence >= 2){
        navigation_state = SAFE;
      }
      */
      break;
    case OUT_OF_BOUNDS:
      // stop
	//setGuided();
     guidance_h_set_guided_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(15));

      navigation_state = REENTER_ARENA;


      break;
    case REENTER_ARENA:

      //guidance_h_set_guided_body_vel(0, 0);	
	    // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SAFE;

        /*
        float angle;
	
        //guidance_h_set_guided_heading(angle);
	PRINT("HELLO");	
        //guidance_h_set_guided_body_vel(0, 0);
        //guidance_h_set_guided_heading_rate(1 * RadOfDeg(15));

	guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi+RadOfDeg(180.f));
	//guidance_h_set_guided_body_vel(1, 0);
	navigation_state = SAFE;
        */
        }
	break;
    case HOLD:
    default:
      break;

  }
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
/*
uint8_t increase_nav_heading(float incrementDegrees) {
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  return false;
}
*/

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */

/*
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters) {
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}
*/

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
/*
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}
*/
/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
/*
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}
*/
