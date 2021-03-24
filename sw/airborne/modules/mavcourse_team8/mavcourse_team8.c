/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/mavcourse_team8/mavcourse_team8.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

//Includes voor open_cv:
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"

// #define MAVCOURSE_TEAM8_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[MAVcourse team 8->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if MAVCOURSE_TEAM8_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

//Define FPS:
#ifndef FPS
#define FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(FPS)

// Initiate setting variables SET INITIAL VALUES HERE!!!!!
float heading_gain = 4.5f; //Old values: 1.17f
float speed_gain = 1.35f; //Old values: 1.91f
float acceptance_width_percent = 0.2; //Old values: 20
//int x_clear = 0;
float heading_increment = 10.f; //Old values: 30.f
float maxDistance = 2.f; //Old values: 2.f

//FILE for debugging:
FILE *fptr;

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  printf("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

// // Setting possible states
enum navigation_state_t {
	FIND_NEW_HEADING,
 	FOLLOWING,
	OUT_OF_BOUNDS
};

enum navigation_state_t navigation_state = FIND_NEW_HEADING;

float speed_setting = 0.f;
float heading_step = 0.f;
float heading_rate = 0.f;
int x_clear = 0;
uint16_t x_max = 520;
int32_t floor_count = 0;
int32_t floor_centroid = 0;
float avoidance_heading_direction = 1.f;
int acceptance_width;

// Define event for ABI messaging
static abi_event direction_ev;
// Callback function for ABI messaging
static void direction_cb(
						uint8_t __attribute__((unused)) sender_id,
						int16_t x_coord,
						int16_t __attribute__((unused)) y_coord)
{
	x_clear = x_coord;
	printf("X clear (direct ABI): %i \n",x_clear);
 }


struct image_t *get_image(struct image_t *img);
struct image_t *get_image(struct image_t *img)
{
  auto time = img->pprz_ts;
	// Call OpenCV (C++ from paparazzi C function)
	opencv_example((char *) img->buf, img->w, img->h);
  return img; // Return modified image for further processing
};

/*
 * Initialisation function
 */
void mavcourse_team8_init(void)
{

	cv_add_to_device(&CAMERA, get_image, FPS); //CAMERA defined in mavcourse_team8_airframe.xml

	// Bind vertical edge detection callback (x_clear is the x coordinate of the clear direction-> the dot)
	AbiBindMsgTARGET_COORDINATE_TEAM_8(VERTICAL_EDGE_DETECTION_ID, &direction_ev, direction_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void mavcourse_team8_periodic(void)
{
	//Save target to use for debugging:
	fptr = fopen("data.txt","w");
	fprintf(fptr,"%d", x_clear);
	fclose(fptr);

	// only evaluate our state machine if we are flying
	if(!autopilot_in_flight()){
		return;
	}

	// In case of overflow error (observed) in x, detect and set to middle value of 120.
	if(x_clear > x_max || x_clear < -1){
		x_clear = 240;
	}
	acceptance_width = acceptance_width_percent * x_max;

	int x_fromcenter = x_clear - x_max/2;
	printf("X clear (after if): %i \n", x_clear);
	printf("X from center: %i \n",x_fromcenter);
	printf("State: %i \n", navigation_state);

	switch (navigation_state){
		case FIND_NEW_HEADING:

			// Proportional relation to heading step and centeredness of dot (yawing towards dot)
			// heading_step = ((float)x_clear-(float)x_max/2) * heading_gain_idle;
			// printf("Heading step: %f \n", heading_step);
			increase_nav_heading(heading_increment);

			moveWaypointForward(WP_TRAJECTORY, 1.5f);

			// Only move to the following state when the dot is in the acceptance width AND the trajectory doesn't take drone out of the cyberzoo
			if(abs(x_clear-x_max/2) < acceptance_width && InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
				navigation_state = FOLLOWING;
			}

			break;

		case FOLLOWING:
			// Proportional relation to heading rate and centeredness of dot
			heading_step = ((float)x_clear-(float)x_max/2) * heading_gain / 10.f;
			heading_step = fminf(heading_step,25.f);

			printf("Heading step: %f \n", heading_step);

			//Inverse relation to speed and centeredness of dot
			speed_setting = fmaxf((1 - abs((float)x_clear-(float)x_max/2)/(acceptance_width/2)),0) * speed_gain;
			printf("Speed setting: %f \n", speed_setting);

			// first increase nav heading, then move waypoint forward
			float moveDistance = fminf(maxDistance, speed_setting);
			increase_nav_heading(heading_step); // first increase nav heading, then move waypoint forward

		    moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

		    // If dot outside acceptance width OR waypoint outside cyberzoo, move back to finding a new heading
		    if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
		    	navigation_state = OUT_OF_BOUNDS;
		    }

		    else if (abs(x_clear-x_max/2) >= acceptance_width){
				navigation_state = FIND_NEW_HEADING;
			}
			// Else move forward with speed (distance) proportional to 'confidence'
			else {
				printf("Move distance: %f m \n",moveDistance);
			    moveWaypointForward(WP_GOAL, moveDistance);
			}

			break;

		case OUT_OF_BOUNDS:
		    increase_nav_heading(heading_increment);
		    moveWaypointForward(WP_TRAJECTORY, 1.5f);

		    if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
		        // add offset to head back into arena
		        navigation_state = FIND_NEW_HEADING;
		      }

	 		break;
	 	/*
		case STUCK:
			moveWaypointForward(WP_GOAL,-1.f);
			increase_nav_heading(heading_increment); */


		default:
			navigation_state = FIND_NEW_HEADING;
			break;
 	      }
	return;
}
