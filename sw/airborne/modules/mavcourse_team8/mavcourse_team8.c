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
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

// Setting possible states
enum navigation_state_t {
  SAFE,
  ADJUST,
  FIND_NEW_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};
uint16_t x_clear = 0;
uint16_t x_max = 100;

// Define event for ABI messaging
static abi_event direction_ev;
// Callback function for ABI messaging
static void direction_cb(uint16_t x_coord){
	x_clear = x_coord;
}


/*
 * Initialisation function
 */
void mavcourse_team8_init(void)
{
	// Bind vertical edge detection callback (x_clear is the x coordinate of the clear direction-> the dot)
	AbiBindMsgVERTICAL_EDGE_DETECTION(VERTICAL_EDGE_DETECTION_ID, &direction_ev, direction_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void mavcourse_team8_periodic(void)
{

}
