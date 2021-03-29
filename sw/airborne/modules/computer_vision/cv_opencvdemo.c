/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

//Clock:
clock_t start, end;
double cpu_time_used;

#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCVDEMO_FPS)

// Function
float x_cor;

static pthread_mutex_t mutex;

//Structure that will be communicated using abi
struct coordinate_message{
  int16_t x_c;
  int16_t y_c;
  bool updated;
};

static struct coordinate_message global_coordinate_message[1];




struct image_t *opencv_func(struct image_t *img);
struct image_t *opencv_func(struct image_t *img)
{

  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    x_cor = opencv_example((char *) img->buf, img->w, img->h);
    
    //Update global coordinates
    pthread_mutex_lock(&mutex);
    global_coordinate_message[0].x_c = x_cor;
    global_coordinate_message[0].y_c = 2; //Y value needs to be passed as well from the function later on
    global_coordinate_message[0].updated = true;
    pthread_mutex_unlock(&mutex);
    
  }

// opencv_example(NULL, 10,10);

  return NULL;
}

void opencvdemo_init(void)
{
  //Initialise global coordinate struct message
  memset(global_coordinate_message, 0, sizeof(struct coordinate_message));
  pthread_mutex_init(&mutex, NULL);
	
  //Use camera to pass iamge to opencv_func
  cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func, OPENCVDEMO_FPS);
}

void opencvdemo_periodic(void)
{
  static struct coordinate_message local_coordinate_message[1];
  //Copy global variable 'global_coordinate_message' that is used in the cv_add_to_device thread to a local variable in order to send the variable with the ABI msg
  pthread_mutex_lock(&mutex);
  memcpy(local_coordinate_message, global_coordinate_message, sizeof(struct coordinate_message));
  pthread_mutex_unlock(&mutex);

  //Update ABI MSG
  if(local_coordinate_message[0].updated){

    AbiSendMsgTARGET_COORDINATE_TEAM_8(TARGET_COORDINATE_TEAM_8_ID, local_coordinate_message[0].x_c, local_coordinate_message[0].y_c);

    local_coordinate_message[0].updated = false;

    //Clock:
    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("Time taken %lf seconds\n", cpu_time_used);
    start = clock();
  }

}

