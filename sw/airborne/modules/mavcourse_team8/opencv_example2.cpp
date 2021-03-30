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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
using namespace cv;
#include "opencv_image_functions.h"
#include "opencv_example.h"

//Comparison function to supply to sorting function later on in order to sort based on x coordinate first and subsequently y coordinates (imo the sorting of y coordinates might not really be necessary and it might be nice to drop that in order to speed up the sorting process).
struct str{
    bool operator() ( Point2f a, Point2f b ){
        if ( a.x != b.x ) 
            return a.x < b.x;
        return a.y <= b.y ;
    }
} comp;

// THIS FUNCTION DOES THE DILUTION ALGORITHM
int opencv_example(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;
  Mat inverted;
  Mat copy;
  Mat dilation;

  // Grey out and rotate the images
  cvtColor(M, image, CV_YUV2GRAY_Y422);
  rotate(image, image, ROTATE_90_COUNTERCLOCKWISE);

  // Initialise the kernel
  int kernel[5][5] = {
      {1, 1, 1, 1, 1} ,   /*  initializers for row indexed by 0 */
      {1, 1, 1, 1, 1} ,   /*  initializers for row indexed by 1 */
      {1, 1, 1, 1, 1} ,   /*  initializers for row indexed by 2 */
      {1, 1, 1, 1, 1} ,   /*  initializers for row indexed by 3 */
      {1, 1, 1, 1, 1}     /*  initializers for row indexed by 4 */
  };

  // Increase contrast
  addWeighted(image, 1, image, 0, 0)

  // Use canny edge detection, only works with greyscale images
  int edgeLowTresh = 50;
  int edgeHighTresh = 100;
  Canny(image, image, edgeLowTresh, edgeHighTresh);

  //Include edges on the left side of the screen and make them white
  for (int i=0; i < height; i++){
    image[i][0] = 255};

  //Include edges of the right side of the screen and make them white
  for (int i=0; i < height; i++){
    image[i][width] = 255};

  // Make a copy of img_edges
  dilation = image;

  // ENTER THE WHILE LOOP ....
  // Within each frame, copy the image and dilute the edges until the penultimate step
  bool ended = false;
  while (not ended) 
  {
    copy = dilation;
    dilate(dilation, kernel, iterations = 1);

    //INVERT black and white and FIND LOCATION of black pixels and SAVE it to ZeroCoordinates
    Mat inverted = ~copy;
    Mat ZeroCoordinates;
    findNonZero(inverted, ZeroCoordinates);

    //Find how many x coordinates we found
    int rows =  sizeof (ZeroCoordinates) / sizeof (ZeroCoordinates[0]);

    //If this length is zero, stop the loop and take penultimate version
    if(rows == 0)
      {
        dilation = copy;
        ended = true;
      }
    }

  //Now that we retrieved the penultimate image, perform FindZero once more:
  Mat inverted = ~dilation;
  Mat ZeroCoordinates;
  findNonZero(inverted, ZeroCoordinates);

  //Sort this list from small to big:
  //todo: Sort functie erin zetten (en accordingly r.128 aanpassen)

  //Declare variables to find target coordinate.
  // todo: Check whether they should be floats
  float int x_pos_target = 0;       // What we are looking for.
  float voters = 0;                 // How many pixels are in a group

  float prev = ZeroCoordinates[0];  //Gebruik als reference om te checken of twee pixels bij dezelfde gr horen
  float avg = 0;
  float N = 0;

  //Calculate the average of the groups
  for (x in x_values) {
    // If pixels belong to the same group:
    if (x - prev <= 1) {
      avg = (avg * N + x) / (N + 1)
      N += 1
      prev = x
    }
    else{
      if (N > voters) {
        x_pos_target; = avg
        voters = N
      }

      avg = x
      N = 1
      prev = x
    }

  if (N > voters) {
    x_pos_target = avg
  }
  }
};
