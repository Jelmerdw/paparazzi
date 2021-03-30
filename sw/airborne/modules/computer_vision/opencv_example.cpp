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
#include <opencv2/core/base.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
using namespace cv;
#include "opencv_image_functions.h"
#include "opencv_example.h"


// THIS FUNCTION DOES THE DILATION ALGORITHM
int opencv_example(char *img, int width, int height)
{
  // Convert image to opencv format:
  Mat M(height, width, CV_8UC2, img);
  Mat image;
  Mat image_center;
  Mat privZeros;

  //Grey out:
  cvtColor(M, image, CV_YUV2GRAY_Y422);

  //Rotate the image:
  rotate(image, image, ROTATE_90_COUNTERCLOCKWISE);


  // Use canny edge detection, only works with greyscale images
  int edgeLowTresh = 50;
  int edgeHighTresh = 100;
  Canny(image, image, edgeLowTresh, edgeHighTresh);

  // ENTER THE WHILE LOOP ....
  // Within each frame, copy the image and dilate the edges until the penultimate step
  //bool ended = false;
  int ITT = 0;
  int x_pos_target = (int) height/2; // What we are looking for.
  while (ITT < 7)
  {
    ITT = ITT + 1;

    //Determine white percentage:
    int amount_white_pixels = (int) cv::sum(image)[0]/255;
    float ratio = (float)amount_white_pixels / (float)(image.rows * image.cols);

    //Create dynamic kernel for dilation:
    int size = (int)(3 + 30 - ratio * 30);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(size, size));

    //Perofrm dilation:
    dilate(image, image, kernel, Point(-1, -1), 1, BORDER_CONSTANT, 255);

  }

  //Define zeros before dilation:
  findNonZero(~image, privZeros);


  //slicing image and checking if  middle columns is full of white pixels
  //if not full of white pixels give straight ahead as heading
  int neighboorhood = 5;
  Rect myROI(x_pos_target - neighboorhood, 0, 2*neighboorhood, image.rows);
  image_center = image(myROI);
  if(cv::sum(image_center)[0] < 255*neighboorhood*2*image.rows)
  {
    return (int)x_pos_target;
  }
  
  //Extract x-values:
  int x_values[privZeros.rows];
  for (int i = 0; i < privZeros.rows; i++)
  {
    x_values[i] = privZeros.at<int>(i, 0);
  }
  
  
  //Sort this list from small to big:
  sort(x_values, x_values + privZeros.rows);

  //Declare variables to find target coordinate.
  //todo: Check whether they should be floats
  int voters = 0;       // How many pixels are in a group

  int prev = x_values[0]; //Use as reference to check if 2 pixels belong to the same group
  float avg = 0;
  int N = 0;

  //Calculate the average of the groups
  for (int i = 0; i < privZeros.rows; i++)
  {
    // If pixels belong to the same group:
    if (x_values[i] - prev <= 1)
    {
      avg = (avg * N + x_values[i]) / (N + 1);
      N += 1;
      prev = x_values[i];
    }
    else
    {
      if (N > voters)
      {
        x_pos_target = avg;
        voters = N;
      }

      avg = x_values[i];
      N = 1;
      prev = x_values[i];
    }
  }

  if (N > voters)
  {
    x_pos_target = avg;
  }

  return (int)x_pos_target;

}
