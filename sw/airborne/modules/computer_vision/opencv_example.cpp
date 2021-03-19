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


int opencv_example(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;

  //  Grayscale image example
  //cvtColor(M, image, CV_YUV2GRAY_Y422);
  cvtColor(M, image, CV_YUV2GRAY_Y422);
  rotate(image, image, ROTATE_90_COUNTERCLOCKWISE);

  

  // CANNY EDGES, ONLY WORKS WITH GRAYSCALE IMAGE
  int edgeLowTresh = 50;
  int edgeHighTresh = 100;
  Canny(image, image, edgeLowTresh, edgeHighTresh);
  

  
  //  MORPHOLOGY OPERATIONS
  //Element is kernel to be used for morphology operations (in this case opening)
  Mat element = getStructuringElement(MORPH_RECT, Size(1, 4));
  morphologyEx(image, image, MORPH_OPEN, element);
  

  //  HOUGH TRANSOFM
  vector<Vec4i> lines; //vector that holds results of HoughDetection

  
  //Define variables for Hough transform
  int rho = 1; // distance resolution in pixels of the Hough grid
  //theta  angular resolution in radians of the Hough grid is equal to PI and gets immediatly passed to function trough CV_PI
  int threshold = 15;  // minimum number of votes (intersections in Hough grid cell)
  int min_line_length = 2;  // minimum number of pixels making up a line
  int max_line_gap = 20;  // maximum gap in pixels between connectable line segments
  
    
  //Execute hough transform
  HoughLinesP(image, lines, rho, CV_PI, threshold, min_line_length, max_line_gap);
  
  Point2f v[lines.size()]; //Initialize empty array with opencv point classes to insert line centroid coordinates


  //Create image with line and calculates center of line 
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    
    v[i] = Point2f((l[0]+l[2])/2, (l[1]+l[3])/2);
  }
  sort(v,v+lines.size(),comp);
  
  //Group points using tresholds. This is pretty messy
  bool button = false;
  float x_center = -1;
  float y_center = -1;
  //This array is allocated with more memory then necessary as amount of centers =< lines.size() just to be sure and avoid overflow. However after this calculation it should be possible to truncate the array to free some memory.
  Point2f centers[lines.size()];
  int centers_idx = 0;
  int N = 0;
  for(size_t j = 0; j < lines.size(); j++)
  {
    if (button == false || v[j].x-x_center < 50){
      x_center = (x_center*N+v[j].x)/(N+1);
      y_center = (y_center*N+v[j].y)/(N+1);
      N=N+1;
      button = true;
    }
    else {
      centers[centers_idx] = Point2f(x_center, y_center);
      x_center = -1;
      y_center = -1;
      N=0;
      button = false;
      centers_idx = centers_idx+1;
    }
  }
  if (button == false){
    centers[centers_idx] = Point2f(x_center, y_center);
  }
  //I have not found a way yet to truncate centers array to the amount of poitns
  //centers.resize(centers_idx);
  
    
  return centers[0].x;
}
