/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// Demo program showing how libelas can be used, try "./elas -h" for help

#include <iostream>
#include <opencv2/opencv.hpp>
#include "elas.h"

using namespace std;
using namespace cv;

// compute disparities of pgm image input pair file_1, file_2
void process (const char* file_1,const char* file_2) {

  cout << "Processing: " << file_1 << ", " << file_2 << endl;

  // load images
  Mat_<uchar> I1, I2;
  I1 = imread(file_1, CV_LOAD_IMAGE_GRAYSCALE);
  I2 = imread(file_2, CV_LOAD_IMAGE_GRAYSCALE);

  // check for correct size
  if (I1.data == NULL || I2.data == NULL || I1.cols != I2.cols || I1.rows != I2.rows) {
    cout << "ERROR: Images must be of same size, but" << endl;
    cout << "       I1: " << I1.cols <<  " x " << I1.rows << 
                 ", I2: " << I2.cols <<  " x " << I2.rows << endl;
    return;    
  }

  // get image width and height
  int32_t width  = I1.cols;
  int32_t height = I1.rows;

  // allocate memory for disparity images
  const int32_t dims[3] = {width,height,I1.step[0]}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(I1.data,I2.data,D1_data,D2_data,dims);

  // find maximum disparity for scaling output disparity images to [0..255]
  float disp_max = 0;
  for (int32_t i=0; i<width*height; i++) {
    if (D1_data[i]>disp_max) disp_max = D1_data[i];
    if (D2_data[i]>disp_max) disp_max = D2_data[i];
  }

  // copy float to uchar
  Mat_<uchar> D1(height, width);
  Mat_<uchar> D2(height, width);
  for (int32_t y=0; y<height; y++)
  	for (int32_t x=0; x<width; x++) {
	  D1(y,x) = (uint8_t)max(255.0*D1_data[y*width+x]/disp_max,0.0);
	  D2(y,x) = (uint8_t)max(255.0*D2_data[y*width+x]/disp_max,0.0);
	}

  // save disparity images
  char output_1[1024];
  char output_2[1024];
  strncpy(output_1,file_1,strlen(file_1)-4);
  strncpy(output_2,file_2,strlen(file_2)-4);
  output_1[strlen(file_1)-4] = '\0';
  output_2[strlen(file_2)-4] = '\0';
  strcat(output_1,"_disp.pgm");
  strcat(output_2,"_disp.pgm");
  
  imwrite(output_1, D1);
  imwrite(output_2, D2);

  // free memory
  free(D1_data);
  free(D2_data);
}

int main (int argc, char** argv) {

  // run demo
  if (argc==2 && !strcmp(argv[1],"demo")) {
    process("img/cones_left.pgm",   "img/cones_right.pgm");
    process("img/aloe_left.pgm",    "img/aloe_right.pgm");
    process("img/raindeer_left.pgm","img/raindeer_right.pgm");
    process("img/urban1_left.pgm",  "img/urban1_right.pgm");
    process("img/urban2_left.pgm",  "img/urban2_right.pgm");
    process("img/urban3_left.pgm",  "img/urban3_right.pgm");
    process("img/urban4_left.pgm",  "img/urban4_right.pgm");
    cout << "... done!" << endl;

  // compute disparity from input pair
  } else if (argc==3) {
    process(argv[1],argv[2]);
    cout << "... done!" << endl;

  // display help
  } else {
    cout << endl;
    cout << "ELAS demo program usage: " << endl;
    cout << "./elas demo ................ process all test images (image dir)" << endl;
    cout << "./elas left.pgm right.pgm .. process a single stereo pair" << endl;
    cout << "./elas -h .................. shows this help" << endl;
    cout << endl;
    cout << "Note: All images must be pgm greylevel images. All output" << endl;
    cout << "      disparities will be scaled such that disp_max = 255." << endl;
    cout << endl;
  }

  return 0;
}


