/*
 * PPM Image Format Reader
 * Code originally from http://stackoverflow.com/questions/2693631/read-ppm-file-and-store-it-in-an-array-coded-with-c
 */

#ifndef OMPL_RVIZ_VIEWER_PPM_
#define OMPL_RVIZ_VIEWER_PPM_

#include <stdio.h>
#include <stdlib.h>

// *********************************************************************************************************************
// PPMPixel Struct
// *********************************************************************************************************************
typedef struct {
  unsigned char red,green,blue;
} PPMPixel;

// *********************************************************************************************************************
// PPMImage Struct
// *********************************************************************************************************************
typedef struct {
  int x, y;
  PPMPixel *data;
} PPMImage;


#define CREATOR "RPFELGUEIRAS"
#define RGB_COMPONENT_COLOR 255

// *********************************************************************************************************************
// Read Function
// *********************************************************************************************************************
PPMImage *readPPM(const char *filename);

#endif
