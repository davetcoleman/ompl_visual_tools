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
/*typedef struct {
  int x, y;
  PPMPixel *data;
} PPMImage;
*/
class PPMImage
{
public:
  PPMImage() {}

  // Convert coordinates to a id number
  int getID( int x_coord, int y_coord )
  {
    return y_coord * x + x_coord;
  }

  // Convert id to x
  int getX( int id )
  {
    return id % x;
  }

  // Convert id to y
  int getY( int id )
  {
    return id / x;
  }

  int getSize()
  {
    return x * y;
  }

  // Member variables
  int x, y;
  PPMPixel *data;
};


#define CREATOR "RPFELGUEIRAS"
#define RGB_COMPONENT_COLOR 255

// *********************************************************************************************************************
// Read Function
// *********************************************************************************************************************
PPMImage *readPPM(const char *filename);

#endif
