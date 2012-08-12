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
	// Constructor
    PPMImage() :
        x(0), y(0), data(NULL)
    {
    }

	// Deconstructor
    ~PPMImage()
    {
        if( data != NULL )
            free(data);
    }

    // Convert coordinates to a id number
    unsigned int getID( unsigned int x_coord, unsigned int y_coord )
    {
        return y_coord * x + x_coord;
    }

    // Convert id to x
    unsigned int getX( unsigned int id )
    {
        return id % x;
    }

    // Convert id to y
    unsigned int getY( unsigned int id )
    {
        return id / x;
    }

    unsigned int getSize()
    {
        return x * y;
    }

    // Member variables
    unsigned int x, y;
    PPMPixel *data;
};


#define CREATOR "RPFELGUEIRAS"
#define RGB_COMPONENT_COLOR 255

// *********************************************************************************************************************
// Read Function
// *********************************************************************************************************************
PPMImage *readPPM(const char *filename);

#endif
