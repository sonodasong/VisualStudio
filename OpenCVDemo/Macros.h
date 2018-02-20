#pragma once

#define ANALOG_CAMERA
#ifdef ANALOG_CAMERA
#define WIDTH						600
#define HEIGHT						480
#else
#define WIDTH						640
#define HEIGHT						480
#endif

#define COS_160						(-0.93969)
#define COS_170						(-0.9848)

#define AREA_RANGE					16
#define AREA_RATIO					0.8

#define BAR_WIDTH					240
#define BAR_HEIGHT					240

#define RED							Scalar(0, 0, 255)
#define GREEN						Scalar(0, 255, 0)
#define BLUE						Scalar(255, 0, 0)
#define PURPLE						Scalar(255, 0, 255)

#define PERFORMANCE
#ifdef PERFORMANCE
#define PERFORMANCE_START()			performanceStart()
#define PERFORMANCE_STOP(tag)		performanceStop(tag)
#else
#define PERFORMANCE_START()
#define PERFORMANCE_STOP(tag)
#endif
