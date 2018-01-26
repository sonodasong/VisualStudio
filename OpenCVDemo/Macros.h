#pragma once

#define ANALOG_CAMERA
#ifdef ANALOG_CAMERA
#define WIDTH						600
#define HEIGHT						480
#else
#define WIDTH						640
#define HEIGHT						480
#endif

#define HULL_REDUCE_ANGLE			170
#define AREA_FACTOR					0.8
#define MIN_AREA					5000

#define COS_150						(-0.866)
#define COS_160						(-0.93969)
#define COS_170						(-0.9848)
#define AREA_RATIO					0.8

#define AREA_RANGE					16

#define RATIO_MIN					2
#define RATIO_MAX					4
#define UPDATE_FACTOR				1.5

#define BAR_1D_WIDTH				320
#define BAR_1D_HEIGHT				240

#define BAR_2D_WIDTH				320
#define BAR_2D_HEIGHT				320

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
