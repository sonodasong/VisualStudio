#pragma once

#define WIDTH						600
#define HEIGHT						480

#define HULL_REDUCE_ANGLE			170
#define AREA_FACTOR					0.8
#define MIN_AREA					5000

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

#undef PERFORMANCE
#ifdef PERFORMANCE
#define PERFORMANCE_START()			performanceStart()
#define PERFORMANCE_STOP(tag)		performanceStop(tag)
#else
#define PERFORMANCE_START()
#define PERFORMANCE_STOP(tag)
#endif
