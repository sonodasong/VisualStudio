#pragma once

#define WIDTH						600
#define HEIGHT						480

#define MIN_BAR_WDITH				4
#define MAX_BAR_WIDTH				10

#define HULL_REDUCE_ANGLE			170
#define AREA_FACTOR					0.8
#define MIN_AREA					5000

#define RATIO_MIN					2
#define RATIO_MAX					4
#define UPDATE_FACTOR				1.5

#define BAR_1D_WIDTH				320
#define BAR_1D_HEIGHT				240

#undef PERFORMANCE
#ifdef PERFORMANCE
#define PERFORMANCE_START()			performanceStart()
#define PERFORMANCE_STOP(tag)		performanceStop(tag)
#else
#define PERFORMANCE_START()
#define PERFORMANCE_STOP(tag)
#endif
