#pragma once

#define WIDTH						320
#define HEIGHT						240

#define MIN_BAR_WDITH				4
#define MAX_BAR_WIDTH				10

#define HULL_REDUCE_ANGLE			170

#define BAR_1D_WIDTH				320
#define BAR_1D_HEIGHT				240

#define PERFORMANCE
#ifdef PERFORMANCE
#define PERFORMANCE_START()			performanceStart()
#define PERFORMANCE_STOP(tag)		performanceStop(tag)
#else
#define PERFORMANCE_START()
#define PERFORMANCE_STOP(tag)
#endif
