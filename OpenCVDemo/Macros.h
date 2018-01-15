#pragma once

#define WIDTH						320
#define HEIGHT						240

#define MIN_BAR_WDITH				4
#define MAX_BAR_WIDTH				10

#undef PERFORMANCE
#ifdef PERFORMANCE
#define PERFORMANCE_START()			performanceStart()
#define PERFORMANCE_STOP(tag)		performanceStop(tag)
#else
#define PERFORMANCE_START()
#define PERFORMANCE_STOP(tag)
#endif
