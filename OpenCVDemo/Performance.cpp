#include "stdafx.h"

#include "Performance.h"

static int64 start = 0;

void performanceStart(void)
{
	start = getTickCount();
}

void performanceStop(String tag)
{
	int64 stop = getTickCount();
	cout << tag << " performance: " << (stop - start) * 1000 / getTickFrequency() << "ms" << endl;
}
