#ifndef CURSES_CLOCK_H
#define CURSES_CLOCK_H

#include <sys/time.h>

#include "curses-gfx.h"



void drawclock(double width, double height, Coordinates2D center, bool miniticks, struct timeval time);

void drawSecondsDial(double width, double height, Coordinates2D center, bool miniticks, struct timeval time);

#endif
