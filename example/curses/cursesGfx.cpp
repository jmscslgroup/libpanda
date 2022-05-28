#include "curses-gfx.h"

#include <cmath>

#include <ncurses.h>

void drawDotFloat(double x, double y) {
	char ch = '`';
	
	double remainder = fmod(y, 1);
	
	// ,.-*'` <- fonts differ in terminal
	
	// 28 pixel charaxter height
	// , -> 23
	// . -> 21  -> separation for above = (23-21)/2/28
	// - -> 16
	// * -> 11
	// ' -> 9
	// ` -> 8
	if (remainder > ((23.0+21.0)/(2.0*28.0))) {
		ch = ',';
	} else if (remainder > ((21.0+16.0)/(2.0*28.0))) {
		ch = '.';
	} else if (remainder > ((16.0+11.0)/(2.0*28.0))) {
		ch = '-';
	} else if(remainder >  ((11.0+9.0)/(2.0*28.0))) {
		ch = '*';
	} else if(remainder >  ((9.0+8.0)/(2.0*28.0))) {
		ch = '\'';
	}
	
	mvaddch( y, x, ch);
}




/*
 set() and ln() are from the ASCII Tesseract Rotation project, tweaked for ncurses.
 ln2() and getp() are more heavily tweaked versions to support more characters
 https://gist.github.com/Mashpoe/3d949824be514c43b58706eb29c33c43
 */
void set( Coordinates2D pt, char c)
{
	mvaddch( pt.y, pt.x, c);
}

char getp( Coordinates2D* pts, double err)
{
	if (abs(pts[0].y - pts[2].y) < 2)	// checks the slope
	{
		// 28 pixel charaxter height
		// _ -> 23
		// - -> 16 -> separation for above = (23-16)/2/28
		// " -> 9
		// ` -> 8

		if (err > 1.0-((9.0+8.0)/(2.0*28.0))) { //  _-"`
			return '`';
		}
		if (err > 1.0-((16.0+9.0)/(2.0*28.0))) { //  _-"`
			return '"';
		}
		if (err > 1.0-((23.0+16.0)/(2.0*28.0)))
		{
			return '-';
		}
		return '_';
	}
	
	if (abs(pts[0].x - pts[2].x) < 2 &&
		(pts[0].x >= pts[2].x || pts[1].x != pts[2].x) &&
		(pts[0].x <= pts[2].x || pts[1].x != pts[0].x))
	{
		return '|';
	}

	int mX = pts[0].y < pts[2].y ? pts[0].x : pts[2].x;
	return mX < pts[1].x ? '\\' : '/';
}

// This is basically https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
// The silly thing here is that for determining individual characters,
//  floating operations are done which circumvents the integer-only intention
//  in the original algorithm
void ln2( Coordinates2D a, Coordinates2D b) {
	set( a, 'O');
	set( b, 'O');
	
	int dx = abs(b.x - a.x);
	int sx = a.x < b.x ? 1 : -1;
	int dy = -abs(b.y - a.y);
	int sy = a.y < b.y ? 1 : -1;
	int err = dx + dy;
	int e2;
	
	int errs[3];
	double errsNormalized[3];
	Coordinates2D pts[3];
	
	// Builds a starting set of points purely needed for getp()
	for (int i = 0; i < 3; ++i)
	{
		pts[i] = a;
		errs[i] = err;
		if (i > 0) {
			errsNormalized[i] = (double)(errs[i-1])/(dx - dy);
			if (sy == -1 ) {
				errsNormalized[i] = 0.0 - errsNormalized[i];
			}
		}
		
		if (a.x == b.x && a.y == b.y) {
			break;
		}

		e2 = 2*err;
		if (e2 >= dy) {
			err += dy;
			a.x += sx;
		} else {
			errs[i] -= dy;
		}
		if (e2 <= dx) {
			err += dx;
			a.y += sy;
		} else {
			errs[i] -= dx;
		}
	}
	
	for (;;)
	{
		set( pts[1], getp( pts, errsNormalized[1] + 0.5));

		pts[0] = pts[1];
		pts[1] = pts[2];
		pts[2] = a;
		
		errs[0] = errs[1];
		errs[1] = errs[2];
		errs[2] = err;
		
		errsNormalized[0] = errsNormalized[1];
		errsNormalized[1] = errsNormalized[2];
		errsNormalized[2] = (double)(errs[1])/(dx - dy);
		if (sy == -1 ) {
			errsNormalized[2] = 0.0 - errsNormalized[2];
		}
		
		if (a.x == b.x && a.y == b.y) {
			break;
		}

		e2 = 2*err;
		if (e2 >= dy) {
			err += dy;
			a.x += sx;
		} else {
			errs[2] -= dy;
		}
		if (e2 <= dx) {
			err += dx;
			a.y += sy;
		} else {
			errs[2] -= dx;
		}
	}
	
	// add the final point
	set( pts[1], getp( pts, errsNormalized[1] +0.5));

}

void ln( Coordinates2D a, Coordinates2D b)
{
	set( a, 'O');
	set( b, 'O');

	int dx = abs(b.x - a.x);
	int sx = a.x < b.x ? 1 : -1;
	int dy = abs(b.y - a.y);
	int sy = a.y < b.y ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2;
	int e2;

	Coordinates2D pts[3];
	double ers[3];	// ers[0] set, never used

	for (int i = 0; i < 3; ++i)
	{
		pts[i] = a;
		ers[i] = ((double)err - dx) / ((double)dy - dx);
		ers[i] = sy == 1 ? 1.0f - ers[i] : ers[i];

		if (a.x == b.x && a.y == b.y) {
			return;
		}

		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			a.x += sx;
		}
		if (e2 < dy) {
			err += dx;
			a.y += sy;
		}
	}

	for (;;)
	{
		set( pts[1], getp( pts, ers[1]));

		pts[0] = pts[1];
		pts[1] = pts[2];
		pts[2] = a;

		ers[0] = ers[1];	// never used
		ers[1] = ers[2];
		ers[2] = ((double)err - dx) / ((double)dy - dx);
		ers[2] = sy == 1 ? 1.0f - ers[2] : ers[2];
		
		if (a.x == b.x && a.y == b.y) {
			break;
		}

		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			a.x += sx;
			
		}
		if (e2 < dy) {
			err += dx;
			a.y += sy;
		}
	}

	// add the final point
	set( pts[1], getp( pts, ers[1]));
}
