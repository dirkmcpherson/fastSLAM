/*
  This is a simple Xwindow that lets the user create boxes
  and arrows.

*/

#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include "ppm.h"
#include "display.h"


/*
  Global Variables determining window and display
*/

Display *curDisplay;
Window   curWindow;
GC       curGC; // graphics context 
int     defdepth;
int     bytesPerPixel;
Pixmap  gblPixmap;


/*
  Global Variables for userland stuff
*/
unsigned char *ximdata;


int initWindow( long width, long height ) {
	int defScreen;
	XSetWindowAttributes wAttr;
	XGCValues gcValues;
	char buffer[64] = "Graphics";
	XTextProperty xtp = {(unsigned char *)buffer, 125, 8, strlen(buffer)};

	/*
	 * connect to the X server.  uses the server specified in the
	 * DISPLAY environment variable
	 */
	curDisplay = XOpenDisplay((char *) NULL);
	if ((Display *) NULL == curDisplay) {
		fprintf(stderr, "xskeleton:  could not open display.\n");
		exit(-1);
	}
   
	/*
	 * begin to create a window
	 */
	printf("creating window\n");
	defdepth = DefaultDepth(curDisplay,0);
	bytesPerPixel = defdepth/8;
	bytesPerPixel = bytesPerPixel == 3 ? 4 : bytesPerPixel;
	printf("default depth %d\n", defdepth);

	defScreen = DefaultScreen(curDisplay);
	printf("Default screen %d\n", defScreen);

	curWindow = XCreateWindow(curDisplay, DefaultRootWindow(curDisplay),
	                          10, 10, width, height, 0, 
	                          defdepth, InputOutput, 
	                          DefaultVisual(curDisplay, defScreen),
	                          0, &wAttr);

	printf("Using depth %d\n", DefaultDepth(curDisplay, 0));

	/*
	 * request mouse button and keypress events
	 */
	wAttr.event_mask = ButtonPressMask | KeyPressMask | ExposureMask;
	XChangeWindowAttributes(curDisplay, curWindow, CWEventMask, &wAttr);

	/*
	 * force it to appear on the screen
	 */
	XSetWMName(curDisplay, curWindow, &xtp);
	XMapWindow(curDisplay, curWindow);

	/*
	 * create a graphics context.  this stores a drawing state; stuff like
	 * current color and line width.  this gc is for drawing into our
	 * window. 
	 */
	printf("creating graphics context\n");
	curGC = XCreateGC(curDisplay, curWindow, 0, &gcValues);

	XSetWindowColormap( curDisplay,
	                    curWindow,
	                    DefaultColormapOfScreen(DefaultScreenOfDisplay(curDisplay)));

	printf("exiting init window\n");
	return(bytesPerPixel);
}

void refreshWindow(int width, int height, PPMImage* map)
{
  PPMPixel temp;
  unsigned long color;
  for( int row = 0 ; row < height ; row++ ){
    for( int col = 0 ; col < width ; col++ ){
      temp = getPixel(col,row,map);
      color = 65536*temp.red + 256*temp.green + temp.blue;
      XSetForeground(curDisplay, curGC, color);
      XDrawPoint( curDisplay, curWindow, curGC, col, row );
    }
  }
  return;
}

void closeWindow(long nFrames)
{
	XCloseDisplay(curDisplay);

	return;
}

void plot( int x, int y ) {
	XDrawPoint( curDisplay, curWindow, curGC, x, y );
}

void arrow( int x0, int y0, int d, float angle ) {
	int xf = x0 + d * cos( angle );
	int yf = y0 - d * sin( angle );

	XDrawLine( curDisplay, curWindow, curGC, x0, y0, xf, yf );

	if( d > 5 ) {
		int xa = x0 + (d-3) * cos( angle - 0.2 );
		int ya = y0 - (d-3) * sin( angle - 0.2 );

		XDrawLine( curDisplay, curWindow, curGC, xf, yf, xa, ya );

		xa =  x0 + (d-3) * cos( angle + 0.2 );
		ya =  y0 - (d-3) * sin( angle + 0.2 );

		XDrawLine( curDisplay, curWindow, curGC, xf, yf, xa, ya );
	}

	return;
}

void box( int x0, int y0, int d ) {
	
	printf(" Drawing box at %d %d of size %d\n", x0, y0, d );

	XDrawLine( curDisplay, curWindow, curGC, x0, y0, x0 + d, y0 );
	XDrawLine( curDisplay, curWindow, curGC, x0 + d, y0, x0 + d, y0 - d );
	XDrawLine( curDisplay, curWindow, curGC, x0 + d, y0 - d, x0, y0 - d);
	XDrawLine( curDisplay, curWindow, curGC, x0, y0 - d, x0, y0 );

	return;
}




