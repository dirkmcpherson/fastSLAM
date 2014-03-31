#ifndef display_h
#define display_h
#include <X11/Xlib.h>
#include <X11/Xutil.h>

int initWindow(long,long);
void refreshWindow(int,int,PPMImage*);
void closeWindow(long);
void plot(int,int);
void arrow(int, int, int, float);
void box( int,int,int);
int main_t(void);

#endif
