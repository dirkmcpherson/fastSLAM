#ifndef ppm_h
#define ppm_h
#define ABS(a) ((a<0)? -a : a)
#define RGB_COMPONENT_COLOR 255

typedef struct {
     unsigned char red,green,blue;
} PPMPixel;

typedef struct {
     int x, y;
     PPMPixel *data;
} PPMImage;

PPMImage *readPPM( const char* );
void writePPM(const char*, PPMImage*);
void changePixelColor(PPMImage*, int,int,int,int,int);
void changeColorPPM(PPMImage *);
void addRedPixel(int,int,PPMImage*);
void addRedDot(int, int, PPMImage*);
PPMPixel getPixel(int,int,PPMImage*);
short isGray(int,int,PPMImage*);
double rayTrace(int,int,float,PPMImage*);

#endif
