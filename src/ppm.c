#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include "ppm.h"

PPMImage *readPPM(const char *filename)
{
  char buff[16];
  PPMImage *img;
  FILE *fp;
  int c, rgb_comp_color;
  //open PPM file for reading
  fp = fopen(filename, "rb");
  if (!fp) {
    fprintf(stderr, "Unable to open file '%s'\n", filename);
    exit(1);
         }
  
  //read image format
  if (!fgets(buff, sizeof(buff), fp)) {
              perror(filename);
              exit(1);
  }
  
    //check the image format
  if (buff[0] != 'P' || buff[1] != '6') {
         fprintf(stderr, "Invalid image format (must be 'P6')\n");
         exit(1);
  }
  
    //alloc memory form image
  img = (PPMImage *)malloc(sizeof(PPMImage));
  if (!img) {
    fprintf(stderr, "Unable to allocate memory\n");
         exit(1);
  }
  
  //check for comments
  c = getc(fp);
  while (c == '#') {
    while (getc(fp) != '\n') ;
    c = getc(fp);
  }
  
  ungetc(c, fp);
  //read image size information
  if (fscanf(fp, "%d %d", &img->x, &img->y) != 2) {
    fprintf(stderr, "Invalid image size (error loading '%s')\n", filename);
    exit(1);
  }
  
  //read rgb component
  if (fscanf(fp, "%d", &rgb_comp_color) != 1) {
    fprintf(stderr, "Invalid rgb component (error loading '%s')\n", filename);
    exit(1);
  }
  
  //check rgb component depth
  if (rgb_comp_color!= RGB_COMPONENT_COLOR) {
    fprintf(stderr, "'%s' does not have 8-bits components\n", filename);
    exit(1);
  }
  
  while (fgetc(fp) != '\n') ;
  //memory allocation for pixel data
  img->data = (PPMPixel*)malloc(img->x * img->y * sizeof(PPMPixel));
  
  if (!img) {
    fprintf(stderr, "Unable to allocate memory\n");
    exit(1);
  }
  
  //read pixel data from file
  if (fread(img->data, 3 * img->x, img->y, fp) != img->y) {
    fprintf(stderr, "Error loading image '%s'\n", filename);
    exit(1);
  }
  
  fclose(fp);

  printf( "image x,y : %d,%d\n", img->x,img->y);
  return img;
}
void writePPM(const char *filename, PPMImage *img)
{
  FILE *fp;
  //open file for output
  fp = fopen(filename, "wb");
  if (!fp) {
    fprintf(stderr, "Unable to open file '%s'\n", filename);
    exit(1);
  }
  
  //write the header file
  //image format
  fprintf(fp, "P6\n");
  
  //comments
  fprintf(fp, "#MAP\n");
  
  //image size
  fprintf(fp, "%d %d\n",img->x,img->y);
  
  // rgb component depth
  fprintf(fp, "%d\n",RGB_COMPONENT_COLOR);
  
  // pixel data
  fwrite(img->data, 3 * img->x, img->y, fp);
  fclose(fp);
}

void changePixelColor( PPMImage* img, int x, int y, int r, int g, int b ){ 
  int i;
  if(img){
    i = img->x*y + x;
    img->data[i].red = r;
    img->data[i].blue = b;
    img->data[i].green = g;
  }
  else{
    printf("Faulty Image Pointer\n");
  }
}

void changeColorPPM(PPMImage *img)
{
  int i;
  if(img){
    
    for(i=0;i<img->x*img->y;i++){
      img->data[i].red=RGB_COMPONENT_COLOR-img->data[i].red;
      img->data[i].green=RGB_COMPONENT_COLOR-img->data[i].green;
      img->data[i].blue=RGB_COMPONENT_COLOR-img->data[i].blue;
    }
  }
}

void addRedPixel(int x, int y, PPMImage *img)
{
  int i= img->x*y + x;
  if(img){
    img->data[i].red = RGB_COMPONENT_COLOR;
    img->data[i].green = 0;
    img->data[i].blue = 0;
  }
}



void addRedDot(int x, int y, PPMImage *img)
{
  int i;
  int j;
  for(i=x-1;i<img->x && i<x+2;i++){
    for(j=y-1;j<img->y && j<y+2;j++){
      
      if(i>=0 && j>=0){
	int temp = img->x*j + i;
	img->data[temp].red = RGB_COMPONENT_COLOR;
	img->data[temp].green = 0;
	img->data[temp].blue = 0;
      }
    }
  }
}

PPMPixel getPixel(int x, int y, PPMImage *img)
{
  int i= img->x*y + x;
  if(img){
    return img->data[i];
  }
  else{
    printf("getPixel failed and returned nothing\n");
  }
}

short isGray(int x, int y, PPMImage *img)
{
  //  printf("gray x,y : %d , %d \n",x,y);
  return getPixel(x,y,img).red<220? 1 : 0;
}


double rayTrace(int x0, int y0, float radians, PPMImage *img)
{
  int startx = x0;
  int starty = y0;
  float dy = sin(radians);
  float dx = cos(radians);

  printf("dx: %f dy: %f\n",dx,dy);

  printf("test: %ld\n", (long)(dx*2000));

  long bigxval = (long) (dx*2000);
  long bigyval = (long) (dy*2000);
  
  long x1 = bigxval + x0;
  long y1 = bigyval + y0;

  printf("x1: %ld y1: %ld\n",x1,y1);

  short steep = 0;
  //if(ABS(y1-y0)>abs(x1-x0))
  //  steep = 1;

  if(ABS(dy)>ABS(dx))
    steep = 1;

  if(steep>0){
    int temp = x0;
    x0 = y0;
    y0 = temp;
    temp = x1;
    x1 = y1;
    y1 = temp;
  }

  

  int deltax = x1 - x0;
  int deltay = y1 - y0;
  int error = deltax /2;
  int ystep;
  int y = y0;

  int inc = x0 < x1 ? 1 : -1;

  if(y0<y1){
    ystep=1;
  }
  else{
    ystep=-1;
  }
  
  int x;
  printf("start: %d end: %ld inc: %d\n",x0,x1,inc);
  for(x = x0;ABS(x)< ABS(x1); x+=inc){
    printf("x:%d y:%d error:%d\n",x,y,error);
    if(steep>0){
      if(isGray(y,x,img)>0){
	break;
      }
    }
    else{
      if(isGray(x,y,img)>0){
	break;
      }
    }
    

    if(radians>=0 && radians<=1.5707){
      error -= deltay;
    }
    else if(radians<0 && radians>=-.78539){
      error +=deltay;
    }
    else if(radians<-.78539 && radians>=-1.5705){
      error-=deltay;
    }
    else if(radians<-1.5705){
      error+=deltay;
    }
    
    if(radians>1.5705 && radians<=2.3561){
      error+=deltay;
    }
    else if(radians>2.3561 && radians<3.1415){
      error-=deltay;
    }

    //error -= radians>0 ? deltay : -deltay;// error-deltay;
    if(error < 0){
      y = y + ystep;
      if(radians>=0 && radians<=1.5707){
	error +=deltax;
      }
      else if(radians<0 && radians>=-.78539){
	error +=deltax;
      }
      else if(radians<-.78539 && radians>=-1.5705){
	error -=deltax;
      }
      else if(radians<-1.5705){
	error -=deltax;
      }
      
      if(radians>1.5705 && radians<=2.3561){
	error +=deltax;
      }
      else if(radians>2.3561 && radians<3.1415){
	error-=deltax;
      }
     
    }
  }


  int finalx;
  int finaly;
  finalx = steep>0? y : x;
  finaly = steep>0? x : y;

  double result = 50.0 * sqrt(pow((finalx-startx),2.0) + pow((finaly-starty),2.0));
  
  
  addRedDot(finalx,finaly,img);
  addRedDot(290,125,img);

  printf("X: %d Y:%d\n",finalx,finaly);
  return result;
}

    
    
/*
int main(int argc, char *argv[]){
    PPMImage *image;
    PPMPixel pix;
    float radians = atof(argv[1]);
    image = readPPM("map.ppm");
    //wimage = readPPM("map.ppm");
    //changeColorPPM(image);
    
    printf("Result %f\n",rayTrace(290,125,radians,image));
    //pix = getPixel(365,166,image);
    //printf("R: %d G: %d B: %d\n",pix.red,pix.green,pix.blue);
    writePPM("map2.ppm",image);
}
*/
