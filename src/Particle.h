#ifndef Particle_h
#define Particle_h
#include "motion_utilities.h"
//#include "global.h"
#include <stdio.h>
#include <stdlib.h>
#include "ppm.h"


#define GRID_RESOLUTION 50 //each space of occupancy grid is assumed to be a square of this side length (mm)
#define LINEAR 0
#define ANGULAR 1
#define PROBABLE_FEATURE -1
#define NO_FEATURE 0
#define NEW_FEATURE 1
#define FEATURE_DEVIATION 3
#define DEFAULT_FEATURE_WEIGHT .5
#define MAX_MAP_WIDTH 580
#define MAX_MAP_HEIGHT 255
#define EVIDENCE_THRESH .95 //requires this much evidence to consider a grid space filled
#define MIN_PROB 1e-4//1e-6 //min probability reading allowed
#define MAX_PROB .99 //max probability reading allowed
#define NUM_SENSOR_READINGS 10
#define CLEAN_UP_COUNT 30
#define MAX_STRIKES 5 

//sensor covariances will be in terms of grid spaces to coordinate with feature covariances 
#define SENSOR_COVARIANCE_X 3 //linear covariance in the x direction
#define SENSOR_COVARIANCE_Y 1 //angular covariance

typedef struct{
  float x,y,theta;
} Pose;

typedef struct{
  //Feature() { newFeature += 1; }
  //~Feature() { newFeature -= 1; }
  int x,y;
  float x_cont, y_cont;
  float x_dev,y_dev;
  float weight;
  float distance;
  int strike;
  double Q11,Q12,Q21,Q22,iQ11,iQ12,iQ21,iQ22;
  double c11,c12,c21,c22; //covariance matrix
}Feature;

extern int newParticle;
extern int newFeature;

class Particle {

private:
  static float l; //temp values (mm)
  static float r;
  static int num_sensor_readings;
  int map_width,map_height;
  PPMImage* map; 
  float** evidence_grid;
  Pose pose;
  float probability;
  float slow_average,fast_average;
  ArrayList< Feature* > *features;
  
public:
  int count;
  ArrayList< Feature* > *toRemove;
  Particle( PPMImage*, float**); //initalize the particle with an occupancy grid as well as the width and length values of tha grid
  ~Particle(void);
  void randomize(void);
  void update_position( float, float,double );
  void calculate_probability(Laser*); //TODO add laser pointer to particle
  Feature* measurement_prediction( int, int, int ,int,int);
  float distance(Feature *f);
  void sort();
  void update_features(Laser*);
  void add_strike(int, int, double);
  int isFeature(double ,double, double);
  bool featureCheck(int botx, int boty, int fx, int fy, int endx, int endy, int sig3x, int sig3y, int mrad);
  void update_average(void);
  float calc_gauss(float, float,float);
  float random_noise(int);
  Particle* copy_particle(void);
  void add_feature( double, double );
  Feature* copy_feature( Feature* );
  float get_probability(void);
  float get_average(void);
  Pose* get_pose(void);
  ArrayList<Feature*> *getFeatures(void);
  void set_pose(int, int, float);
  void set_probability( float );
  void set_average(float,float);
  void set_feature_list( ArrayList< Feature* > * );
  void print_pose(void);
  void print_features(void);
  void print_feature(Feature* feature);
};


#endif
