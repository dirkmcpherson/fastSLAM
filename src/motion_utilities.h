#ifndef motion_utilities_h
#define motion_utilities_h
#include "laser.h"
#include <math.h>
#include "ArrayList.cpp"

#define MAX_LINEAR_VEL 400
#define MAX_ANGULAR_VEL 700
#define STOP_THRESH 500
#define SENSOR_DISTANCE_THRESH 200
#define MAX_LASER_DISTANCE 5000 //5600
#define SLOW_AVERAGE_CONSTANT .01
#define FAST_AVERAGE_CONSTANT .1
#define SLEEP_TIME 100000

typedef struct{
	int loc;
	int count;
} ClientData;

typedef struct{
  short sensor_number;
  float value;
  float theta;
} Sensor;

typedef struct{
  float x0,y0,x1,y1;
  float slope,length;
  float y_intercept;
  int start_idx, end_idx;
  int perpendicular_laser_index;
} Line;

typedef struct{
  float x,y;
} Point; 

typedef struct{
  int width,height;
  int (*map_ptr)[5][10];
} Map;

typedef struct{
  long* state;
  Laser* laser;
  int last_linear_vel, last_angular_vel;
  Point* goal_points;
  float goal_theta;
  float wall_distance, wall_angle;
  int direction;
  Line goal_feature;
  int toTravel;
  int toTurn;
  short success;
  int goal;
  int goals;
} Robot;


void sighandler(int);
int isClear(long*, Laser*, int);
int rotate(long*, float, long);
int move(long*,Laser *, long, long, long, long);
int goto_coords(long*, Laser *, long, long);
int wander(Robot*);
void wander_state(Robot*,int*,int*);
int wall_find(Robot*);
Laser* laser_setup(void);
void laser_disconnect(Laser*);
void emergency_stop(long,int);
float get_new_direction(long*,Laser*,int,int);
short check_right_angles( Laser * laser);
long follow(long*, void*, long, int);
long followgeneral(long*, void*, long, int, int);
Sensor get_min_sonar( long*, int*, short);
Sensor get_max_sonar( long*, int*, short);
Sensor get_min_laser(Laser*, short, short,short);
Sensor get_max_laser(Laser*, short, short,short);
Line find_features(Laser *, short*, short,float, void*);
Line get_line(Laser *, int, int);
int compare_lines( void*, void*);
int compare_ints(void*, void*);
int get_median_index(short*, short);
Line median_RANSAC( Laser*, short*, short, float, int, void*);
void turn_to_wall( Robot*, int);
int wall_follow(Robot*, int, float, float);
int get_shortest_perpendicular_distance(Laser*, int, int);
void gotoxy(Robot* rbt, int x, int y);

#endif
