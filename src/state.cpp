#include <unistd.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <Mage.h>
#include "motion_utilities.h"
#include "laser.h"
#include "Landscape.h"
#include "ppm.h"
#include "Particle.h"
#include "p_filter.h"
#include "display.h"

#define STATE_UPDATE 0
#define STATE_NAV 1
#define STATE_ACTION 2

PPMImage * construct_map( float** evidence_grid, PPMImage* map );
PPMImage* clear_map(PPMImage* map);


int newArrayList = 0;
int newParticle = 0;
int newFeature = 0;

int main(int argc, char *argv[]){
  
  long State[NUM_STATE]; //Mage dumps state stuff into state
  long *statePtr = State;
  Laser* laser;
  Robot robot;
  Robot *botPtr = &robot;

  Point * points = NULL;
  int state = 0;
  int ang_vel = 0;
  int lin_vel = 0;
  
  //variables for wander_state, must keep track of turn count and next_turn now that wander uses state machine
  int turn_count = 0;
  int next_turn = -1;
  int loop = 1;

  //variables for stop_time counter
  int STOP_TIME = 2400; //count to 2 min
  int stop_counter = 0;

  //variables for map and p_filter
  PPMImage* map;
  PPMImage* red_map;
  long nBuf = 2; //for window
  int n = 40; //use n particles
  P_filter *filter;

  //String Variable for writing .gif
  char *filename = (char*) malloc(sizeof(char)*12);
  int gif_count = 0;

  connectRobot(State, MAGE_MODEL_MAGELLAN, (char *)"/dev/ttyUSB0");   // Connect (serial line) with the robot 
  
  //For some reason robot starts spinning around wildly after connecting. Don't delete this
  // 0 command
  printf("sending 0.0 \n");
  vm(0,0);
 
  irOn(); // Start up IRs
  cout << "IRs on" << endl;
  sonarOn(); // Start up Sonars
  cout << "sonars on" << endl;
  laser = laser_setup();
  cout << "laser on" << endl;
  // catch cntl-c to get a clean shutdown
  signal( SIGINT, sighandler ); 
  
  // set the speed and wait for things to start up
  vm(0,0);
  usleep(3000000);

  //set up the robot structure
  robot.state = statePtr;
  robot.laser = laser;
  robot.last_linear_vel = 0;
  robot.last_angular_vel = 0;
  robot.goal_points = points;
  robot.wall_distance = 0;
  robot.wall_angle = 0;
  robot.goal_theta = 0;
  robot.direction = 1;

  //read in the map and set up the PPMImage and display structures
  map = readPPM("map.ppm");
  red_map = readPPM("map.ppm"); //create a copy so the memory for the data is in place
  initWindow(map->x, map->y);

  //for the purposes of testing (and probably this whole project) use the map_template to draw back on to width/height
  //initialize the evidence grid
  const int max_height = map->y;
  const int max_width = map->x;
  float** evidence_grid;
  evidence_grid = new float*[max_height];
  for( int i = 0 ; i < max_height ; i++ ){
    evidence_grid[i] = new float[max_width];
    for( int j = 0 ; j < max_width ; j++ ){
      evidence_grid[i][j] = 0.01;
    }
  }

  //initialize particle filter using the map and n particles
  filter = new P_filter(botPtr, map, evidence_grid, n);

  while( loop ){
		
    switch( state ){
    case STATE_UPDATE:

      cout << "Memory - Particles , Features, ArrayList : " << newParticle << " , " << newFeature << " , " << newArrayList << endl;

      printf("State Update\n");
      //update the laser, check for obstacles and reassign state based on existence of obstacle
      robot.laser->update();
      robot.direction = robot.last_linear_vel < 0 ? -1:1;
      
      //cout << "before predict" << endl;
      //apply the motion model
      filter->predict();
      cout << "after predict" << endl;
      //update particle probabilties
      filter->correct();
      cout << "after correct" << endl;
      //resample particles based on probabilities
      filter->resample( n , (float) .001);
      cout << "after resample" << endl;

      // no reason to update the evidence grid until its to be displayed
      //filter->update_evidence();

      state = STATE_NAV;
      break;

    case STATE_NAV:
      printf("State Nav\n");
      //wander_state modifies the linear/angular velocities in the robot structure so they can be called in the action state. 
      wander_state(botPtr,&turn_count,&next_turn);
      lin_vel = robot.last_linear_vel;
      ang_vel = robot.last_angular_vel;
      state = STATE_ACTION;
      break;

    case STATE_ACTION:
      printf("State Action\n");
      //cap the velocities and send the vm command. 
      lin_vel = fabs( lin_vel ) > MAX_LINEAR_VEL ? robot.direction*MAX_LINEAR_VEL : lin_vel;
      if( ang_vel > MAX_ANGULAR_VEL ){ ang_vel = MAX_ANGULAR_VEL; }
      else if( ang_vel < -1*MAX_ANGULAR_VEL ){ ang_vel = -1*MAX_ANGULAR_VEL; }
      vm(lin_vel, ang_vel);
      state = STATE_UPDATE;
      break;

    default:
      printf("Default State\n");
      loop = 0;
      state = STATE_UPDATE;
      vm(0,0);
    }

    /*  cout << "copying data" << endl;
    memcpy( red_map->data , map->data,sizeof(PPMPixel)*map->x*map->y); 
    cout << "writing particles" << endl;
    sprintf(filename, "red_map%d.ppm", gif_count);
    writePPM(filename, red_map);
    */

    gif_count += 1;
    if( gif_count % 10 == 1 ){
      cout << "before window refresh" << endl;
      //reset the evidence grid and draw it
      for( int i = 0 ; i < max_height ; i++ ){
	for( int j = 0 ; j < max_width ; j++ ){
	  evidence_grid[i][j] = 0.01;
	}
      }
      filter->update_evidence();
      cout << "refreshingWindow" << endl;
      map = clear_map(map);
      map = construct_map( evidence_grid , map );
      filter->display_particles(map);

      cout << "writing map" << endl;
      sprintf(filename, "fastSLAM%d.ppm", gif_count/10);
      //writePPM(filename, map);t

      refreshWindow(max_width  , max_height,map );
    }
    
    //make sure things aren't happening faster than 10hz
    if( .1 - filter->accum > 0 ){ //need to wait additional ms for 10hz
      usleep( 100000 - filter->accum*1000000 );
      cout << "sleeping" << endl;
    }
    
    //increment the stop counter every 100 ms and if it is greater than stop_time break out of the loop
    stop_counter += 1;
    //cout << "stop_counter : "<< stop_counter <<endl;
    if( stop_counter >= STOP_TIME ){
      loop = 0;
      writePPM("fastslam.ppm" , map);
    }
    
  }
  
  //deallocate evidence grid
  for( int i = 0 ; i < max_height ; i++ ){
    delete[] evidence_grid[i];
  }
  delete[] evidence_grid;

  delete filter;
  //delete botPtr;
  free(filename);

  closeWindow(nBuf);

  //ensure the robot is stopped and shut everything down
  vm(0,0);
  irOff();
  sonarOff();
  laser_disconnect( laser );
  usleep(1000000);
  disconnectRobot(); // Disconnect and shut down sensors

}

//clear the map and return a pointer to it
PPMImage* clear_map(PPMImage *map){
	for (int i = 0; i < MAX_MAP_HEIGHT; i++){
		for (int j = 0; j < MAX_MAP_WIDTH; j++){
			changePixelColor(map, j, i, 255, 255, 255);
		}
	}
	
	return map;
}


PPMImage * construct_map( float** evidence_grid , PPMImage *map){
  for( int i = 0 ; i < MAX_MAP_HEIGHT ; i++ ){
    for(int j = 0 ; j < MAX_MAP_WIDTH ; j++ ){
      //if( evidence_grid[i][j] > 0.0 ){
      //	cout << "evidence grid : " << evidence_grid[i][j] << endl;
      //      }
      if( evidence_grid[i][j] >= EVIDENCE_THRESH ){ //make the occupied pixels black
				changePixelColor( map, j, i, 0, 0 , 0);
      }
      else{
				changePixelColor( map, j, i, 255,255,255);
      }
    }
  }
  return map;
}
