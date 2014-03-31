#include <unistd.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <Mage.h>
#include "laser.h"
#include "motion_utilities.h"
#include "ArrayList.cpp"
//#include "Landscape.h"



void sighandler(int signal) {

  // turn everything off and disconnect
  irOff();
  sonarOff();
  disconnectRobot();
  
  fprintf(stderr, "Exiting on signal %d\n", signal);

  exit(-1);
}


//Find the minimum sonar reading between the given start and end indicies, grouping the sonars by an input value
Sensor get_min_sonar( long* state, short * sonars, short num_sonars ){
  float curr = 7000;
  float min = curr; 
  Sensor min_sonar;
  short rand_choice;
  srand(time(NULL));
  for(int i = 0 ; i < num_sonars ; i++){
    curr = state[sonars[i] + STATE_SONAR_0];
    rand_choice = rand() % 2 == 0 ? -1 : 1;
    if( curr < min){
      if( min - curr  <= SENSOR_DISTANCE_THRESH && rand_choice > 0){
	continue;
      }
      else{
	min = curr;
	min_sonar.value = min;
	min_sonar.sensor_number = sonars[i];
      }
    }
  }
  //determine theta of max_sonar
  if( min_sonar.sensor_number < 9 ){ //for 0 : 180
    min_sonar.theta = min_sonar.sensor_number*.39; //each segment takes up 22.5 degrees
  }
  else{ //for 0 : -180
    min_sonar.theta = min_sonar.sensor_number*.39 - 2*M_PI;
  }
  min_sonar.theta *= 1000;
  // printf( "Min_Sonar ( # , value , theta) : ( %d , %f , %f)\n" , min_sonar.sensor_number, min_sonar.value, min_sonar.theta);
  return(min_sonar);
}

//Find the minimum sonar reading between the given start and end indicies, grouping the sonars by an input value
Sensor get_max_sonar( long* state, short * sonars, short num_sonars ){
  float curr = 0;
  float max = curr; 
  Sensor max_sonar;
  short rand_choice;
  srand(time(NULL));
  for(int i = 0 ; i < num_sonars ; i++){
    curr = state[sonars[i] + STATE_SONAR_0];
    rand_choice = rand() % 2 == 0 ? -1 : 1;
    if( curr > max){
      if(curr - max  <= SENSOR_DISTANCE_THRESH && rand_choice > 0){
	continue;
      }
      else{
	max = curr;
	max_sonar.value = max;
	max_sonar.sensor_number = sonars[i];
      }
    }
  }
  //determine theta of max_sonar
  if( max_sonar.sensor_number < 9 ){ //for 0 : 180
    max_sonar.theta = max_sonar.sensor_number*.39; //each segment takes up 22.5 degrees
  }
  else{ //for 0 : -180
    max_sonar.theta = max_sonar.sensor_number*.39 - 2*M_PI;
  }
  max_sonar.theta *= 1000;
  //printf( "Max_Sonar ( # , value, theta ) : ( %d , %f , %f )\n" , max_sonar.sensor_number, max_sonar.value, max_sonar.theta);
  return(max_sonar);
}


// find the max laser value between the start and end indeces, grouping the laser by group#
Sensor get_min_laser( Laser * laser, short startIdx, short endIdx, short group ){
  float curr = 5000;
  float min = curr;
  float laser_average = 0;
  Sensor min_laser;
  short rand_choice;
  short group_count = 0;

  srand(time(NULL));
  laser->update();
  //give the laser a tenth of a second to update
  // usleep(100000);
  for( short i = startIdx ; i < endIdx + 1 ; i++){
    /*average up "group" laser readings, a bit lavish on the counters, but modding 
    and division of startIdx/group runs into problems with certain group values. 
    A full group of lasers must be counted or false minimums are reported (big problem)
    */
    if(group_count < group){
      laser_average += laser->value(i);
      group_count += 1;
      continue;
    }

    //once group number of lasers has been counted, average and check against ma
    curr = laser_average / group;
    rand_choice = rand() % 2 == 0 ? -1 : 1;
    if( curr < min){
      if(min - curr  <= SENSOR_DISTANCE_THRESH && rand_choice > 0){
	continue;
      }
      else{
	min = curr;
	min_laser.value = min;
	min_laser.sensor_number = i - group / 2;
	min_laser.theta = laser->angle(min_laser.sensor_number) * 1000; //convert angle to mrad

 	//test code to print out the 10 readings that generated the value
	//	for( int j = i - group ; j < i ; j++ ){ 
	//	  printf(" %f\n ", laser->value(j));
	//	}
      }
    }
      //grab the current reading for the next group
    laser_average = laser->value(i);
    group_count = 1;
  }
  //  printf("Min_laser ( # , value , theta ) : ( %d , %f , %f ) \n", min_laser.sensor_number , min_laser.value, min_laser.theta);
  return(min_laser);
}


// find the max laser value between the start and end indeces, grouping the laser by group#
Sensor get_max_laser( Laser * laser, short startIdx, short endIdx, short group ){
  float curr = 0;
  float max = curr;
  float laser_average = 0;
  Sensor max_laser;
  short rand_choice;
  short group_count = 0;
  srand(time(NULL));
  laser->update();
  for( short i = startIdx ; i < endIdx + 1 ; i++){
    if(group_count < group){
      laser_average += laser->value(i);
      group_count += 1;
      continue;
    }

    //once group number of lasers has been counted, average and check against ma
    curr = laser_average / group;
    //printf( "Laser Average : %f / %d\n",laser_average,group);
    rand_choice = rand() % 2 == 0 ? -1 : 1;
    if( curr > max){
      if(curr - max  <= SENSOR_DISTANCE_THRESH && rand_choice > 0){
	continue;
      }
      else{
	max = curr;
	max_laser.value = max;
	max_laser.sensor_number = i - (short) group / 2;
	max_laser.theta = laser->angle(max_laser.sensor_number) * 1000; //convert angle to mrad
      }
    }
    group_count = 0;
    laser_average = 0;
  }
  //printf("Max_laser ( # , value , theta) : ( %d , %f ,%f ) \n", max_laser.sensor_number , max_laser.value, max_laser.theta);
  return(max_laser);
}

/* for wander only use the middle three sensors, using five makes it hard to move around */
int isClear(long *state,  Laser *laser, int direction){
  //  int front_sonars[5] = {0,1,2,14,15};
  //int back_sonars[5] = {6,7,8,9,10};
  static short numSensors = 3;
  int front_sonars[3] = {0,1,15};
  int back_sonars[3] = {7,8,9};

  float minDist = 2*STOP_THRESH; //TODO: set to max sonar distance
  static short backOffset = back_sonars[0] + STATE_SONAR_0; //offset to grab min distance from
  short i,j,clear;

  //if moving backwards grab the shortest reading from the back sonars and return -1 if something in way
  if(direction < 0){
    for( i = 0; i < numSensors; i++){
      //     printf("Sonar %d : %ld\n",back_sonars[i], state[ (back_sonars[i] + STATE_SONAR_0) ]); 
     if( state[back_sonars[i] + STATE_SONAR_0] < minDist ){
       minDist = state[i + backOffset];
     }
    }
  }

  //same if moving forwards but with front sensors
  else{
    float laser_average = 0;
    const short laser_group = 4;
    // sensor front_laser;
    for( i = 0; i < numSensors; i++){
      if(state[front_sonars[i] + STATE_SONAR_0] < minDist){
	minDist = state[i + STATE_SONAR_0];
      }
    }
    /* 
    front_laser = get_min_laser( laser, 281 , 400, 10);
    if( front_laser.value < minDist ){
      minDist = front_laser.value;
    } 
    */
    
   //Grab the front 100 laser readings, grouped into sets of 4 to get an average 
   laser->update();
   for( i = 240/laser_group ; i < 441 / laser_group ; i++ ){
     // printf("Laser info (ang, value) : ( %f , %f )\n", laser->angle(i), laser->value(i));
     for( j = 0 ; j < laser_group ; j++ ){ 
       laser_average += laser->value(i*laser_group + j);
     }
     if( laser_average / laser_group < minDist ){
       minDist = laser_average/laser_group;
       //printf("Min Laser Distance found at ( angle, dist ) : ( %f , %f )\n", laser->angle(i), minDist); 
    }
    laser_average = 0; 
   } 
  }
   
  //printf("Smallest Sonar Distance : %ld \n",minDist);
  clear = minDist <= STOP_THRESH ? -1 : 0;
  //  printf("mindist : %f\n",minDist);
  return(clear);
}

//gradually halt the robot once an obstacle is detected
void emergency_stop(long vel,int direction){
  while( (direction*vel) > 0 ){
    vel -= direction*MAX_LINEAR_VEL/8;
    vm(vel,0);
    //printf("Emergency Stop Vel : %ld\n",vel);
    usleep(100000);
  }
}

/* rotate takes a goal orientation in milliradians and iterates until it reaches that orientation
   returns a (1) when complete
    */
int rotate( long *state, float target, long velocity){
  long error;
  long dt;
  long maxVel = MAX_ANGULAR_VEL; 
  long numIncs = 10;  //number of increments velocity can be throttles up
  long velInc = maxVel/numIncs; //one increment increase in velocity for smooth motion
  long minVel = velInc; //set the minVel to be 1 velInc to fix bug where error < velInc
  const int stopThresh = 50; //stop thresh is ~3 degrees
  short direction = 0; //-1 is CW , 1 is CW
  
  //fprintf(stderr,"Theta: %ld\n",state[STATE_T]);

 //get new state and put it in the -pi to pi space
  dt = target - state[STATE_T]; //JUST CHANGED THIS 2.28.13

  if( dt > 3142 ){
    dt -= 6283;
  }
  else if( dt < -3142 ){
    dt += 6283;
  }
  error = dt; 
  direction = error < 0 ? -1 : 1;

  //printf("error : %ld\n" , error);

  //if close enough to goal, break
  if(fabs(error) <= stopThresh){
    return(0);
  }
   
  //TODO: fix for when error is between minVel and minVel + velInc
  //if error is greater than 0, ratchet up the velocity to turn CCW
  if( error > 0){
    if( velocity < error ){ //if moving CCW slower than error speed up
      velocity += velInc;
     }
     else{
       velocity -= velInc; //otherwise slow down 
     }
   } 
  else if(error < 0){
  if(velocity > error){ //if moving CW slower than the error speed up
      velocity -= velInc;
    }
    else{   //otherwise slow down
      velocity += velInc;
    }
  }   
 
 // final check to be sure the velocity lies within the bounds
  if( fabs(velocity) > maxVel){
     velocity = direction*maxVel;
   }
  if( fabs(velocity) < minVel ){
     velocity = direction*minVel;
   }
  //  printf("Velocity: %d \n",velocity);
  return(velocity);
}

//move the robot linearly forward or backwards by [target] distance
int move(long *state, Laser * laser, long x0, long y0, long target, long velocity){
  int dx,dy,dist;
  int direction, error;
  static const int stopThresh = 40; //stop within mm 
  //static const int maxVel = MAX_LINEAR_VEL; //to start, limit the bot to .5 m/s
  //static const short numIncs = 20;
  //static const short velInc = maxVel/numIncs;
  //static const int minVel = 20; //move at least 20 mm/s (CHECK IF THIS IS > MIN)
	
	long newVel;
	long prevVel;
  direction = target < 0 ? -1 : 1; //keep track if we're moving forwards or backwards

  //check if an obstacle is the way, if it is issue a vm(0,0) command and return
  if( isClear(state,laser,direction) < 0 ){
    printf("Obstacle detected \n");
    emergency_stop(velocity, direction);
    return(0);
  }

  dx = state[STATE_X] - x0;
  dy = state[STATE_Y] - y0;
    
  dist = direction * sqrt(dx*dx + dy*dy);
  error = target - dist;

	newVel = error;
	prevVel = velocity;
	//	printf("Velocity Test: %d %d\n", error, stopThresh);
	if (error < stopThresh){
		newVel = prevVel - 20 > newVel ? prevVel - 20 : newVel;
		newVel = newVel <= -500 ? -500 : newVel;
	}
	else if (error > -1 * stopThresh){
		newVel = prevVel + 20 < newVel ? prevVel + 20 : newVel;
		newVel = newVel >= 500 ? 500 : newVel;
	}
	else{
		newVel = newVel < 0 ? newVel + 20 : newVel - 20;
	}
	newVel = newVel < 20 && newVel > -20 ? 0 : newVel;
	
	usleep(10000);
	
	return(newVel);
}

int goto_coords(long *state,Laser * laser, long targetX, long targetY ){
  int x0,y0,dx,dy;
  double theta;
  long linDistance;
  long angVel = 1;
  long linVel = 0;
  short direction = 1; //need direction variable to tell isClear() which side to check for obstacles

  // calculate the difference between the start location and the end location
  x0 = state[STATE_X];
  y0 = state[STATE_Y];
  dx = targetX - x0;
  dy = targetY - y0;

	//calculate the linear distance that must be traversed
  linDistance = sqrt( dx*dx + dy*dy );

  //calculate the angle between the current location and the goal location
  theta = atan2(dy,dx);
 
	if (theta < M_PI / -2 || theta > M_PI / 2){
		theta = M_PI + theta;
		direction = -1;
		linDistance *= direction;
	}

	//convert to milliradians
  theta *= 1000;
  printf("goal orientation: %f \n", theta);

  //use the simple strategy of rotating to the correct orientation and going, move will return -1 if it detects an object
  //TODO: change this to something interested and more effective
  while(1){
    while(angVel != 0){
      angVel = rotate(state,theta,angVel);
      vm(0,angVel);
    }
    linVel = move(state,laser,x0,y0,linDistance,linVel);
    //check for obstacles
    if(isClear(state,laser,direction) < 0 && linVel != 0){
      emergency_stop(linVel, 1);
      printf("Halting Movement\n");
      return(-1);
    }
    if(linVel == 0){
      vm(0,0);
      break;
    }
    vm(linVel,angVel);
    usleep(100000);
  }

  return(0);
}


/*utility function to prefer right angled turns while wandering ( explore a room ) if theres space above a threshhold
to either the right or left (as told by laser) then automatically go that way. return -1 for right turn (CW) 1 for left (CCW) 
0 if not enough space*/
short check_right_angles( Laser * laser){
  const int threshold = 5000; //only prefer a turn if theres a ton of open space on one side
  Sensor right,left;
  short decision = 0;
  right = get_max_laser(laser, 110, 121, 10);
  left = get_max_laser(laser, 570, 581, 10);
  if( right.value >= threshold || left.value >= threshold ){
    decision = right.value > left.value ? -1 : 1;
  }
  return(decision);
}

//-1 for CW, 1 for CCW 
float get_new_direction(long* state, Laser * laser, int next_turn){
  // int next_turn = -1; //next_turn determines what directoin (CCW vs CW) the robot prefers to go
  int change_turn_thresh = 1000; //only change turn direction if theres drastically more space on one side
  Sensor left_laser,right_laser,left_sonar,right_sonar;
  //ignore the back 3 sonars
  short left_sonar_array[6] = {10,11,12,13,14,15};
  short right_sonar_array[7] = {0,1,2,3,4,5,6};
  short *left = left_sonar_array;
  short *right = right_sonar_array;
  short right_angle = 0;
  float max_left,max_right;
  float goal_orientation;
  
  //If theres an open 90 turn, take it.
  right_angle = check_right_angles(laser);
  if( right_angle != 0 ){
    goal_orientation = right_angle > 0 ? M_PI/2 : -M_PI/2;
    goal_orientation *= 1000;
    return(goal_orientation);
  }
  // get the left and right greatest sensor values
  left_laser = get_max_laser( laser, 341, 680, 20 );
  right_laser = get_max_laser( laser, 0, 340, 20);
  left_sonar = get_max_sonar(state, left, 6);
  right_sonar = get_max_sonar(state, right, 7);
  
  //normalize sonar readings so sonars and lasers can be compared on the same grounds
  if( left_sonar.value > 5000 ){ left_sonar.value = 5000; }
  if( right_sonar.value > 5000 ){right_sonar.value = 5000; }

  max_left = left_laser.value > left_sonar.value ? left_laser.value : left_sonar.value;
  max_right = right_laser.value > right_sonar.value ? right_laser.value : right_sonar.value;

  //if the difference between left and right is > change_turn_thresh change the preferred direction of turning
  if( fabs( max_left - max_right ) >= change_turn_thresh ){
    next_turn = max_left > max_right ? 1 : -1;
  }
  
  if( next_turn == -1 ){
    goal_orientation = right_laser.value > right_sonar.value ? right_laser.theta : right_sonar.theta;
  }
  else{
    goal_orientation = left_laser.value > left_sonar.value ? left_laser.theta : left_sonar.theta;
  }
  printf( "Goal_Orientation : %f\n", goal_orientation);
  return(goal_orientation);
}


/* wander function slowly moves around the landscape until a sensor set picks up an obstacle within STOP_THRESH
   it then turns in a random direction chosen between the sensor sets that show the greatest amount of open space */
int wander(Robot* robot){
  Laser *laser = robot->laser;
  long* state = robot->state;
  long linear_velocity = 0;
  long angular_velocity = 1;
  float desired_theta = 0;
  short direction = 1; //assume robot starts out moving forward
  int turn_count = 0;
  int next_turn = -1;
  while(1){
    while(isClear(state, laser,direction) >= 0){
			if(direction*linear_velocity < MAX_LINEAR_VEL){
	  		linear_velocity += direction*MAX_LINEAR_VEL/10;
			}
			vm(linear_velocity,0);
			usleep(100000);
    }
    emergency_stop(linear_velocity, direction);
    desired_theta = get_new_direction(state,laser,next_turn);
    //make the desired theta with respect to the current state since thats how rotate is written
    desired_theta += state[STATE_T];
      
    //switch the preferred turn direction after 3 turns
    turn_count += 1;
    if(turn_count % 3 == 0){ next_turn *= -1; }
    printf("Goal Theta : %f\n", desired_theta);
    while( fabs(angular_velocity) > 0 ){
			angular_velocity = rotate(state,desired_theta,angular_velocity);
			vm(0,angular_velocity);
			laser->update();
			usleep(100000);
     }
     
      //hack so the bot will turn next time
      angular_velocity = 1;
      linear_velocity = 0;
      usleep(100000);
  }

  return(0);
}

/* wander for badge detection, modified to work in a state machine */
void wander_state(Robot* robot, int* turn_count, int* next_turn){
  Laser *laser = robot->laser;
  long* state = robot->state;
  long linear_velocity = robot->last_linear_vel;
  long angular_velocity = robot->last_angular_vel;
  float desired_theta = robot->goal_theta;
  short direction = 1; //assume robot starts out moving forward

  if(isClear(state, laser,direction) >= 0 && robot->last_angular_vel >= 0){// >= 0){ //if we were moving forward last time
    if(direction*linear_velocity < MAX_LINEAR_VEL){
      linear_velocity += direction*MAX_LINEAR_VEL/10;
    }
    //vm(linear_velocity,0);
    robot->last_linear_vel = linear_velocity;
    robot->last_angular_vel = 0;
    return;
  }

  //only stop and get a new direction if the previous action was not a turn
  if( robot->last_angular_vel == 0 ){
    emergency_stop(linear_velocity, direction);
    desired_theta = get_new_direction(state,laser, *next_turn);
    //make the desired theta with respect to the current state since thats how rotate is written
    desired_theta += state[STATE_T];  
    //switch the preferred turn direction after 3 turns
    *turn_count += 1;
    if(*turn_count % 3 == 0){ *next_turn *= -1; }
    
    robot->goal_theta = desired_theta;
  }


  angular_velocity = rotate(state,desired_theta,angular_velocity);
  vm(0,angular_velocity);
  //printf("desired_theta: %f\n", desired_theta);
  //printf("current theta: %ld\n", state[STATE_T]);
  //printf("angular_velocity : %ld\n", angular_velocity);
  robot->last_angular_vel = angular_velocity;
  robot->last_linear_vel = 0;
  return;
}



//wander function for finding and latching onto a wall. When the function finds a wall it returns the model
/*int wall_find( Robot* robot){
  Laser *laser = robot->laser;
  long* state = robot->state;
  long linear_velocity = 0;
  long angular_velocity = 1;
  float desired_theta = 0;
  short direction = 1; //assume robot starts out moving forward
  int turn_count = 0;
  int next_turn = -1;

  //for feature detection
  Line wall; //the wall model that will be returned
  short laser_count = laser->count();
  short* laser_points = new short [laser_count];
  Landscape *scape = new Landscape(laser);
  int feature_length_threshold = 500;

  //put all the laser indexes in an array for median_RANSAC
  for( int i = 0; i < laser_count ; i++){
    laser_points[i] = i;
  }

  while(1){
    while(isClear(state, laser,direction) >= 0){
	if(direction*linear_velocity < MAX_LINEAR_VEL){
	  linear_velocity += direction*MAX_LINEAR_VEL/10;
	}
	vm(linear_velocity,0);
	robot->last_linear_vel = linear_velocity;
	//look for walls on the left and right side
	wall = median_RANSAC( laser, laser_points, laser_count, 3000, 5, scape);
	//if the found feature is long enough, return to wall follow
	if( wall.length >= feature_length_threshold ){
	  robot->goal_feature = wall;
	  return(0);
	}
	usleep(100000);
    }
      emergency_stop(linear_velocity, direction);
      robot->last_linear_vel = 0;
      //once an emergency stop occurs, look for a wall in front (otherwise its just a pair of inconsiderate legs)
      //look for walls on the left and right side
	wall = median_RANSAC( laser, laser_points, laser_count, 3000, 5, scape);
	//if the found feature is long enough, return to wall follow
	if( wall.length >= feature_length_threshold ){
	  robot->goal_feature = wall;
	  return(0);
	}

      //if no wall model is found, keep wandering
      desired_theta = get_new_direction(state,laser,next_turn);
      //make the desired theta with respect to the current state since thats how rotate is written
      desired_theta += state[STATE_T];
      
      //switch the preferred turn direction after 3 turns
      turn_count += 1;
      if(turn_count % 3 == 0){ next_turn *= -1; }
      printf("Goal Theta : %f\n", desired_theta);
      while( fabs(angular_velocity) > 0 ){
	angular_velocity = rotate(state,desired_theta,angular_velocity);
	vm(0,angular_velocity);
	robot->last_angular_vel = angular_velocity;
	laser->update();
	usleep(100000);
      }
      //hack so the bot will turn next time
      angular_velocity = 1;
      linear_velocity = 0;
      usleep(100000);
  }
  
  delete scape;
  delete[] laser_points;

  return(-1);
}

// function that keeps the robot dist meters from the object in front of it
long follow(long *state, void* lsr, long last_velocity, int dist){
	Laser *laser = (Laser *) lsr;
	long linear_velocity = 0;
//	long angular_velocity = 1;
//	short direction = 1;
	const int laser_group = 4;
  float laser_average = 0;
	float minDist = 10000;
		
	int i;
	int j;
	
	laser->update();
	usleep(1000);
	for( i = 280/laser_group ; i < 401 / laser_group ; i++ ){
		// printf("Laser info (ang, value) : ( %f , %f )\n", laser->angle(i), laser->value(i));
		for( j = 0 ; j < laser_group ; j++ ){ 
		  laser_average += laser->value(i*laser_group + j);
		}
		//printf("laseravg: %lf\n", laser_average/4.0);
		if( laser_average / 4.0 < minDist ){
		  minDist = laser_average/4.0;
		  //printf("Min Laser Distance found at ( angle, dist ) : ( %f , %f )\n", laser->angle(i), minDist); 
		}
		laser_average = 0; 
	}

	//robot is too close
	if(/*state[STATE_SONAR_0] < 950 && *//*minDist < dist - 50){
	
		//assign error to linear_velocity
		linear_velocity = minDist - dist; //move(state, laser, 0, 0, (1000 + minDist), linear_velocity);
	
		//increase speed negatively within bounds
		linear_velocity = last_velocity - 20 > linear_velocity ? last_velocity - 20 : linear_velocity;
		linear_velocity = linear_velocity < -400 ? -400 : linear_velocity;
	
		//don't crash!
		if (!isClear(state, laser, -1) < 0){
			emergency_stop(linear_velocity, 1);
		}
	
		//go back
		//vm(linear_velocity, 0);
	
	}

	//robot is too far
	else if(/*state[STATE_SONAR_0] > 1050 &&*//* minDist > dist + 50){
	
		//calculate velocity
		linear_velocity = minDist - dist; //move(state, laser, 0, 0, (1000 - minDist), linear_velocity);
	
		//increase speed positively within bounds
		linear_velocity = last_velocity + 20 < linear_velocity ? last_velocity + 20 : linear_velocity;
		linear_velocity = linear_velocity > 400 ? 400 : linear_velocity;
	
		//don't crash!
		if (isClear(state, laser, 1) < 0){
			emergency_stop(linear_velocity, 1);
		}
	
		//go back
		//vm(linear_velocity, 0);
	
	}
	else{
		//slow to a stop
		linear_velocity = linear_velocity < 20 && linear_velocity > -20 ? 0 : linear_velocity;
		while(1){
			linear_velocity = linear_velocity < 0 ? linear_velocity + 20 : linear_velocity - 20;
			linear_velocity = linear_velocity < 20 && linear_velocity > -20 ? 0 : linear_velocity;
			if (linear_velocity == 0){
				//vm (linear_velocity, 0);
				return (0);
			}
		}
	}
	printf("MinDistJoey: %.2lf\n", minDist);
	return linear_velocity;
}

// function that keeps the robot dist meters from the object in front of it
long followgeneral(long *state, void* lsr, long last_velocity, int dist, int index){
	Laser *laser = (Laser *) lsr;
	long linear_velocity = 0;
//	long angular_velocity = 1;
//	short direction = 1;
  float laser_average = 0;
  int minDist = 10000;
		
	int i;
	
	laser->update();
	minDist = 1000000;
	for( i = index - 1 ; i <= index + 1; i++ ){
		  laser_average += laser->value(i);
	}
		
	minDist = (int) laser_average / 3;

	//robot is too close
	if(/*state[STATE_SONAR_0] < 950 && *//*minDist < dist - 50){
	
		//assign error to linear_velocity
		linear_velocity = minDist - dist; //move(state, laser, 0, 0, (1000 + minDist), linear_velocity);
	
		//increase speed negatively within bounds
		linear_velocity = last_velocity - 20 > linear_velocity ? last_velocity - 20 : linear_velocity;
		linear_velocity = linear_velocity < -400 ? -400 : linear_velocity;
	
		//don't crash!
		if (!isClear(state, laser, -1) < 0){
			emergency_stop(linear_velocity, 1);
		}
	
		//go back
		//vm(linear_velocity, 0);
	
	}

	//robot is too far
	else if(/*state[STATE_SONAR_0] > 1050 &&*//* minDist > dist + 50){
	
		//calculate velocity
		linear_velocity = minDist - dist; //move(state, laser, 0, 0, (1000 - minDist), linear_velocity);
	
		//increase speed positively within bounds
		linear_velocity = last_velocity + 20 < linear_velocity ? last_velocity + 20 : linear_velocity;
		linear_velocity = linear_velocity > 400 ? 400 : linear_velocity;
	
		//don't crash!
		if (isClear(state, laser, 1) < 0){
			emergency_stop(linear_velocity, 1);
		}
	
		//go back
		//vm(linear_velocity, 0);
	
	}
	else{
		//slow to a stop
		linear_velocity = linear_velocity < 20 && linear_velocity > -20 ? 0 : linear_velocity;
		while(1){
			linear_velocity = linear_velocity < 0 ? linear_velocity + 20 : linear_velocity - 20;
			linear_velocity = linear_velocity < 20 && linear_velocity > -20 ? 0 : linear_velocity;
			if (linear_velocity == 0){
				//vm (linear_velocity, 0);
				return (0);
			}
		}
	}
	return linear_velocity;
}

/* Find the lines around the robot using RANSAC within an input range of the robot. For now, only use the laser for this. Takes
   in an array of laser index points to check, the length of that array, and the max distance a point will be considered.
 */
 /*
Line find_features(Laser * laser, short* points, short laser_points_length, float threshold, void* lscape){
    float p = .95; //desired likelihood of finding complete set of inliers
    float w = .3; //percentage of points which are likely to be inliers
    int k; //the number of iterations required to meet above goals
    short numPoints = laser_points_length;
    short idx;
    int p1,p2; //indices of random points chosen
    int curr_inliers = 0; //number of inliers found
    int max_inliers = 0;
    float xp,yp,d,theta;
    Line model;
    float distance_threshold = 15; //points within 2 cm are considered inliers
//    int shortest_line = 20; //the shortest line must consist of at least 20 points
    ArrayList<short> *laser_points, *inliers,* saved_inliers,* tmp;
    inliers = new ArrayList<short>(numPoints);
    saved_inliers = new ArrayList<short>(numPoints);
    laser_points = new ArrayList<short>(points, laser_points_length);
    
    Landscape* scape = (Landscape*) lscape;
    
    scape->clear();
    ///for (int c = 0; c < 3; c++){
		  k = (int) floor(log( 1 - p )/log( 1 - w*w));
		  
		  //printf("K : %d\n",k);

		  laser->update();
		  srand(time(NULL)); //seed random number generator
		  for (int i = 0; i < k; i++) { //for k iterations
		    //pick two random points, check if they're within the threshold distance
		    p1 = laser_points->get(rand() % numPoints);
		    p2 = laser_points->get(rand() % numPoints);
		   
		    //check if points are the same of p2 is > thresh
		    while( p1 == p2){
					p2 = laser_points->get(rand() % numPoints);
		    }
		    
		    model = get_line(laser, p1, p2);
		    
		    /*go through every other point and determine whether they are inliers
				If they are, add their indices to an array, keep track of how many *//*
		    for(int j = 0; j < numPoints; j++){
					idx = laser_points->get(j);
					if (idx == p1 || idx == p2) { //ignore the points used to construct the model
						continue;
					}
					else{
						d = laser->value(idx);
						//only care about points within input threshold
						if(d > threshold){ continue; }
			
						theta = laser->angle(idx);
						xp = d*cos(theta);
						yp = d*sin(theta);
						//check if the point lies within 1cm above or below the line if so store the index of the point and increment curr_inliers
						if( yp - model.slope*xp - model.y_intercept <= distance_threshold && -distance_threshold <= yp - model.slope*xp - model.y_intercept ){
			  			inliers->add(idx);
			  			curr_inliers += 1;
						}
					}
		    }
		    
		    /* if the current set of inliers contains more points than the stored set, point the stored set at the 
			 	curr set and let the curr set write over the stored set. *//*
		    if( (curr_inliers - 1) > max_inliers ){
					max_inliers = curr_inliers - 1; //pointer swapping
					tmp = saved_inliers;
					saved_inliers = inliers;
					inliers = tmp;
				}
		   	inliers->clear();
		   	curr_inliers = 0;
		  }
		  
		  //while testing print out the list of inliers found
		  //printf( "%d inliers found from %d to %d\n", max_inliers, saved_inliers[0], saved_inliers[max_inliers]);
		  //if( max_inliers < shortest_line ){
		    
		    //    }

		  /*now saved_inliers holds the max_inliers in the model from right most laser to leftmost laser
		    therefore the line we want is the one from saved_inliers[0] to saved_inliers[max_inliers]*//*
		  model = get_line(laser, saved_inliers->get(0), saved_inliers->get(max_inliers));
		  //cout << "Model: " <<model.length << endl;
		  //scape->addLine(model);
		  //laser_points->remove(inliers);
		//}
		
    //cout << last.length << endl;
		//model = scape->closestLine();
    return(model);
}

/*
  take in the indices of the start and end laser points that make the line up, find the index between these two that
  has the shortest distance. The angle of this line will be the amount that must be turned by. 
 */
int get_shortest_perpendicular_distance(Laser* laser, int start, int end){
  float min_distance = MAX_LASER_DISTANCE;
  int min_idx = 0;
  for( int i = start ; i < end + 1; i++ ){
    if( laser->value(i) < min_distance ){
      min_distance = laser->value(i);
      min_idx = i;
    }
  }
  return(min_idx);
}

/* make a function that takes in an array of (x,y) points, and returns the model best fitting those points */

//given two laser indices, return the line between the points
/*
Line get_line(Laser *laser, int p1, int p2){
  float theta,d;
  float x0,y0,x1,y1;
  Line l;
  
  d = laser->value(p1);
  theta = laser->angle(p1);
  x0 = d*cos(theta);
  y0 = d*sin(theta);
  
  d = laser->value(p2);
  theta = laser->angle(p2);
  x1 = d*cos(theta);
  y1 = d*sin(theta);
  
  l.x0 = x0;
  l.y0 = y0;
  l.x1 = x1;
  l.y1 = y1;
  l.length = sqrt( (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) );
  //cout << "length: " << l.length << endl;
  l.slope = (y1 - y0)/(x1 - x0);
  l.y_intercept = y0 - l.slope*x0;
  l.start_idx = p1;
  l.end_idx = p2;
  l.perpendicular_laser_index = get_shortest_perpendicular_distance( laser, p1, p2);
  
  return(l);
}

int compare_lines( const void * c, const void * d){
  Line * la = (Line *) c;
  Line * lb = (Line *) d;
  float a = fabs( la->slope ); //sign of slopes does not matter (order of points picked )
  float b = fabs(lb->slope);
  
  if( a <  b ) { return -1;}
  else if( a == b ) { return 0;}
  else if( a > b ) { return 1;}

  printf("compare returning illegal value\n");
  return(0);
}

int compare_ints( const void *a, const void *b){
  if( *(short*)a < *(short*)b ){ return -1;} 
  else if( *(short*)a == *(short*)b ){ return 0;}
  else return 1;
}



int get_median_index( short* idxs, short idxs_length){
  qsort(idxs, (size_t) idxs_length, sizeof(short), compare_ints);
  return(idxs[idxs_length/2]);
}

/*
  Take n Ransac models, sort them by generated slope and take the median value, hopefully reduces noise
 *//*
Line median_RANSAC( Laser * laser, short* laser_points, short laser_count, float threshold, int num_samples, void* landscape){
  Line *line_array = new Line [num_samples];
  Landscape* scape = (Landscape*) landscape;

  for( int i = 0 ; i < num_samples ; i++ ){
    line_array[i] = find_features(laser, laser_points, laser_count, threshold, (void *) scape);
  }
  qsort(line_array, (size_t) num_samples, sizeof(Line), compare_lines);
  delete[] line_array;
  return(line_array[num_samples/2]);
}

//turn perpendicular to the found wall
void turn_to_wall( Robot* robot,  int mode ){
  long* state = robot->state;
  Laser * laser = robot->laser;
  Line wall = robot->goal_feature;
  float goal_orientation;
  float angular_velocity = 1;

  goal_orientation = laser->angle(wall.perpendicular_laser_index);
  goal_orientation = mode == -1 ? goal_orientation + M_PI/2 : goal_orientation - M_PI/2; 
  goal_orientation *= 1000;
  goal_orientation += state[STATE_T];
  //first, turn perpendicular to the found wall
  while( fabs(angular_velocity) > 0){
    angular_velocity = rotate(state, goal_orientation, angular_velocity);
    vm(0,angular_velocity);
    //printf("angVel : %f\n",angular_velocity);
    usleep( 100000 );
  }
  return;
}

/* 
do either left or right wall follow. mode = -1 for right (CW) 1 for left (CCW) and zero for find the largest wall

return 1 for an external corner and -1 for an internal corner
 */
/*
int wall_follow( Robot* robot, int mode, float desired_distance, float desired_lin_vel){
  
  long* state = robot->state;
  Laser*laser = robot->laser;

  float prev_distance, curr_distance; //d0, zk, zk+1
  float prev_lin_vel, prev_ang_vel, velocity_constraint; //vk, vd, alpha
  float linear_velocity, angular_velocity;
  float k,B0,B1,c; //constant terms for angle correction
  float threshold = 3500; //only care about walls within 3500 mm
  float term1,term2; //for ease of computation
  float angle_from_wall;
  Line wall;
  short laser_count = laser->count();
  short * laser_points = new short [laser_count];
  short num_median = 5;
  short * idxs = new short [num_median];
  int direction = 1;
  int new_model_counter = 0;
  int refresh_distance = 0;
  int prev_perpendicular_index;
  int ret = 0; //return value
  float external_check;
  int perp_idx;
  perp_idx = mode < 0 ? 115 : 570;
  external_check = laser->value(perp_idx);


  Landscape * scape = new Landscape(laser);

  k = 10.0; //nominal is .25 from paper
  c = .07;
  B0 = 2.5;//1.25;
  B1 = c*B0;
  prev_lin_vel = 1;
  prev_ang_vel = 1;
 
  laser->update();
  /*initially find the wall on the indicated side, set initial_distance, curr_distance
   *//*
  switch( mode ){
  case -1:
    laser_count /= 2;
      //make an array of the laser points corresponding the the right side (0,340)
    for(int i = 0; i < laser_count ; i++){
      laser_points[i] = i;
    }
    prev_perpendicular_index = 114;
    break;
  case 1:
    laser_count /= 2;
    for(int i = 0 ; i < laser_count ; i++){ //laser points corresponding to left
      laser_points[i] = i + laser_count;
    }
    prev_perpendicular_index = 586;
    break;
  case 0:
    for( int i = 0 ; i < laser_count ; i++ ){ //all laser points
      laser_points[i] = i;
      }
    break;
  default:
    mode = 0;
  }
  
  //get the wall corresponding to the laser points
  wall = median_RANSAC( laser, laser_points, laser_count, threshold, 5,scape);
  //rotate to wall in the correction direction
  //turn_to_wall(state,laser,wall,mode);

  //set the standard difference so you can check for external corners
  external_check = laser->value(wall.perpendicular_laser_index);

  while( isClear(state,laser,direction) >= 0){ //until an internal corner is found
     //look for the new wall model once per second 
    if( new_model_counter % 1000000/SLEEP_TIME == 0 ){
      wall = median_RANSAC( laser, laser_points, laser_count, threshold, 5,scape);
    }
    new_model_counter += 1;
   
    //change current distance reading every .3 seconds, get the median of 5 sets of laser data
    if( refresh_distance % 3 == 0 ){
      //get the median of 5 laser readings
      for( int i = 0 ; i < num_median ; i++){
	curr_distance = 5600;
	laser->update();
	for(int j = wall.start_idx ; j < wall.end_idx ; j++ ){
	  curr_distance = laser->value(j) < curr_distance ? laser->value(j) : curr_distance;
	  idxs[i]= j;
	}
      }
      wall.perpendicular_laser_index = get_median_index( idxs, num_median);

      //to prevent random jumps, if the new perpendicular index is more than 30 away from the old one ignore it
      if( fabs( wall.perpendicular_laser_index - prev_perpendicular_index ) > 30 ){
	//	printf("wall perp : prev perp | %d : %d\n", wall.perpendicular_laser_index, prev_perpendicular_index);
	wall.perpendicular_laser_index = prev_perpendicular_index;
	//	printf( "Ignored perpendicular Index\n");
      }
      else{ //otherwise store this index
	prev_perpendicular_index = wall.perpendicular_laser_index;
      }
      
      //average the 3 values around the perpendicular index
      curr_distance += (laser->value(wall.perpendicular_laser_index-1) + laser->value(wall.perpendicular_laser_index+1));
      curr_distance /= 3;
    } 
    refresh_distance += 1;
   

    
    //if theres a huge difference between the current and last perpendicular distance say its an external corner
    /*if( laser->value(perp_idx) - external_check > 2000 && external_check != 0 ){ 
      printf("External Corner Detected with value (new : old) - %f : %f  \n", laser->value(perp_idx), external_check);
      ret = 1;
      break;
    }
    else{ external_check = laser->value(perp_idx); }
    */
    //check if the wall model ends
 /*   if( (mode == -1 && wall.end_idx < 100) || (mode == 1 && wall.end_idx > 580) ){
      printf("perpendicular laser idx : %d\nBetween %d and %d\n", wall.perpendicular_laser_index,wall.start_idx, wall.end_idx);      
      ret = 1;
      break;
    } 


    /* calculate the linear and angular velocity given the equations from class */
    //linear velocity = velocity_constraint * v_desired
/*    velocity_constraint = 1 < fabs(MAX_LINEAR_VEL/prev_lin_vel) ? 1 : fabs(MAX_LINEAR_VEL/prev_lin_vel);
    velocity_constraint = fabs(MAX_ANGULAR_VEL/prev_ang_vel) < velocity_constraint ? fabs(MAX_ANGULAR_VEL/prev_ang_vel) : velocity_constraint;
    
    //printf("vel_constraint, desired_vel : ( %f , %f )\n", velocity_constraint, desired_lin_vel);
    linear_velocity = velocity_constraint*desired_lin_vel;
    
    //angular Vel
    angle_from_wall = M_PI/2 + (-mode*laser->angle(wall.perpendicular_laser_index));
    term1 = mode*( k*(curr_distance - desired_distance) / (desired_lin_vel * cos( angle_from_wall )));   
    term2 = -1*mode*( (B0 + B1*abs(curr_distance - desired_distance) ) * tan( angle_from_wall  ) );
    //printf("Angle From Wall : %f\n", angle_from_wall);
    //printf("term1 : %f\n",term1);    
    //printf("term2 : %f\n",term2);    
    angular_velocity = velocity_constraint*10*(term1 + term2);
    //printf("angVel : %f\n", angular_velocity);
    

    //velocity caps
    linear_velocity = linear_velocity > MAX_LINEAR_VEL ? MAX_LINEAR_VEL : linear_velocity;
    linear_velocity = linear_velocity < -1*MAX_LINEAR_VEL ? -1*MAX_LINEAR_VEL : linear_velocity;
    angular_velocity = angular_velocity > MAX_ANGULAR_VEL ? MAX_ANGULAR_VEL : angular_velocity;
    angular_velocity = angular_velocity < -1*MAX_ANGULAR_VEL ? -1*MAX_ANGULAR_VEL : angular_velocity;
    vm(linear_velocity, angular_velocity);
    robot->last_linear_vel = linear_velocity;
    robot->last_angular_vel = angular_velocity;

    prev_lin_vel = linear_velocity;
    prev_ang_vel = angular_velocity;
    prev_distance = curr_distance;

    direction = linear_velocity < 0 ? -1 : 1;
    
    //printf("Last Vel Command (lin,ang) : ( %f , %f )\n", linear_velocity, angular_velocity);
    //printf("Current Distance From Wall : ( %f )\n", curr_distance);

    usleep(SLEEP_TIME);
  }
  //obstacle detected! assume its an internal corner, emergency stop and return -1
  emergency_stop(linear_velocity,1);
  ret = ret == 1 ? ret : -1;
  delete[] idxs;
  delete[] laser_points;

  return(ret);
}
*/
//function to get the appropriate linear and angular velocities
//when going to a given x, y coordinate
void gotoxy(Robot* rbt, int x, int y){
	
	long x0, y0, dx, dy;
	double theta;
	double alpha, p;
	long theta2;
	long dist;
	long goalAngle;
	long *state = rbt->state;
	
	//get initial and desired final locations
	x0 = state[STATE_X];
	y0 = state[STATE_Y];
	dx = x - x0;
	dy = y - y0;

	//calculate the distance to travel
	dist = sqrt(dx*dx + dy*dy);
	
	//determine if the point has been reached
	if (abs(dist) < 50){
		rbt->success = 1;
	}
	
	//calculate the angle
	theta = atan2(dy, dx);
	theta *= 1000;
	theta2 = state[STATE_T] > M_PI * 1000 ? state[STATE_T] - 2000 * M_PI: state[STATE_T];
	goalAngle = theta - theta2;
	
	//convert the angle
	goalAngle = goalAngle < -M_PI * 1000 ? goalAngle + M_PI * 2000: goalAngle;
	goalAngle = goalAngle > M_PI * 1000 ? goalAngle - M_PI * 2000: goalAngle;
	
	//calculate scale factor
	goalAngle = goalAngle / 2;
	if (goalAngle != 0)
		p = .1 / goalAngle > 1 ? 1 : 100.0 / goalAngle;
	else
		p = 1;
	
	//scale the speed
	dist = dist * fabs(p);
	
	//calculate alpha
	if (goalAngle != 0 && dist != 0){ 
		alpha = fabs(700.0 / goalAngle) < fabs(400.0 / dist) ? 700.0 / goalAngle : 400.0 / dist;
		alpha = 1 < fabs(alpha) ? 1 : alpha; 
	}
	else
		alpha = 1;
	
	//goalAngle = ((long(M_PI * 2000) + (long) theta) - state[STATE_T]) % ((long)((M_PI / 2) * 1000));
	//goalAngle = ((long) theta) - state[STATE_T] < 0 ? -1 * (1000 * M_PI - goalAngle) : goalAngle;
	
	//scale by alpha and assign to robot
	rbt->toTravel = dist * fabs(alpha) * 2;
	rbt->toTurn = goalAngle * alpha * 2;
	
	return;
}

/* connect to and set up laser, return a pointer to the laser */
Laser* laser_setup(void){
  Laser *las;
  las = new Laser();
  //create a new Laser object and connect to the device
 if( las->connect() ) {
   printf("Unable to connect to laser, make sure player is running\n");
   delete las;
   exit(-1);
  }
 return las;
}

//disconnect and delete the laser object
void laser_disconnect(Laser* ls){
  ls->disconnect();
  delete ls;
}



