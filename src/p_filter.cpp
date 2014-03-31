//p_filter.cpp
//Joseph Harwood
//04/18/13

#include "p_filter.h"
#include "Particle.h"


//constructor for a particle filter
P_filter::P_filter(Robot *r, PPMImage* map, float** evidence_grid, int n){

	//seed random
	srand(time(NULL));
	
	//create an arraylist to hold the particles and store the bot and map pointers
	this->pts = new ArrayList<Particle *>();
	this->topTen = new Particle *[10];
	this->bot = r;
	this->map = map;
	this->evidence_grid = evidence_grid;
	
	//initialize the timestep counter
	clock_gettime(CLOCK_REALTIME, &this->time_c);
	this->accum = 0;
	
	//create new particles
	for (int i = 0; i < n; i++){
	  this->pts->add(new Particle(map,evidence_grid));
	}

}

//////// The 3 main methods/phases //////////////
//method to predict the movement
void P_filter::predict(){
	
	//init
	struct timespec curTime;
	//double accum;
	
	//get the current time and initialize p
	clock_gettime(CLOCK_REALTIME, &curTime);
	
	//calculate the difference in time
	accum = (curTime.tv_sec - this->time_c.tv_sec) + (double) (curTime.tv_nsec - this->time_c.tv_nsec) / 1000000000;

	//loop through the particles currently in the system
	for (int i = 0; i < pts->getSize(); i++){
		
		//move the given particle by the distance suggested by physics and a random guaussian amount
		pts->get(i)->update_position(this->bot->last_linear_vel, this->bot->last_angular_vel, accum);
	}
	
	//reset the last prediction time
	clock_gettime(CLOCK_REALTIME, &this->time_c);

}

//method to correct the movement
void P_filter::correct(){

  //loop through the particles currently in the system
  for (int i = 0; i < pts->getSize(); i++){
    //update each particles feature list
    pts->get(i)->update_features(this->bot->laser);
    cout << "after update_features" << endl;
    //update the weight
    pts->get(i)->calculate_probability(this->bot->laser);
    cout << "after calc_prob" << endl;
  }

}

//Method to choose n new particles based on the weights
//the percentage of particles that should be newly created is given by r
void P_filter::resample(int n, float r){
	
	//m is the number of new particles
	int m = (int) n * r;
	
	//n is the number of old particles
	n = n - m;
	
	//loop through the particles to find the sum of the weights
	double sum = 0;
	

	for (int i = 0; i < pts->getSize(); i++){
		sum += pts->get(i)->get_probability();
		//		cout << "resample - probability " << pts->get(i)->get_probability() << endl;
	}
	
	//calculate a random number between 0 and 1
	double random = rand() / (double)RAND_MAX;
	 
	//modify the random number so that its max is the sum of the probabilities and then divide it so that it's <= sum / n (comb method)
	random = random * sum / (n + m);
	//create a new set of particles and normalize the weights
	ArrayList<Particle *> *temp = new ArrayList<Particle *>();
	Particle *p;
	double cSum = 0;
	int resampled = 0;
	
	//grab a resampling of old particles
	for (int i = 0; i < (n + m); i++){
	  p = pts->get(i);

	  //add the current sum of weights to the current particle's weight
	  cSum += p->get_probability();
	  //choose the first particle that makes the cumulative sum greater than the random number
	  while (cSum > random){ //put this as a while so the same particle will be picked over and over when it occupies a large part of the probability space
	    //get the new particle add it to a temporary list and normalize the weight
	    cout << "before add" << endl;
	    temp->add(p->copy_particle());
	    cout << "after add" << endl;
	    //move to the next "random" number
	    random += sum / n;
	    //cout << "resample - point added, random incremented to : " << random << endl;
	    resampled += 1;
	  }
  }
  
  //add to the sample new particles where 
  for (int i = 0; i < m; i++){
	  	
    temp->add(new Particle(this->map,this->evidence_grid));
	  	resampled += 1;
	  	
	}
	
  cout << "resampled: " << n<< " " << m << " " << resampled << endl;
	
	//delete the old list and store the new one
	delete this->pts;
    
	this->pts = temp;
	return;
}

//create a map with red dots at the locations of the particles
void P_filter::display_particles(PPMImage *red_map){
  
  //init locals
  int x_mark,y_mark;
  Particle *particle;
  
  //loop through all the particles
  for(int i = 0; i < this->pts->getSize(); i++){
  	
  	//store the current particle
    particle = this->pts->get(i);
    
    //get the (x, y) locations
    //(x, y) must be cast to int so that they allign with grid spaces 
    x_mark = (int) particle->get_pose()->x/GRID_RESOLUTION;
    y_mark = (int) particle->get_pose()->y/GRID_RESOLUTION;
    
    //add a red dot to the given image map
    addRedDot( x_mark,y_mark, red_map);
  }
  
  return;
}


//return the particles with the highest probability
void P_filter::updateTopTen(){
	//loop through all particles and determine the highest 10 probabilities
	for (int i = 0; i < this->pts->getSize(); i++){
		for (int j = 0; j < 10; j++){
			if (i < 10){
				this->topTen[j] = this->pts->get(i);
				continue;
			}
			else if (this->pts->get(i)->get_probability() > this->topTen[j]->get_probability()){
			  //cout << "updated" << endl;
				this->topTen[j] = this->pts->get(i);
				break;
			}
		} 
	}
	return;
}

void P_filter::update_evidence(){
  cout << "updating evidence" << endl;
  Particle *particle;
  Feature *feature;
  ArrayList<Feature*> *features;
  int feature_x, feature_y; 
  double x_dev,y_dev;
  double eig_x_1,eig_x_2,eig_y_1,eig_y_2;
  double eig_norm, eig_term;

  this->updateTopTen(); //grab the top ten particles
  
  //for each of the top ten particles, loop through their feature list and update the feature location and space around it due to covariance
  for( int i = 0 ; i < 10 ; i++ ){
    particle = this->topTen[i];
    features = particle->getFeatures();
    
    //for each of that particles features
    for( int j = 0; j < features->getSize() ; j++ ){
      //      cout << "update_evidence - got " << j << endl;
      feature = features->get(j);
      feature_x = feature->x;
      feature_y = feature->y;
      x_dev = feature->x_dev;
      y_dev = feature->y_dev;
      

      //find the eigenvectors of the covariance matrix and use them along with the eigenvalues to update the probability
      eig_term = (x_dev - feature->c11) / feature->c12;
      eig_norm = sqrt( 1 + ( eig_term*eig_term) ); 
      eig_x_1 = 1 / eig_norm;
      eig_y_1 = eig_term/eig_norm;
      
      eig_term = (y_dev - feature->c11) / feature->c12;
      eig_norm = sqrt( 1 + ( eig_term*eig_term) ); 
      eig_x_2 = 1 / eig_norm;
      eig_y_2 = eig_term/eig_norm;

      evidence_grid[feature_y][feature_x] += .05;

      //TODO: use eigenvectors and eigenvalues to update the probabilies around features
      //boost all the poitns around the feature by .05
      for( int k = 0 ; k < 3 ; k++){
	  for( int m = 0; m < 3 ; m++){
	    feature_y += m;
	    feature_x += k;
	    if( feature_y < this->map->y && feature_x < this->map->x ){
	      evidence_grid[feature_y][feature_x] += .05;
	    }
	  }
	}
     
      
      /* 
	 feature_x -= x_dev;
	 
	 while( feature_x < (feature->x + x_dev) ){
	 evidence_grid[feature_y][feature_x] += .1;
	 feature_x += 1;
	 }
	 feature_x = feature->x;
	 feature_y -= y_dev;
	 while( feature_y < (feature->y - y_dev) ){
	 evidence_grid[feature_y][feature_x] += .1;
	 feature_y += 1;
	 }
      */  
      evidence_grid[feature_y][feature_x] = evidence_grid[feature_y][feature_x] > .99 ? .99 : evidence_grid[feature_y][feature_x];
    evidence_grid[feature_y][feature_x] = evidence_grid[feature_y][feature_x] < .01 ? .01 : evidence_grid[feature_y][feature_x];
    }
  }
}

/*
void P_filter::update_evidence(){
  Laser* laser = this->bot->laser;

  this->updateTopTen();
  
  float robot_orientation, laser_angle, global_orientation;
  float x_laser,y_laser,h,slope,b;
  float map_distance;  //to hold the true distance of an obstacle for probability calculations
  int temp_x, temp_y;  //discrete x,y points to follow the line of the laser and check occupancy grid spaces
  int laser_idx;
  int laser_count = 0;
  int pose_x_grid = 0;
  int pose_y_grid = 0;
  float probability;
  Pose* pose;
  Particle* particle;
  laser_count = laser->count();

  for( int i = 0 ; i < 10 ; i++ ){
    particle = this->topTen[i];
    
    //particle->print_pose();

    pose = particle->get_pose();
    pose_x_grid = (int) pose->x/GRID_RESOLUTION; //for grid distance measuring 
    pose_y_grid = (int) pose->y/GRID_RESOLUTION;

    probability = 0; //reset the probability
    // the bots (x,y) is in grid coords, so we should be able to calculate which grid space a laser reading is 
    //     hitting based on the current orientation of the bot and the offset of the laser reading. 
    //given the angle of the robot and the laser reading, calculate the x,y coords of the laser reading.
    
    for (int j = 0; j < NUM_SENSOR_READINGS; j++) {
      //calculate the x,y coords of the obtained laser reading
      laser_idx = (int) (i)*laser_count/NUM_SENSOR_READINGS; //grab regularly spaced readings (TODO: better if random?)

      //calculate the global angle based on the robots orientation and the angle of the laser reading. 
      laser_angle = laser->angle(laser_idx);
      robot_orientation = pose->theta;
      global_orientation = robot_orientation + laser_angle;
      
      //calculate the x and y offsets of the laser reading
      h = laser->value(laser_idx);
      x_laser = h*cos(global_orientation);
      y_laser = -1*h*sin(global_orientation);// negative one since y increases down
      
      //make sure x_laser is never 0
      x_laser = x_laser == 0 ? 1 : x_laser;
      
      //once (x_laser,y_laser) are known find the slope of the line and check every box along that until an obstacle is found
      //or a maximum distance is reached
      slope = y_laser/x_laser;
      b = pose_y_grid - slope*pose_x_grid;
      
      //represent the start point as a discrete grid point for ease of computation. Since the x,y coords are continuous,
      //divide by the grid resolution to get the map coords (space occupied) of the robot 
      temp_x =  pose_x_grid;
      temp_y =  pose_y_grid;
      
      while( temp_x >= 0 && temp_x <= MAX_MAP_WIDTH  ) { //follow the line to the edge of the box 
				//TODO: deal with case when x_laser == pose.x
				if( x_laser < 0 ){ //if the line is from right to left
					temp_x -= 1;
				}
				else if( x_laser > 0 ){ //if the line is from left to right
					temp_x += 1;
				}
				temp_y = slope*temp_x + b; //get the y from the x given the slope of the line

				//restrict the temp coords to the map
				temp_y = temp_y < 0 ? 0 : temp_y;
				temp_y = temp_y > MAX_MAP_HEIGHT ? MAX_MAP_HEIGHT : temp_y;
	
				//cout << "pose (x,y) : temp (x,y) - ( "<<pose_x_grid<<" , "<<pose_y_grid<<" ) : ( "<<temp_x<<" , "<<temp_y<<" )"<<endl;

				//record the apparent distance the laser reading SHOULD have returned given current map understanding
				map_distance = sqrt( (pose_x_grid - temp_x)*(pose_x_grid- temp_x) + (pose_y_grid-temp_y)*(pose_y_grid-temp_y) );      
				map_distance *= GRID_RESOLUTION; //multiply by grid resolution to compare with laser readings

				//stop at the first occupied grid space or at the end of the lasers range
				if( (temp_y <= 0 || temp_y >= MAX_MAP_HEIGHT) ){ //if the y coord of the box is beyond the edge of the grid, break
					break;
				}
				//otherwise fill in the evidence grid with the probabilities
				else{
					if( map_distance >= h + 100 ){ break; } //we can't say anything useful about distances much further than the object
					evidence_grid[temp_y][temp_x] *= particle->calc_gauss(map_distance,h,2.0);
					
					//cout << "Laser Distance/Map Distance : " << h << " : " << map_distance << " Probability : " << particle->calc_gauss(map_distance,h) << endl;

						//cout << "update_evidence : " << temp_x << " , " << temp_y << endl;
					evidence_grid[temp_y][temp_x] = evidence_grid[temp_y][temp_x] > .99 ? .99 : evidence_grid[temp_y][temp_x];
					evidence_grid[temp_y][temp_x] = evidence_grid[temp_y][temp_x] < .01 ? .01 : evidence_grid[temp_y][temp_x];
				}
      }
    }
  }
}

*/
  
  ///////////End the main phases////////////////////
