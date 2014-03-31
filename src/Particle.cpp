//
//  Particle.cpp
//  
//
//  Created by James Staley on 4/15/13.
//
//

#include "Particle.h"

//Particle construrctor
Particle::Particle( PPMImage* m, float**e){
  map = m;
  evidence_grid = e;
  map_width = m->x - 1;
  map_height = m->y - 1;
  probability = 0;
  slow_average = 0;
  fast_average = 0;
  count = 1; //so the features wont be cleaned up for CLEAN_UP_COUNT updates
  features = new ArrayList<Feature *>();
  toRemove = new ArrayList<Feature *>();
  //set_pose(6000,9500,0);
  randomize(); //have the particle randomize its location upon creation?t
  set_pose(MAX_MAP_WIDTH*GRID_RESOLUTION/2, MAX_MAP_HEIGHT*GRID_RESOLUTION/2 , pose.theta); //keep a random orienation but set all the particles to be in the middle of the 'map' to start

  newParticle += 1;

}

//Particle destructor
Particle::~Particle(){
  //cout << "Memory - Deleting Particle features" << endl;
  //cout << "MemoryFeatures - Features Deleted : " << features->getSize() << endl;
  newFeature -= features->getSize();
  delete features;
  //cout << "Memory - Deleting Particle toRemove" << endl;
  toRemove->clear();
  delete toRemove;
  newParticle -= 1;
  //cout << "Memory - Particle Deleted" << endl;

  return;
  }

//static variable declaration
float Particle::l = 180; //radius of robot (mm)
float Particle::r = 30; //radius of wheel (mm)
int Particle::num_sensor_readings = 10;

//randomize the particles by x,y (grid resolution) and orientation (mRad)
void Particle::randomize(void){
  pose.x =  (rand()%map_width) * GRID_RESOLUTION; //multiply by resolution to switch from grid to mm
  pose.y =  (rand()%map_height) * GRID_RESOLUTION;
  pose.theta = rand()%(int)(1000*2*M_PI); //any theta is acceptable
}

//update the particles position based on the last linear and angular velocity commands sent and the timestep
void Particle::update_position( float lin_vel, float ang_vel, double timestep){
  //multiply the current pose by the translation matrix and the rotation matrix
  float l_phi, r_phi;
  float x_vel, y_vel, theta_vel;
  
  //wheel velocities are mm/s -> mrad/s (2pi*linear_vel/pi*D) +- half of the angular velocity
  l_phi = lin_vel/r - ang_vel/2;
  r_phi = lin_vel/r + ang_vel/2;
    
  //get the x,y,and theta velocities by multiplying rotation matrix by transformation matrix
  x_vel = (r/2)*cos(pose.theta)*(r_phi + l_phi);
  y_vel = (r/2)*sin(pose.theta)*(r_phi + l_phi);
  theta_vel = r/(2*l) * (r_phi - l_phi);
    
  //the particles new pose is each component plus the instaneous velocity * the timestep + some random noise
  pose.x = x_vel == 0 ? pose.x : pose.x + x_vel*timestep + random_noise(LINEAR);
  pose.y = y_vel == 0 ? pose.y : pose.y + y_vel*timestep + random_noise(LINEAR);
  pose.theta += theta_vel*timestep + random_noise(ANGULAR);

  //cap the pose at th edges of the map
  pose.x = pose.x > map_width*GRID_RESOLUTION ? map_width*GRID_RESOLUTION : pose.x;
  pose.x = pose.x < 0 ? 0 : pose.x;

  pose.y = pose.y > map_height*GRID_RESOLUTION ? map_height*GRID_RESOLUTION : pose.y;
  pose.y = pose.y < 0 ? 0 : pose.y;

  // Does theta need to be capped?
  //  pose.theta = pose.theta > 1000*2*M_PI ? pose.theta - (1000*2*M_PI) : pose.theta;
  return;
}

//based on the sensor readings and the occupancy grid, recalculate the probability of seeing a given reading in a location
void Particle::calculate_probability(Laser* laser){
  float robot_orientation, laser_angle, global_orientation;
  float x_laser,y_laser,h;//,slope,b;
  //float map_distance;  //to hold the true distance of an obstacle for probability calculations
  double laser_grid_x, laser_grid_y;
  int laser_idx;
  int laser_count = 0;
  int pose_x_grid = (int) pose.x/GRID_RESOLUTION; //for grid distance measuring 
  int pose_y_grid = (int) pose.y/GRID_RESOLUTION;

  //variables for importance factor
  float normalization_factor = .75;//1.0;
  //Q matrix
  double Q11,Q12,Q21,Q22;
  //Inverted Q matrix
  double iQ11,iQ12,iQ21,iQ22;

  double new_prob; //keep track of the individual probabilities for debugging
  
  Feature *z_hat; //measurement prediction for laser reading
  int x_error, y_error; //the difference bettween the expected and returned measurement
  


  laser_count = laser->count();

  probability = 1.0; //reset the probability to accept whatever it is first multiplied by

  for (int i = 0; i < num_sensor_readings; i++) {
    //calculate the x,y coords of the obtained laser reading
     laser_idx = (int) (i)*laser_count/num_sensor_readings; //grab regularly spaced readings (TODO: better if random?)

    //calculate the global angle based on the robots orientation and the angle of the laser reading. 
    laser_angle = laser->angle(laser_idx);
    robot_orientation = pose.theta;
    global_orientation = robot_orientation + laser_angle;
  
    //calculate the x and y offsets of the laser reading
    h = laser->value(laser_idx);
    x_laser = h*cos(global_orientation);
    y_laser = -1*h*sin(global_orientation);// negative one since y increases down

    //make sure x_laser is never 0
    x_laser = x_laser == 0 ? 1 : x_laser;

    laser_grid_x = (  x_laser +  pose.x )/GRID_RESOLUTION;
    laser_grid_y = (  y_laser  + pose.y )/GRID_RESOLUTION;
   

    z_hat = measurement_prediction( pose_x_grid, pose_y_grid, (int) laser_grid_x, (int) laser_grid_y , global_orientation); 

    if( z_hat != NULL ){
      //print_feature( z_hat );
      x_error = fabs(laser_grid_x - z_hat->x_cont);//*1000;
      y_error = fabs(laser_grid_y - z_hat->y_cont);//*1000;
      Q11 = z_hat->Q11;
      Q12 = z_hat->Q12;
      Q21 = z_hat->Q21;
      Q22 = z_hat->Q22;

      iQ11 = z_hat->iQ11;
      iQ12 = z_hat->iQ12;
      iQ21 = z_hat->iQ21;
      iQ22 = z_hat->iQ22;

      //check all terms
      //cout << "Z : x , y - " << x_error << " , " << y_error << endl;
      //cout << "probability Q : " << endl << Q11 << " " << Q12 << endl << Q21 << " " << Q22 << endl;
      //cout << "iQ : " << endl << iQ11 << " " << iQ12 << endl << iQ21 << " " << iQ22 << endl;     

      //just ignore features
      if( Q11 == 1 && Q22 == 1 ){ continue; } //TODO: initialize Q in some way

      //weight update as given in Thrun book (expanded on paper)
      new_prob = normalization_factor*( 1 / sqrt( 2*M_PI*( Q11*Q22 - Q12*Q21 ) ) ) * exp( -.5 * ( (x_error*x_error)*iQ11 + (x_error*y_error)*(iQ21+iQ12) + (y_error*y_error)*iQ22) );
      //cout << "new prob" << new_prob << endl;
      //cap new prob
    }
    else{ //no feature found probability is a gaussian around the laser distance where the mean is MAX_LASER_DISTANCE TODO: THIS IS THE WRONG THING TO DO
      new_prob = calc_gauss(h, MAX_LASER_DISTANCE, 1.0);
      //cout << "new prob gauss : " << h << endl;
    }
    new_prob = new_prob < MIN_PROB ? MIN_PROB : new_prob;
    new_prob = new_prob > MAX_PROB ? MAX_PROB : new_prob;
    
    //cout <<  " probability : " << new_prob << endl;
    if( isnan( new_prob ) || !finite(new_prob) ){ probability *= MIN_PROB; } //temp to control for nans and infs
    else{ probability *= new_prob; }
    
  }  
  //cap total probability;
  probability = probability < MIN_PROB ? MIN_PROB : probability;
  probability = probability > .99 ? .99 : probability; 
  cout << "Final Prob : " << probability << endl;
  //update the averages
  update_average();
  return;
}


//return a pointer to any feature that lies in the way of the passed in endpoints of a line
Feature* Particle::measurement_prediction( int x0, int y0, int x1, int y1, int global_orientation ){
  Feature* feature = NULL; //return a null pointer if no feature is found
  Feature* temp;
  for( int i = 0 ; i < features->getSize() ; i++ ){
    temp = features->get(i);
    if( temp->distance >= MAX_LASER_DISTANCE ) {continue;} //break now that its sorted?

    //is the feature intersected by the line?
    if( featureCheck( x0, y0, temp->x,temp->y, x1, y1,  temp->x_dev ,temp->y_dev, global_orientation)){ //true if feature is intersected by line
      //if( featureCheck( temp->x, temp->y, x1, y1, feature->x_dev, feature->y_dev ) ){
	  feature = temp;
	  break;
    }

  }
  return feature;
}


//return the distance from a feature to the robot
float Particle::distance(Feature *f){
	
  //return sqrt(((f->x - this->pose.x / GRID_RESOLUTION )* (f->x - this->pose.x / GRID_RESOLUTION))+ ((f->y - this->pose.y / GRID_RESOLUTION) * (f->y - this->pose.y / GRID_RESOLUTION)));
  return sqrt(((f->x*GRID_RESOLUTION - this->pose.x )* (f->x*GRID_RESOLUTION - this->pose.x))+ ((f->y*GRID_RESOLUTION - this->pose.y) * (f->y*GRID_RESOLUTION - this->pose.y)));
	
}

//bubblesort the list
void Particle::sort(){
	
	int currentSwap = 0; //keep track of where the last swap is on the current iteration
	int lastSwap = features->getSize() - 1; //store the index where the last swap occurred -- initialize at this->size - 1
	Feature* temp;
	
	int swapped = 1; //swap until you go through one round with no change
       
	//calculate the current distances
	for (int i = 0; i < features->getSize(); i++){
		features->get(i)->distance = distance(features->get(i));
	}
	
	//bubble sort
	while (swapped){
	  swapped = 0; //set swapped to false
	  //	  cout << "lastSwap : " << lastSwap << endl;
		for (int i = 0; i < lastSwap; i++){
			if (features->get(i)->distance > features->get(i + 1)->distance){
				temp = features->get(i);
				features->set(features->get(i + 1), i);
				features->set(temp, i + 1);
				currentSwap = i;
				swapped = 1; //set swapped to true
			}
		}
		lastSwap = currentSwap;
	}
	//cout << "sort returning" << endl;		
}

/* Update the current set of features held based on the laser readings */
void Particle::update_features( Laser* laser){
  double robot_orientation, laser_angle, global_orientation;
  float x_laser,y_laser,h,slope,b;
  double feature_x, feature_y;
  int temp_x, temp_y;  //discrete x,y points to follow the line of the laser and check occupancy grid spaces
  int laser_idx,feature_state;
  int laser_count = 0;
  int pose_x_grid = (int) pose.x/GRID_RESOLUTION; //for grid distance measuring 
  int pose_y_grid = (int) pose.y/GRID_RESOLUTION; 

  laser_count = laser->count();

  //cout << "Memory - numFeatures: " << features->getSize() << endl;

  //if its been long enough, clean up the features list and resort features
  if( count%CLEAN_UP_COUNT == 0 && toRemove->getSize() > 0){
    //cout << "Clean : pre-size of features "<< features->getSize() << endl;
    features->remove( toRemove ); 
    //    print_features();
    //cout << "Clean: Features Removed : " << toRemove->getSize() << endl;
    newFeature -= toRemove->getSize();
    toRemove->clear();
    //cout << "Clean : pose-size of features "<< features->getSize() << endl;
    sort();
  }
  count += 1;

  //For each of num_sensor_readings check if a feature is found
  for( int i = 0 ; i < num_sensor_readings ; i++ ){
    laser_idx = (int) (i*laser_count)/num_sensor_readings;

    //calculate the global angle based on the robots orientation and the angle of the laser reading. 
    laser_angle = laser->angle(laser_idx);
    robot_orientation = pose.theta;
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
    

    temp_x = x_laser < 0 ? pose_x_grid - 1 : pose_x_grid + 1; //set the first square to look in 
    temp_y = slope*temp_x + b;

    //get the global grid space of the detected feature (grid space of particle + offset)
    //feature_x =  ( (int) x_laser / GRID_RESOLUTION) + pose_x_grid; 
    //feature_y =  ( (int) y_laser / GRID_RESOLUTION) + pose_y_grid;

    feature_x = (x_laser + pose.x) / GRID_RESOLUTION;
    feature_y = (y_laser + pose.y) /GRID_RESOLUTION;

    if( h >= MAX_LASER_DISTANCE ){ //check all features within range and add a strike to any the ray passes through
      add_strike( (int) feature_x, (int) feature_y , global_orientation);
      continue;
    }
    //implement this step using the covariance update equation given in the lecture notes 
    
    //    cout << "no seg fault before isFeature" << endl;
    //features are relative to 
    feature_state = isFeature(feature_x,feature_y, global_orientation);
    //    cout << "after isFeature" << endl;
  }
}

// in the case of a max laser reading, add a strike to any feature the ray passes through
void Particle::add_strike( int x, int y, double global_orientation){
  Feature* feature;
  for( int i = 0 ; i < features->getSize() ; i++ ){
    //cout << i << endl;
    feature = features->get(i);
    //since their sorted, once they're out of range we dont care
    if( feature->distance >= MAX_LASER_DISTANCE ){ continue; }
    
    if( featureCheck( (int) pose.x/GRID_RESOLUTION, (int) pose.y/GRID_RESOLUTION, feature->x, feature->y, x, y, feature->x_dev, feature->y_dev, global_orientation ) ){// the laser passed through an apparent feature
    //if( featureCheck( temp_x, temp_y, x, y, feature->x_dev, feature->y_dev ) ){  
      feature->strike += 1;
      //cout << "Strike: Dev  x,y : " << feature->x_dev << " , " << feature->y_dev << endl;
      //cout << "Strike: Diff x,y : " << fabs( feature->x - x ) << " , " << fabs( feature->y - y ) << endl;
     
      if( feature->strike == MAX_STRIKES ){ //if the laser passes through the feature 3 times set the feature to be removed
	toRemove->add( feature ); 
      }
    }
    
  }
}

/*Determine whether the indicated feature is in this particles feature list. Since the list is assumed to be sorted by x-val
  BinarySearch by x and then look for y. */
int Particle::isFeature( double x, double y , double global_orientation){
  int numFeatures = features->getSize();
  //  int searchIdx = numFeatures / 2;
  //need to find features within the std_deviation set in Particle.h (for now consider a 3x3 square with the feature in the middle) TODO: make this legit
  double diff_x, diff_y, minor_axis, major_axis;
  //double lower_x,upper_x,lower_y,upper_y;
  int num_sigma = 1; //how many sigma do go out in the deviation
  int temp_x,temp_y;
  Feature* feature;
  int feature_state;

  //variables for K calculation
  double cos_val, sin_val;
  double G11,G12,G21,G22; //values of jacobian matrix, from top left to bottom right
  double C11,C12,C21,C22; //values of covariance matrix from top left to bottom right
  double A,B,C,D;
  double Q11,Q12,Q21,Q22;
  double det,trace,iQ11,iQ12,iQ21,iQ22; // calculate the inverse of the matrix
  double K11,K12,K21,K22; //K matrix
  double x_error,y_error; // the difference between the expected and the measured

  //trying to incorporate probability into this in one fell swoop
  //Feature* z_hat;

  //base case for no features
  if( numFeatures == 0 ){
    add_feature( x, y );
  }

  //Start with super slow implementation, clean up to speed up later 
  for( int i = 0 ; i < numFeatures; i++ ){
    //    z_hat = NULL; //zhat is null unless it is a measured feature to generate probability
    feature = features->get(i);
    //    cout << "Feature distance : " << feature->distance << endl;
    //we don't care about looking at a feature if its out of range, skip them
    if( feature->distance >= MAX_LASER_DISTANCE ){ continue; } //break since features are sorted

    temp_x = feature->x;
    temp_y = feature->y;

    if( featureCheck( (int) pose.x/GRID_RESOLUTION, (int) pose.y/GRID_RESOLUTION, temp_x, temp_y,(int)  x,(int)  y, feature->x_dev, feature->y_dev, global_orientation ) ){// the laser passed through an apparent feature
      //if( featureCheck( temp_x, temp_y, x, y, feature->x_dev, feature->y_dev ) ){
      feature->strike += 1;
      //cout << "Strike: Dev  x,y : " << feature->x_dev << " , " << feature->y_dev << endl;
      //cout << "Strike: Diff x,y : " << fabs( feature->x - x ) << " , " << fabs( feature->y - y ) << endl;
      if( feature->strike == MAX_STRIKES ){ //if the laser passes through the feature 3 times set the feature to be removed
	toRemove->add( feature ); 
      }

    }

    global_orientation /= -1000; //convert to rads and rotate back to origin

    /* since the equation is for an ellipse at the origin alligned with cartesian axes, translate the elipse to the origin
       and rotate the point to the angle of the laser before translating by same amount as elipse so see if the point is within ellipse */

    temp_x = feature->x_cont*cos(global_orientation) - feature->y_cont*sin(global_orientation) - x;
    temp_y = feature->x_cont*sin(global_orientation) + feature->y_cont*cos(global_orientation - y);

    //set the avlues needed to check if the center of the laser reading lies within another features elipse
    minor_axis = num_sigma*feature->y_dev;
    major_axis = num_sigma*feature->x_dev;
    //zeros because of translation to origin
    diff_x = 0 - temp_x;
    diff_y = 0 - temp_y;
    minor_axis *= minor_axis;
    major_axis *= major_axis;
    diff_x *= diff_x;
    diff_y *= diff_y;

    // does the point lie within the bounding box of the ellipse of the feature given its covariances? TODO: so rough
    //lower_x = x - num_sigma*feature->x_dev;
    //upper_x = x + num_sigma*feature->x_dev;
    //lower_y = y - num_sigma*feature->y_dev;
    //upper_y = y + num_sigma*feature->y_dev;

    

    if( ( (diff_x / major_axis) + (diff_y / minor_axis) ) <= 1  ){ //check if x,y lie in elipse defined by feature TODO: This elipse is always aligned with the cartesian plane?
    //if( (lower_x <= temp_x && temp_x <= upper_x) && (lower_y <= temp_y && temp_y <= upper_y) ){
      feature_state = PROBABLE_FEATURE;
      //x,y error is difference between the x,y passed in and the x,y of the feature found (this is possibley > 0 because of deviation)
      x_error = fabs( x - feature->x_cont );
      y_error = fabs( y - feature->y_cont );

      //update the features covariance     
      sin_val = sin(global_orientation);
      cos_val = cos(global_orientation);

       //store G
      G11 = cos_val;
      G12 = -1*sin_val;
      G21 = sin_val;
      G22 = cos_val;

      //current covariance values
      C11 = feature->c11;
      C12 = feature->c12;
      C21 = feature->c21;
      C22 = feature->c22;

      //GT * Sigma
      A = G11*C11 + G21*C21;
      B = G11*C12 + G21*C22;
      C = G12*C11 + G22*C21;
      D = G12*C12 + G22*C22;

      //find matrix to be inverted - GT * sigma * G + R
      Q11 = A*G11 + B*G21 + SENSOR_COVARIANCE_X;
      Q12 = A*G12 + B*G22;
      Q21 = C*G11 + D*G21;
      Q22 = C*G12 + D*G22 + SENSOR_COVARIANCE_Y;      
      
      //invert the above matrix
      //calcualte the determinate, if its zero or very near zero just ignore it
      det  = Q11*Q22 - Q12*Q21;
      trace = Q11 + Q22; //trace for eigen calcualting
      if( det < 1e-6 ){ 
	continue;
	cout << "Determinate too small : " << det << endl;
      }

      //calculate the inverted matrix values
      iQ11 = 1/det * Q22;
      iQ12 = 1/det * -1 * Q12;
      iQ21 = 1/det * -1 * Q21;
      iQ22 = 1/det * Q11;

      //Resuse A-F to store values for sigma*G (values are not needed once Q is calculated )
      A = G11*C11 + G21*C12;
      B = G12*C11 + G22*C12;
      C = G11*C21 + G21*C22;
      D = G12*C21 + G22*C22;
     
      //Calculate K
      K11 =A*iQ11 + B*iQ21;
      K12 =A*iQ12 + B*iQ22;
      K21 =C*iQ11 + D*iQ21;
      K22 =C*iQ12 + D*iQ22;
      
      //Update the covariances : [I - KG^T] * sigma ( Repurpose A - D )
      //first [I - KG^T]
      A = 1 - (K11*G11 + K12*G12);
      B = -1*(K11*G21 + K12*G22);
      C = -1*(K21*G11 + K22*G12);
      D = 1 - (K21*G21 + K22*G22);
      
      //update the mean of the feature
      feature->x_cont +=  K11*x_error + K12*y_error;
      feature->y_cont +=  K21*x_error + K22*y_error;
      
      //make sure features stay on map
      feature->x_cont = feature->x_cont < 0 ? 0 : feature->x_cont;
      feature->x_cont = feature->x_cont > map_width ? map_width : feature->x_cont;
      feature->y_cont = feature->y_cont < 0 ? 0 : feature->y_cont;
      feature->y_cont = feature->y_cont > map_height ? map_height : feature->y_cont;

      feature->x = (int) feature->x_cont;
      feature->y = (int) feature->y_cont;

      //now update covariances
      feature->c11 = A*C11 + B*C21;
      feature->c12 = A*C12 + B*C22;
      feature->c21 = C*C11 + D*C21;
      feature->c22 = C*C11 + D*C22;

      //update the major and minor covariances of the feature by taking the eigenvalues of the covariance matrix
      feature->x_dev = ( trace / 2 ) + 1 / sqrt( (trace*trace) / 4.0 - det );
      feature->y_dev = ( trace / 2 ) - 1 / sqrt( (trace*trace) / 4.0 - det );

      //cout << "Major,Minor : " << feature->x_dev << " , " << feature->y_dev << endl;

      //finally, check if the feature has any strikes and remove one if it does
      feature->strike -= 1; // feature->strike > 0 ? feature->strike - 1 : 0;


      //TODO: This is a temporary fix so we don't have to recalculate q in update probability. CHANGE LATER, for real this is throwing another k on our n. 
      feature->Q11 = Q11;
      feature->Q12 = Q12;
      feature->Q21 = Q21;
      feature->Q22 = Q22;
      feature->iQ11 = iQ11;
      feature->iQ12 = iQ12;
      feature->iQ21 = iQ21;
      feature->iQ22 = iQ22;     

      //check all terms
      //cout << "Z : x , y - " << x_error << " , " << y_error << endl;
      //cout << "Q : " << endl << Q11 << " " << Q12 << endl << Q21 << " " << Q22 << endl;
      //cout << "iQ : " << endl << iQ11 << " " << iQ12 << endl << iQ21 << " " << iQ22 << endl;     


      return feature_state;
    }
  }
  add_feature(x,y);
  return feature_state;
}

//function to roughly check whether a 

//a method to detect bad feature representations through ray casts
bool Particle::featureCheck(int botx, int boty, int fx, int fy, int endx, int endy, int sig3x, int sig3y, int mrad){

	int dvx = endx - botx; //direction vector x
	int dvy = endy - boty; //direction vector y
	
	int fvx = botx - fx;   //feature vector x
	int fvy = boty - fy;	 //feature vector y
	
	int dotdd = dvx * dvx + dvy * dvy; //dvx dot dvx
	int dotfd = fvx * dvx + fvy * dvy; //fvx dot dvx
	int dotff = fvx * fvx + fvy * fvy; //fvy dot fvy
	
	float rad = mrad / 1000.0;
	float sig3 = sqrt(((cos(rad) * sig3x) * (cos(rad) * sig3x)) + ((sin(rad) * sig3y) * ((sin(rad) * sig3y))));
	
	//set up quadratic formuala
	int a = dotdd;
	int b = 2 * dotfd;
	float c = dotff - sig3 * sig3;
	
	//solve for the determinant
	float solve = b * b - 4 * a * c;
	
	//no intersect if a negative value is returned
	if (solve < 0){
		return false;
	}
	
	//decide if there is a through intersection
	float intersect = (-b - sqrt(solve)) / (2 * a);

	if (intersect >= 0 && intersect <= 1){
		return true;
	}
	else{
		return false;
	}	
}


//do we keep the running average here or in the Particle Filter class
void Particle::update_average(void){
  slow_average = (1 - SLOW_AVERAGE_CONSTANT)*slow_average + SLOW_AVERAGE_CONSTANT*(probability); //do these probabilities need to be some kind of average? 
  fast_average = (1 - FAST_AVERAGE_CONSTANT)*fast_average + FAST_AVERAGE_CONSTANT*(probability);
}

//calculate the probability of a particle being an accurate representation of the robot's state
float Particle::calc_gauss(float measured_distance, float map_distance, float peak_val){
  float probability;
  //gaussian variables
  float b = map_distance; //controls the center of the bell ( in mm ) set when map distance is known
  float c = 100.0; //std deviation of peak (sqrt(5) gives a 10 mm space)
  float a = peak_val;// 1.0/(c*sqrt(2*M_PI));//the height of the curves peak (1.0 for 100% certainty)
  
  float exponent = -1*( ( (measured_distance - b)*(measured_distance - b) ) / (2* (c*c) ) );
  probability = a*pow(M_E,exponent); //calc gauss
  
  //cap the probability between MIN_PROB
  probability = probability < MIN_PROB ? MIN_PROB : probability;
 	
 	//cout << probability << endl;
  return probability;
}

//create a gaussian random number to jitter the particles by
float Particle::random_noise(int mode){
  float noise,random,exponent;
  float a = mode > 0 ? 50 : GRID_RESOLUTION; //if providing angular noise 200mRad, otherwise half a grid space. 
  float b = 100; //make the center of the curve at 100
  float c = 18; //std deviation
  
  random = 2*b * rand()/(float) RAND_MAX; //generate a random number between 0 and 2*b
  
  //cout << "random_noise - random : " << random << endl;
  exponent = -1*( ( (random - b)*(random - b) ) / (2* (c*c) ) );
  noise = a*pow(M_E,exponent);

  //randomize whether to push noise in positive or negative direction
  random = rand() / (float) RAND_MAX;
  random = random < .5 ? -1 : 1;
  noise *= random;

  //cout << "random_noise - noise : " << noise << endl;
  return noise;
}

//copy the particle and return a pointer to the new copy
Particle* Particle::copy_particle(void){
  Particle* particle = new Particle(map,evidence_grid);
  Feature* temp,*feature;
  particle->set_pose( pose.x ,pose.y , pose.theta );
  particle->set_probability(probability );
  particle->set_average( slow_average , fast_average );
  particle->count = count;
  //copy feature list
  for( int i = 0 ; i < features->getSize() ; i++ ){
    temp = features->get(i);
    //reassign temp to the copied feature
    //print_feature( temp );
    feature = particle->copy_feature( temp );
    particle->getFeatures()->add( feature );
    //every time a feature is copied, check if its in line to be removed and add it to the new Particles remove list if it is. 
    if( feature->strike >= MAX_STRIKES ){
      particle->toRemove->add( feature );
    }
  }
  return particle;
}

void Particle::add_feature( double x , double y ){
  x = x < 0 ? 0 : x;
  x = x > map_width ? map_width : x;
  y = y < 0 ? 0 : y;
  y = y > map_height ? map_height : y;
  Feature* feature = new Feature;
  
  newFeature += 1;

  feature->x_cont = x;
  feature->y_cont = y;

  feature->x = (int) x;
  feature->y = (int) y;

  //initialize covariance to be circular about the feature
  feature->c11 = FEATURE_DEVIATION;
  feature->c12 = 0;
  feature->c21 = 0;
  feature->c22 = FEATURE_DEVIATION;
  
  //default importance weight
  feature->weight = DEFAULT_FEATURE_WEIGHT;

  //initialize distance from robot
  feature->distance = 0;

  feature->strike = 0;

  //TODO: get of rid of this BS
  feature->Q11 = 1;
  feature->Q12 = 0;
  feature->Q21 = 0;
  feature->Q22 = 1;
  feature->iQ11 = 1;
  feature->iQ12 = 0;
  feature->iQ21 = 0;
  feature->iQ22 = 1;

  features->add( feature );
}

//copy a feature from one particle to another 
Feature* Particle::copy_feature( Feature* f ){
  Feature* feature = new Feature;

  newFeature += 1;

  feature->x = f->x;
  feature->y = f->y;
  feature->x_cont = f->x_cont;
  feature->y_cont = f->y_cont;
  feature->x_dev = f->x_dev;
  feature->y_dev = f->y_dev;
  feature->weight = f->weight;
  feature->distance = f->distance;
  feature->strike = f->strike;

  //cout << "first block" << endl;

  feature->Q11 = f->Q11;
  feature->Q12 = f->Q12;
  feature->Q21 = f->Q21;
  feature->Q22 = f->Q22; 
  feature->iQ11 = f->iQ11;
  feature->iQ12 = f->iQ12;
  feature->iQ21 = f->iQ21;
  feature->iQ22 = f->iQ22; 

  //cout << "second block" << endl;

  feature->c11 = f->c11;
  feature->c12 = f->c12;
  feature->c21 = f->c21;
  feature->c22 = f->c22;

  //cout << "before add feature" << endl;
  return feature;
}

//returns the particles probability
float Particle::get_probability(void){
  return probability;
}

//returns averages TODO A fast and slow get average?
float Particle::get_average(void){
  return fast_average;
}

//return a pointer to the pose of the particle
Pose* Particle::get_pose(void){
  return &pose;
}

ArrayList<Feature*> *Particle::getFeatures(void){
  return features;
}

//set the pose of the particle
void Particle::set_pose(int x_new, int y_new, float theta_new){
  pose.x = x_new;
  pose.y = y_new;
  pose.theta = theta_new;
}

//set the probability of the particle
void Particle::set_probability( float p ){
  probability = p;
}

//set the slow and fast averages of the particle
void Particle::set_average(float new_slow , float new_fast){
  slow_average = new_slow;
  fast_average = new_fast;
}

//utility method for printing the pose of a particle
void Particle::print_pose(void){
  cout << "Pose (x,y,theta) : ( ";
  cout << pose.x << " , ";
  cout << pose.y << " , ";
  cout << pose.theta << " )" << endl;
}


//print out the x,y of every feature
void Particle::print_features(void){
  Feature *feature;
  for( int i = 0 ; i < features->getSize() ; i++ ){
    feature = features->get( i );
    cout << "print_features : " << feature->x << " , " << feature->y << endl;
  }
  cout << "Printed " << features->getSize() << endl;
}

void Particle::print_feature( Feature* feature ){
  cout << "Feature at : " << feature->x<< " , " << feature->y << endl;
  cout << "Deviation : "<< feature->x_dev << " , " << feature->y_dev << endl;
  cout << "Distance : " << feature->distance << endl;
  cout << "Strikes : " << feature->strike << endl;
  cout << "Q : " << endl << feature->Q11 << " " << feature->Q12 << endl << feature->Q21 << " " << feature->Q22 << endl;
  cout << "iQ : " << endl << feature->iQ11 << " " << feature->iQ12 << endl << feature->iQ21 << " " << feature->iQ22 << endl;     

};
