/*pseudocode for writing red squares at particle locations on a given PPMImage*/
void display_particles( PPMImage *red_map ){
  for(int i = 0; i < num_particles; i++){
    addRedDot( (int) particle->x , (int) particle->y, red_map); //(x,y) must be cast to int so that they allign with grid spaces 
  }
  return;
}


/*Determine whether the indicated feature is in this particles feature list. Since the list is assumed to be sorted by x-val
  BinarySearch by x and then look for y. */
int Particle::isFeature( int x, int y , float global_orientation){
  int numFeatures = features->getSize();
  //  int searchIdx = numFeatures / 2;
  //need to find features within the std_deviation set in Particle.h (for now consider a 3x3 square with the feature in the middle) TODO: make this legit
  int diff_x, diff_y, minor_axis, major_axis;
  int num_sigma = 1; //how many sigma do go out in the deviation
  int temp_x,temp_y;
  Feature* feature;
  int feature_state;

  //variables for K calculation
  double cos_val, sin_val;
  double J11,J12,J13,J21,J22,J23; //values of jacobian matrix, from top left to bottom right
  double C1,C2; //values of covariance matrix from top left to bottom right
  double A,B,C,D,E,F,G,H,I; //values of matrix to be inverted from top left to bottom right
  double det,Ai,Bi,Ci,Di,Ei,Fi,Gi,Hi,Ii; // calculate the inverse of the matrix
  double K11,K12,K13,K21,K22,K23; //K matrix
  //base case for no features
  if( numFeatures == 0 ){
    return NEW_FEATURE;
  }

  //Start with super slow implementation, clean up to speed up later TODO: implement binary search
  for( int i = 0 ; i < numFeatures; i++ ){
    feature = features->get(i);
    temp_x = feature->x;
    temp_y = feature->y;

    if( featureCheck( (int) pose.x/GRID_RESOLUTION, (int) pose.y/GRID_RESOLUTION, temp_x, temp_y, x, y, feature->x_dev ) ){// the laser passed through an apparent feature
      feature->strike += 1;
      if( feature-> strike == 3 ){ //if the laser passes through the feature 3 times set the feature to be removed
	toRemove->add( feature ); 
      }
    }
    
    //set the avlues needed to check if the center of the laser reading lies within another features elipse
    minor_axis = num_sigma*feature->y_dev;
    major_axis = num_sigma*feature->x_dev;
    diff_x = x - temp_x;
    diff_y = y - temp_y;
    minor_axis *= minor_axis;
    major_axis *= major_axis;
    diff_x *= diff_x;
    diff_y *= diff_y;

    if( ( (diff_x / major_axis) + (diff_y / minor_axis) ) <= 1  ){ //check if x,y lie in elipse defined by feature TODO: This elipse is always aligned with the cartesian plane?
      feature_state = PROBABLE_FEATURE;
      //update the features covariance     
      global_orientation /= -1000; //convert to rads and rotate back to origin
      sin_val = sin(global_orientation);
      cos_val = cos(global_orientation);

       //store G
      J11 = cos_val;
      J12 = -1*sin_val;
      J13 = x*cos_val - y*sin_val;
      J21 = sin_val;
      J22 = cos_val;
      J23 = x*sin_val + y*cos_val;

      //current covariance values
      C1 = feature->x_dev;
      C2 = feature->y_dev;

      //calc matrix to be inverted
      A = J11*( J11*C1 + J21*C2 ) + J21*( J11*C2 + J21*C1 );
      B = J12*( J11*C1 + J21*C2 ) + J22*( J11*C2 + J21*C1 );
      C = J13*( J11*C1 + J21*C2 ) + J23*( J11*C2 + J21*C1 );

      D = J11*( J12*C1 + J22*C2 ) + J21*( J12*C2 + J22*C1 );
      E = J12*( J12*C1 + J22*C2 ) + J22*( J12*C2 + J22*C1 );
      F = J13*( J12*C1 + J22*C2 ) + J23*( J12*C2 + J22*C1 );
      
      G = J11*( J13*C1 + J23*C2 ) + J21*( J13*C2 + J23*C1 );
      H = J12*( J13*C1 + J23*C2 ) + J22*( J13*C2 + J23*C1 );
      I = J13*( J13*C1 + J23*C2 ) + J23*( J13*C2 + J23*C1 );
      
      //invert the above matrix
      //calcualte the determinate, if its zero or very near zero just ignore it
      det  = A*( E*I - F*H ) - B*(D*I - F*G) + C*( D*H - E*G );
      if( det < 1e-6 ){ 
	continue;
	cout << "Determinate too small : " << det << endl;
      }
      //calculate the inverted matrix values
      Ai = det*(E*I - F*H);
      Bi = det*(C*H - B*I);
      Ci = det*(B*F - C*E);
      Di = det*(F*G - D*I);
      Ei = det*(A*I - C*G);
      Fi = det*(C*D - A*F);
      Gi = det*(D*H - E*G);
      Hi = det*(B*G - A*H);
      Ii = det*(A*E - B*D);

      //Resuse A-F to store values for sigma*G (values are not needed once inverse is calculated )
      A = J11*C1 + J21*C2;
      B = J12*C1 + J22*C2;
      C = J13*C1 + J23*C2;
      D = J11*C2 + J21*C1;
      E = J12*C2 + J22*C1;
      F = J13*C2 + J23*C1;      

      //Calculate K
      K11 = A*Ai + B*Di + C*Gi;
      K12 = A*Bi + B*Ei + C*Hi;
      K13 = A*Ci + B*Fi + C*Ii;
      K21 = D*Ai + E*Di + F*Gi;
      K22 = D*Bi + E*Ei + F*Hi;
      K23 = D*Ci + E*Fi + F*Ii;

      //Update the covariances : [I - KG^T] * sigma ( Repurpose A - D )
      //first [I - KG^T]
      A = 1 - (K11*J11 + K12*J12 + K13*J13);
      B = -1*(K11*J21 + K12*J22 + K13*J23);
      C = 1 - (K21*J11 + K22*J12 + K23*J13);
      D = -1*(K21*J21 + K22*J22 + K23*J23);
      
      //now update covariances
      feature->x_dev = A*C1 + B*C2;
      feature->y_dev = C*C1 + D*C2;
      return feature_state;
    }
  }
  feature_state = NEW_FEATURE;
  return feature_state;
}


//based on the sensor readings and the occupancy grid, recalculate the probability of seeing a given reading in a location
void Particle::calculate_probability(Laser* laser){
  float robot_orientation, laser_angle, global_orientation;
  float x_laser,y_laser,h,slope,b;
  float map_distance;  //to hold the true distance of an obstacle for probability calculations
  int temp_x, temp_y;  //discrete x,y points to follow the line of the laser and check occupancy grid spaces
  int laser_idx;
  int laser_count = 0;
  int pose_x_grid = (int) pose.x/GRID_RESOLUTION; //for grid distance measuring 
  int pose_y_grid = (int) pose.y/GRID_RESOLUTION;

  laser_count = laser->count();

  probability = 0; //reset the probability
  /* the bots (x,y) is in grid coords, so we should be able to calculate which grid space a laser reading is 
     hitting based on the current orientation of the bot and the offset of the laser reading. */
  //given the angle of the robot and the laser reading, calculate the x,y coords of the laser reading.

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

    //once (x_laser,y_laser) are known find the slope of the line and check every box along that until an obstacle is found
    //or a maximum distance is reached
    slope = y_laser/x_laser;
    b = pose_y_grid - slope*pose_x_grid;
      
    /* represent the start point as a descrete grid point for ease of computation. Since the x,y coords are continuous,
       divide by the grid resolution to get the map coords (space occupied) of the robot */
    temp_x =  pose_x_grid;
    temp_y =  pose_y_grid;
    
    while( temp_x >= 0 && temp_x <= map_width  ) { //follow the line to the edge of the box 
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
      temp_y = temp_y > map_height ? map_height : temp_y;

      //record the apparent distance the laser reading SHOULD have returned given current map understanding
      map_distance = sqrt( (pose_x_grid - temp_x)*(pose_x_grid- temp_x) + (pose_y_grid-temp_y)*(pose_y_grid-temp_y) );      
      map_distance *= GRID_RESOLUTION; //multiply by grid resolution to compare with laser readings 

      //stop at the first occupied grid space as or at the end of the lasers range
      if( (temp_y <= 0 || temp_y >= MAX_MAP_HEIGHT) ){ //if the y coord of the box is beyond the edge of the grid, break
	break;
      }
      //      else if( isGray(temp_x,temp_y,map)  == 1 || (map_distance  >= MAX_LASER_DISTANCE) ){
      else if( isFeature(temp_x,temp_y,global_orientation) == PROBABLE_FEATURE  || (map_distance  >= MAX_LASER_DISTANCE) ){
	break;
      }
    }
    
    //once the true distance to the obstacle (or max reading) is found, use it to set the pdf and calculate part of the probablility of the reading
    if( probability == 0 ){ //probability will only be 0 if the particle hasn't yet produced any probabilities (gauss function is limited to 1e-6)
      probability = calc_gauss(map_distance,h,1.0);
    } 
    else{
      probability *= calc_gauss(map_distance,h,1.0); //otherwise multiple all the probabilities to together to generate a snapshot of the entire laser sample
    } 
    //probability += calc_gauss(map_distance,h,1.0)/num_sensor_readings;
  }
  
  //cap total probability;
  probability = probability < 1.0e-6 ? 1.0e-6 : probability;
  probability = probability > .99 ? .99 : probability; 
  //update the averages
  update_average();
  return;
}
