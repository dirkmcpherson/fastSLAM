//a method to detect bad feature representations through ray casts
bool featureCheck(int botx, int boty, int fx, int fy, int endx, int endy, int sig3){
	
	int dvx = endx - botx; //direction vector x
	int dvy = endy - boty; //direction vector y
	
	int fvx = botx - fx;   //feature vector x
	int fvy = boty - fy;	 //feature vector y
	
	int dotdd = dvx * dvx + dvy * dvy; //dvx dot dvx
	int dotfd = fvx * dvx + fvy * dvy; //fvx dot dvx
	int dotff = fvx * fvx + fvy * fvy; //fvy dot fvy
	
	//set up quadratic formuala
	int a = dotdd;
	int b = 2 * dotfd;
	int c = dotff - sig3 * sig3;
	
	//solve for the determinant
	int solve = b * b - 4 * a * c;
	
	//no intersect if a negative value is returned
	if (solve < 0){
		return false;
	}
	
	//decide if there is a through intersection
	int intersect = (-b - sqrt(solve)) / (2 * a);
	if (intersect >= 0 && intersect <= 1){
		return true;
	}
	else{
		return false;
	}
	
}
