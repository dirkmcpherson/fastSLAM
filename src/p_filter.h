#include <ctime>
#include "Particle.h"

//class to represent a particle filter
class P_filter{

private:

	ArrayList<Particle *> *pts;
	Particle **topTen;
	Robot *bot;
	struct timespec time_c;
	PPMImage* map;
	float** evidence_grid;
	
public:
	double accum; 
	P_filter(Robot* r, PPMImage* map, float** evidence_grid, int n);
	void predict();
	void correct();
	void resample(int n, float r);
	void display_particles( PPMImage *red_map );
	void updateTopTen();
	void update_evidence();
	
};
