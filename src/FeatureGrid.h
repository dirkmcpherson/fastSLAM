#ifndef featuregrid
#define featuregrid
#include "Particle.h"


class FeatureGrid{

	private Feature[] grid;
	
	public FeatureGrid(int height, int width);
	public setFeature(int row, int col);
	public removeFeature(int row, int col);
	public getFeature(int row, int col);
	

}

#endif
