#ifndef Landscape_h
#define Landscape_h
#include "motion_utilities.h"
//#include "ArrayList.cpp"

class Landscape{

	private:
		ArrayList<Line> *lines;
		Line last;
		Line closest;
		Laser* lsr;
		
	public:
		Landscape(Laser* lsr);
		~Landscape();
		void addLine(Line l);
		void clear();
		Line lastLine();
		Line closestLine();
		Line getSimilarLine(Line l);

};

#endif
