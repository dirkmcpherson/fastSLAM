
#include <iostream>
#include "Landscape.h"

using namespace std;

//constructor for LandScape class
Landscape::Landscape(Laser* lsr){
	this->lines = new ArrayList<Line>();
	this->lsr = lsr;
}

Landscape::~Landscape(){
	delete this->lines;
}

//add a line to the landscape
void Landscape::addLine(Line l){
	this->lines->add(l);
	if (lsr->value(l.perpendicular_laser_index) < lsr->value(this->closest.perpendicular_laser_index)){
		this->closest = l;
	}
}

//clear the landscape
void Landscape::clear(){
	this->lines->clear();
}

//remember the last similar line
Line Landscape::lastLine(){
	return last;
}

//get the shortest line in the scape
Line Landscape::closestLine(){
	return this->closest;
}


//gets the most similar line in terms of length
Line Landscape::getSimilarLine(Line l){
	
	//the best match
	Line best;
	
	//special case when there are no lines
	if (this->lines->getSize() == 0){
		cout << "There are no lines in the landscape." << endl;
	}
	
	//find the best
	best = this->lines->get(0);
	for (int i = 1; i < this->lines->getSize(); i++){
		if (fabs(this->lines->get(i).length - l.length) < fabs(best.length - l.length)){
			best = this->lines->get(i);
		}
	}
	
	this->last = best;
	
	//	cout << "best: " << best.length << endl;
	return best;
}
