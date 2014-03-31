
#ifndef ArrayList__
#define ArrayList__
#include <iostream>
//#include "global.h"

#define START_SIZE 10

using namespace std;


extern int newArrayList;

//As of the final project this will only work with objects that have a probability field

//Simple array list class has methods to add to the
//end of the list, to get an element, to clear the list, 
//to get the size, and to print the elements in order
//parameterized on T
template <class T>
class ArrayList{

	private:
		
		T *arr;
		T *tempArr;
		T *topTen;
		int size;
		int capacity;

	public:


		//constructor for the array list object
		ArrayList<T>(){
			this->arr = new T[START_SIZE];
			this->topTen = new T[10];
			this->size = 0;
			this->capacity = START_SIZE;
			
			newArrayList += 1;

			return;
		}
		
		//constructor for the array list object
		//you can specify the starting size
		ArrayList(int size){
			this->arr = new T[size];
			this->topTen = new T[10];
			this->size = 0;
			this->capacity = size;
			
			newArrayList += 1;

			return;
		}
		
		//construction
		ArrayList(T* list, int length){
			this->arr = list;
			this->topTen = new T[10];
			this->size = length;
			this->capacity = length;

			newArrayList += 1;		      
			
		}
		
  //deconstructor for the array list object
  ~ArrayList(){

    for( int i = 0 ; i < this->size ; i++ ){
      delete this->arr[i];
    }
    
    delete[] this->arr;
    delete this->topTen;
    
    //cout << "memory - deleting arraylist" << endl;
    
    newArrayList -= 1;
    
  }
  
		
		//method to add "element" to the last position in arr 
		void add(T element){
			
			//special case when array is full
			if (size >= capacity){
			  // cout << "add: increasing cap" << endl;
				//create a new array twice as large
				//copy data from the first to the second
				this->tempArr = new T[2 * this->capacity];
				for (int i = 0; i < this->capacity; i++){
					this->tempArr[i] = this->arr[i]; 
				}
				
				//	cout << "add : copy data" << endl;
				//update the capacity field
				//delete the old array
				//switch the arr pointer to point to the new array
				this->capacity = this->capacity * 2;
				delete[] this->arr;
				this->arr = this->tempArr;
				//				cout << "add : pointer swapping" << endl;
			}
			
			//add the element to the end of the array
			//increment the size field
			arr[this->size] = element;
			this->size++;
			//			cout << "add : increment and add" << endl;
		}
		
		//method to set a specific element to a given value within the bounds
		void set(T element, int i){
		
			if (i < 0 || i > this->size){
			  //printf("Out of bounds");
			  cout << "out of bounds" << endl;
			}
			else{
				this->arr[i] = element;
			}
		
		}
		
		//remove all elements of the arraylist l from this arraylist
		void remove(ArrayList* l){
			int count = 0;
			int flag = 0;

			T *temp = new T[this->size];
			for (int j = 0; j < this->size; j++){
				int i;
				flag = 0; //false if an entry isnt matched

				for (i = 0; i < l->getSize(); i++){
				  if (this->arr[j] == l->get(i)){
				    flag = 1; //found an entry in the array

				    delete l->get(i); //delete the feature
				    l->set(NULL, i); //set the pointer to null so its not recognized later
				    //i--;
				    break;
				  }
				}
				//if (i + 1 >= l->size){
				if( flag == 0 ){ //if an entry wasn't matched
				temp[count] = this->arr[j];
				count++;
			}
				
			}
			
			T* t = this->arr;
			
			this->arr = temp;
			this->size = count;
			delete[] t;
		}
		
		
		//return the element at the given index i
		T get(int i){
			if (i < 0 || i >= this->size){
				cout << "Index out of bounds: " << i << endl;
				return NULL;
			}
			return this->arr[i];
		}
		
		//return the size of the array list
		int getSize(){
			return this->size;
		}
		
		//makes all elements in the current array list garbage
		void clear(){
		  //this->arr;
		  this->size = 0;						
		  return;
		}
		
		
		//print out the elements of the array list in order
		void print(){
			
			if (this->size > 0){
			  cout << "[";
			}
			else{
				cout << "There are no elements to print." << endl;
				return;
			}
			
			for (int i = 0; i < this->size; i++){
				if (i < this->size -1){
					cout << this->arr[i] << ", ";
				}
				else{
					cout << this->arr[i] << "]" << endl;
				}
			}
			
			return;
		}
		
};

#endif
/*
int main(int argc, char* argv[]){
	
	ArrayList<int> *list = new ArrayList<int>();
	ArrayList<int> *list2 = new ArrayList<int>();
	
	list->add(3);
	list->add(1);
	list->add(4);
	list->add(1);
	list->add(5);
	list2->add(1);
	list2->add(4);
	list2->add(2);
	list->print();
	list->remove(list2);
	list->print();
	cout << "0: " << list->get(0) << endl;
	cout << "1: " << list->get(1) << endl;
	cout << "Size: " << list->getSize() << endl;
	list->clear();
	cout << list->getSize() << endl;
	list->print();
	
	return 0;
	
}
*/

