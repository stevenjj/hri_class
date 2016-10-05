#ifndef SOCIAL_TBOT_HEADER
#define SOCIAL_TBOT_HEADER

#include "ros/ros.h"

#define INIT_COST 100000

class Cell{
public:
	int x;
	int y;
	float gCost;

	int id;
	Cell(int id_);
	Cell();
	~Cell();


};

struct Cell_Compare
{
   bool operator() (const Cell& lhs, const Cell& rhs) const
   {
       return lhs.id < rhs.id;
   }
};

struct Cell_Cost_Compare
{
   bool operator() (const Cell& lhs, const Cell& rhs) const
   {
       return lhs.gCost < rhs.gCost;
   }
}cell_cost_compare_obj;


class A_star{
public:
	std::map<int, std::map< int, Cell> > grid;
	std::vector<Cell> open_set;
	std::vector<Cell> closed_set;
	std::map< Cell,  Cell, Cell_Compare> cameFrom; // cameFrom[Cell] returns the cell it camefrom
	std::map< Cell, float> gScore;
	std::map< Cell, float> fscore;


  	float x_min; float y_min;
  	float x_max; float y_max;
	int resolution;

	int grid_x_min; int grid_y_min;
	int grid_x_max; int grid_y_max;	

	float dx;
	float dy;

	void sort_open_set();
	int x_location_to_map_index(float query_in);
	int y_location_to_map_index(float query_in);	

	A_star();
	A_star(int resolution_, float x_min_, float x_max_, float y_min_, float y_max_);
	~A_star();
};


Cell::Cell(){}
Cell::Cell(int id_): id(id_){
	gCost = INIT_COST; 
}
Cell::~Cell(){}

A_star::A_star(){}
A_star::A_star(int resolution_, float x_min_, float x_max_, float y_min_, float y_max_): 
               resolution(resolution_),
			   x_min(x_min_), x_max(x_max_),
			   y_min(y_min_), y_max(y_max_){

  // Initialize the grid discretized by the resolution
  dx = (x_max - x_min) / (float) resolution;
  dy = (y_max - y_min) / (float) resolution;  
  for(size_t i = 0; i < resolution; i++){
  	  for(size_t j = 0; j < resolution; j++){
  	  	int x = floor((x_min + i*dx)*resolution);
  	  	int y = floor((y_min + j*dy)*resolution);  	  	
  		std::cout << "x:" << x << " y:" << y << std::endl;
  		Cell new_cell(i);
		grid[x][y] = new_cell; 

		if ((i == 0) && (j == 0)){
			grid_x_min = x; // Store grid minimums
			grid_y_min = y;			
		}
		grid_x_max = x; // Store grid maximums
		grid_y_max = y;			
  	  }
  }


}


A_star::~A_star(){}

void A_star::sort_open_set(){
	std::sort (open_set.begin(), open_set.end(), cell_cost_compare_obj); 
}

// Given a location query eg: x = 9.5
int A_star::x_location_to_map_index(float query_in){
  float query = query_in;
  if (query <= x_min){
  	return grid_x_min;
  }
  if (query >= x_max){
  	return grid_x_max;
  }

  int index = floor((query - x_min)/dx) ;
  int result = floor( (x_min + index*dx) *resolution);

//  int num = floor(query*resolution);
//  int result = num - (num % 10) ;

  return result;
}

int A_star::y_location_to_map_index(float query_in){
  float query = query_in;
  if (query <= y_min){
  	return grid_y_min;
  }
  if (query >= y_max){
  	return grid_y_max;
  }

  int index = floor((query - y_min)/dy) ;
  int result = floor( (y_min + index*dy) *resolution);

//  int num = floor(query*resolution);
//  int result = num - (num % 10) ;
  return result;
}



#endif
