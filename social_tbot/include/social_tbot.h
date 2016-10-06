#ifndef SOCIAL_TBOT_HEADER
#define SOCIAL_TBOT_HEADER

#include "ros/ros.h"

#define INIT_COST 100000

class Cell{
public:
	float x;
	float y;
	float gCost;

	int grid_x_loc;
	int grid_y_loc;

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
	std::map< Cell, float, Cell_Compare> gScore;
	std::map< Cell, float, Cell_Compare> fScore;


  	float x_min; float y_min;
  	float x_max; float y_max;
	int resolution;

	int grid_x_min; int grid_y_min;
	int grid_x_max; int grid_y_max;	

	float dx;
	float dy;

	Cell start_cell;
	Cell goal_cell;	

	void init_gfScores();
	void init_start_goal_cells(const float &start_x, const float &start_y, 
							   const float &goal_x, const float &goal_y);
	int find_path(const float &start_x, const float &start_y, 
							   const float &goal_x, const float &goal_y);


	float heuristic_distance_cost(Cell c1_start, Cell c1_end);
	float dist_between(Cell c1_start, Cell c1_end);
	float social_cost(Cell cell_eval);	

	void sort_open_set();
	int x_location_to_map_index(const float &query_in);
	int y_location_to_map_index(const float &query_in);	
	std::vector<Cell> neighbors(const Cell &cell_query);

	Cell convert_xy_to_cell(const float &x, const float &y);


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

  int cell_index = 0;
  for(size_t i = 0; i < resolution; i++){
  	  for(size_t j = 0; j < resolution; j++){
  	  	float x_val = x_min + i*dx;
		float y_val = y_min + j*dy;  	  	
  	  	int x = floor(x_val*resolution);
  	  	int y = floor(y_val*resolution);  	  	
  		std::cout << "x:" << x << " y:" << y << std::endl;

  		// Initialize cell location
  		Cell new_cell(cell_index);
  		new_cell.x = x_val;
  		new_cell.y = y_val;  		
  		new_cell.grid_x_loc = x;
  		new_cell.grid_y_loc = y;

		grid[x][y] = new_cell; 

		if ((i == 0) && (j == 0)){
			grid_x_min = x; // Store grid minimums
			grid_y_min = y;			
		}
		grid_x_max = x; // Store grid maximums
		grid_y_max = y;

		cell_index++;			
  	  }
  }
  // End grid initialization.
  init_gfScores(); // initialize gfScores

}

A_star::~A_star(){}

void A_star::init_gfScores(){
	gScore.clear();
	fScore.clear();
	int total_cells = grid.size() * grid[0].size();

	//std::cout << total_cells << std::endl;

	typedef std::map<int, std::map< int, Cell> >::iterator it_x;	
	typedef std::map<int, Cell>::iterator it_y;

	for(it_x iter_i = grid.begin(); iter_i != grid.end(); iter_i++) {
		for(it_y iter_j = iter_i->second.begin(); iter_j != iter_i->second.end(); iter_j++){ 			
			(iter_j->second).gCost = INIT_COST;
			gScore[ (iter_j->second) ] = INIT_COST;
			fScore[(iter_j->second)] = INIT_COST;
		}
    	// iterator->first = key
    	// iterator->second = value
    	// Repeat if you also want to iterate through the second map.
	}

	//std::cout<< "Hello. size of gscore is: " << gScore.size() << std::endl;

}

void A_star::sort_open_set(){
	std::sort (open_set.begin(), open_set.end(), cell_cost_compare_obj); 
}

// Given a location query eg: x = 9.5
int A_star::x_location_to_map_index(const float &query_in){
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

int A_star::y_location_to_map_index(const float &query_in){
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


bool operator== ( const Cell &c1, const Cell &c2) 
{
        return c1.id == c2.id;
}

std::vector<Cell> A_star::neighbors(const Cell &cell_query){
	// Performs a simple neighbor search since the grid is 2D and everything is reachable
	std::vector<Cell> current_neighbors;
	std::vector<Cell>::iterator it_cell;

	std::cout << "HELLO " ;
  	std::cout << cell_query.grid_x_loc << "," << cell_query.grid_y_loc << std::endl;  	

	//This has 8 neighbors
	float current_x = cell_query.x;
	float current_y = cell_query.y;

	// Neighbor 1 LowerLeft
	float neighbor_1_x = current_x - dx;
	float neighbor_1_y = current_y - dy;
	int index_x = x_location_to_map_index(neighbor_1_x); 
	int index_y = y_location_to_map_index(neighbor_1_y);
		// Add if not in list and not itself
		it_cell = std::find(current_neighbors.begin(), current_neighbors.end(), grid[index_x][index_y]);
		if ((it_cell == current_neighbors.end()) && ( (grid[index_x][index_y]).id != cell_query.id)){
			current_neighbors.push_back(grid[index_x][index_y]);	
		}

	// neighbor 2 Lower
	float neighbor_2_x = current_x;
	float neighbor_2_y = current_y - dy;
	index_x = x_location_to_map_index(neighbor_2_x); 
	index_y = y_location_to_map_index(neighbor_2_y);
		// Add if not in list and not itself
		it_cell = std::find(current_neighbors.begin(), current_neighbors.end(), grid[index_x][index_y]);
		if ((it_cell == current_neighbors.end()) && ( (grid[index_x][index_y]).id != cell_query.id)){
			current_neighbors.push_back(grid[index_x][index_y]);	
		}

	// Neighbor 3 LowerRight
	float neighbor_3_x = current_x + dx;
	float neighbor_3_y = current_y - dy;
	index_x = x_location_to_map_index(neighbor_3_x); 
	index_y = y_location_to_map_index(neighbor_3_y);
		// Add if not in list and not itself
		it_cell = std::find(current_neighbors.begin(), current_neighbors.end(), grid[index_x][index_y]);
		if ((it_cell == current_neighbors.end()) && ( (grid[index_x][index_y]).id != cell_query.id)){
			current_neighbors.push_back(grid[index_x][index_y]);	
		}

	// neighbor 4 MidRight
	float neighbor_4_x = current_x + dx;
	float neighbor_4_y = current_y;	
	index_x = x_location_to_map_index(neighbor_4_x); 
	index_y = y_location_to_map_index(neighbor_4_y);
		// Add if not in list and not itself
		it_cell = std::find(current_neighbors.begin(), current_neighbors.end(), grid[index_x][index_y]);
		if ((it_cell == current_neighbors.end()) && ( (grid[index_x][index_y]).id != cell_query.id)){
			current_neighbors.push_back(grid[index_x][index_y]);	
		}

	// neighbor 5 UpperRight
	float neighbor_5_x = current_x + dx;
	float neighbor_5_y = current_y + dy;		
	index_x = x_location_to_map_index(neighbor_5_x); 
	index_y = y_location_to_map_index(neighbor_5_y);
		// Add if not in list and not itself
		it_cell = std::find(current_neighbors.begin(), current_neighbors.end(), grid[index_x][index_y]);
		if ((it_cell == current_neighbors.end()) && ( (grid[index_x][index_y]).id != cell_query.id)){
			current_neighbors.push_back(grid[index_x][index_y]);	
		}


	// neighbor 6 MidUpper
	float neighbor_6_x = current_x;
	float neighbor_6_y = current_y + dy;			
	index_x = x_location_to_map_index(neighbor_6_x); 
	index_y = y_location_to_map_index(neighbor_6_y);
		// Add if not in list and not itself
		it_cell = std::find(current_neighbors.begin(), current_neighbors.end(), grid[index_x][index_y]);
		if ((it_cell == current_neighbors.end()) && ( (grid[index_x][index_y]).id != cell_query.id)){
			current_neighbors.push_back(grid[index_x][index_y]);	
		}


	// neighbor 7 UpperLeft
	float neighbor_7_x = current_x - dx;
	float neighbor_7_y = current_y + dy;
	index_x = x_location_to_map_index(neighbor_7_x); 
	index_y = y_location_to_map_index(neighbor_7_y);
		// Add if not in list and not itself
		it_cell = std::find(current_neighbors.begin(), current_neighbors.end(), grid[index_x][index_y]);
		if ((it_cell == current_neighbors.end()) && ( (grid[index_x][index_y]).id != cell_query.id)){
			current_neighbors.push_back(grid[index_x][index_y]);	
		}

	// neighbor 8 MidLeft
	float neighbor_8_x = current_x - dx;
	float neighbor_8_y = current_y;				
	index_x = x_location_to_map_index(neighbor_8_x); 
	index_y = y_location_to_map_index(neighbor_8_y);
		// Add if not in list and not itself
		it_cell = std::find(current_neighbors.begin(), current_neighbors.end(), grid[index_x][index_y]);
		if ((it_cell == current_neighbors.end()) && ( (grid[index_x][index_y]).id != cell_query.id)){
			current_neighbors.push_back(grid[index_x][index_y]);	
		}


/*	std::cout << "Neighbors of (" << current_x << "," << current_y << ")" << std::endl;
	  for(size_t i = 0; i < current_neighbors.size(); i++){
	  	std::cout << "    gridX:" << current_neighbors[i].grid_x_loc << " gridY:" << current_neighbors[i].grid_y_loc << " id:" << current_neighbors[i].id << std::endl;
	  }				*/


	return current_neighbors;
}

Cell A_star::convert_xy_to_cell(const float &x, const float &y){
	int index_x = x_location_to_map_index(x);
	int index_y = y_location_to_map_index(y);	
	return grid[index_x][index_y];
}

void A_star::init_start_goal_cells(const float &start_x, const float &start_y, 
			   				       const float &goal_x, const float &goal_y){
	start_cell = convert_xy_to_cell(start_x, start_y);
	goal_cell = convert_xy_to_cell(goal_x, goal_y);
}

int A_star::find_path(const float &start_x, const float &start_y, 
			   				       const float &goal_x, const float &goal_y){
	open_set.clear();	
	cameFrom.clear();
	gScore.clear();
	fScore.clear();
	closed_set.clear(); 

	// Start A* search
	init_start_goal_cells(start_x, start_y, goal_x, goal_y);
	init_gfScores();

	
	gScore[start_cell] = 0;
	fScore[start_cell] = heuristic_distance_cost(start_cell, goal_cell);
		start_cell.gCost = fScore[start_cell];
		grid[start_cell.grid_x_loc][start_cell.grid_y_loc] = fScore[start_cell];

	open_set.push_back(start_cell);

	while (open_set.size() > 0){
		sort_open_set();
		Cell current = open_set[0]; // Lowest fScore
		if (current == goal_cell){
			std::cout << "Found goal" << std::endl;
			return 1;
		}
		open_set.erase(open_set.begin()); // Remove current
		closed_set.push_back(current); // Add current to explored states

		// for each neighbor of current
		std::vector<Cell> list_of_neighbors = neighbors(current);

		std::cout << "Neighbors of (" << current.x << "," << current.y << ")" << std::endl;
		for(size_t i = 0; i < list_of_neighbors.size(); i++){
			std::cout << "    gridX:" << list_of_neighbors[i].grid_x_loc << " gridY:" << list_of_neighbors[i].grid_y_loc << " id:" << list_of_neighbors[i].id << std::endl;

			Cell neighbor = list_of_neighbors[i];
			// if neighbor in closedSet:
			std::vector<Cell>::iterator it_cell;
			it_cell = std::find(closed_set.begin(), closed_set.end(), neighbor);
			if (it_cell != closed_set.end()) {
				continue; // ignore since it's already evaluated
			}			
            // The distance from start to a neighbor
            float tentative_gScore = gScore[current] + dist_between(current, neighbor); //+ othercost

			// if neighbor not in openSet:
			it_cell = std::find(open_set.begin(), open_set.end(), neighbor);
			if (it_cell == open_set.end()) {
				// Record this path
	            cameFrom[neighbor] = current;
	            gScore[neighbor] = tentative_gScore;
	            fScore[neighbor] = gScore[neighbor] + heuristic_distance_cost(neighbor, goal_cell);
	            neighbor.gCost = fScore[neighbor];
				open_set.push_back(neighbor); // Discover new node			
			}else if(tentative_gScore >= gScore[neighbor]){
				continue; // This is not a better path

			}			
		}	


	}

}

float A_star::heuristic_distance_cost(Cell c1_start, Cell c1_end){
	float h_dcost =  sqrt(pow((c1_start.x - c1_end.x),2) + pow((c1_start.y - c1_end.y),2));
	std::cout << "Distance:" << h_dcost << std::endl;
	return h_dcost;
}

float A_star::dist_between(Cell c1_start, Cell c1_end){
	float dist_cost =  sqrt(pow((c1_start.x - c1_end.x),2) + pow((c1_start.y - c1_end.y),2));
	std::cout << "Dist_between cost:" << dist_cost << std::endl;
	return dist_cost;
}



#endif
