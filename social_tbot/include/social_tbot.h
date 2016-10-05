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
	std::vector< std::vector<Cell> > grid;

	std::vector<Cell> open_set;
	std::vector<Cell> closed_set;
	std::map< Cell,  Cell, Cell_Compare> cameFrom; // cameFrom[Cell] returns the cell it camefrom
	std::map< Cell, float> gScore;
	std::map< Cell, float> fscore;

	void sort_open_set();

	A_star();
	~A_star();
};


Cell::Cell(){}
Cell::Cell(int id_): id(id_){
	gCost = INIT_COST; 
}
Cell::~Cell(){}

A_star::A_star(){}
A_star::~A_star(){}

void A_star::sort_open_set(){
	std::sort (open_set.begin(), open_set.end(), cell_cost_compare_obj); 
}

#endif
