#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp> 

// #include <utility>// std::pair std::make_pair @Wei

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{   
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    using namespace std;
    std::fill(cells_.begin(),cells_.end(),-1.0f); // initilized with impossible negative distance
    queue<Point<int>> obstacle_cells;
    for(int i = 0 ; i < width_; i++){
        for(int j = 0; j < height_; j++){
            if(map.logOdds(i,j) >= 0){
                this->distance(i,j) = 0.0f;
                obstacle_cells.push(Point<int>(i, j));
            }
        }
    }
    if(obstacle_cells.empty()){
        std::fill(cells_.begin(),cells_.end(),0.0f);
        return;
    }
    printf("@Wei: number of obstacles cells: %zu\n",obstacle_cells.size());

    int next[4][2] = {{1,0},{0,1},{-1,0},{0,-1}};
    while(!obstacle_cells.empty()){
        Point<int> curr_cell = obstacle_cells.front();
        obstacle_cells.pop();
        int i = curr_cell.x;
        int j = curr_cell.y;
        for(int k = 0;k < 4;k++){
            int i_ = i + next[k][0];
            int j_ = j + next[k][1];
            if(!isCellInGrid(i_,j_)) continue;
            if(this->distance(i_,j_) < 0){
                this->distance(i_,j_) = this->distance(i,j) + map.metersPerCell();
                obstacle_cells.push(Point<int>(i_,j_));
            }
        }
    }


}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}