#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>

// #include <vector> //@Wei: moved to hpp
#include <algorithm> // @Wei sort
#include <queue> // @Wei priority_queue
#include <cmath> // @Wei atan2
#include <stack> // Reconstruct the path

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{

    ////////////////// TODO: Implement your A* search here //////////////////////////
    // printf("entering A*\n");

    using namespace std;
    robot_path_t path;
    path.utime = start.utime;
    // path.path.push_back(start); 
    /*your code begins here*/

    Point<double> init_coords(start.x, start.y);
    Point<double> goal_coords(goal.x,goal.y);
    printf("init coord: %lf %lf; goal coord: %lf %lf \n",init_coords.x, init_coords.y, goal_coords.x, goal_coords.y);
    Point<int> init_cell = global_position_to_grid_cell(init_coords, distances);
    Point<int> goal_cell = global_position_to_grid_cell(goal_coords, distances); 
    printf("init index: %d %d; goal_index: %d %d \n",init_cell.x, init_cell.y, goal_cell.x, goal_cell.y);
    printf("global origin: %f %f; cellsPerMeter: %f\n",distances.originInGlobalFrame().x,distances.originInGlobalFrame().y,distances.cellsPerMeter());
 	
    vector<vector<Node*>> search_space;

    search_space.resize(distances.widthInCells());
    for(int i = 0; i < distances.widthInCells(); i++){
    	search_space[i].resize(distances.heightInCells());
    	for(int j = 0; j < distances.heightInCells();j++){
    		search_space[i][j] = new Node(i,j,0.0f);
    	}
    }

    auto ptr = search_space[init_cell.x][init_cell.y];
    ptr -> status = open;
    ptr -> g_score = 0.0f;
    ptr -> h_score = distances.metersPerCell()*Diagonal_Distance(init_cell,goal_cell);
    ptr -> f_score = ptr -> g_score + ptr -> h_score;

    priority_queue<Node*,vector<Node*>, Comp> OPEN;
    OPEN.push(ptr);

    int del[8][2] = {{1,0},{1,1},{0,1},{-1,1},
                    {-1,0},{-1,-1},{0,-1},{1,-1}};
    while(!OPEN.empty())
    {
        Node* ptr = OPEN.top();
        OPEN.pop();

        Point<int> current_cell(ptr->i, ptr->j);
        if(Diagonal_Distance(current_cell, goal_cell) < 4){
            reconstruct_path(ptr,search_space,path,distances);
            path.path.pop_back();
            path.path.push_back(goal);
            path.path_length = path.path.size();
            return path;
        }
        ptr->status = closed;

        for(int n = 0;n < 8;n++){

            int i = ptr->i + 4*del[n][0];
            int j = ptr->j + 4*del[n][1];
            if(!distances.isCellInGrid(i,j)) continue;
            Node* neighbour = search_space[i][j];
            if (neighbour -> status == closed){
                continue;
            }
            if (distances(i,j) < params.minDistanceToObstacle){
                neighbour->status = closed;
                continue;
            }
            Point<int> neighbour_cell(i,j);
            float g_temp = ptr->g_score +  distances.metersPerCell()*Manhattan_Distance(current_cell, neighbour_cell);
            if(neighbour->status == unknown){

                neighbour->theta = slope_between(ptr,neighbour);
                neighbour->parent = ptr;
                neighbour->status = open;
                neighbour->g_score = g_temp;
                neighbour->f_score = g_temp + distances.metersPerCell()*Manhattan_Distance(neighbour_cell,goal_cell);
                if(distances(i,j) < params.maxDistanceWithCost)
                    neighbour->f_score += powf(params.maxDistanceWithCost - distances(i,j), params.distanceCostExponent);
                OPEN.push(neighbour);
            }
        }

    }
    printf("@Wei: A* does not find a path!\n");
    path.path.clear();
    path.path.push_back(start);
    path.path_length = 1;
    return path;
}


////////////////@Wei: A* utility
Node::Node(int i_, int j_, float theta_){
	i = i_;
	j = j_;
    theta = theta_;
	f_score = 1e12;
	h_score = 1e12;
	g_score = 1e12;
	parent = NULL;
	status = unknown;
}

float Euclidean_Distance(Point<int> A,Point<int> B){
    return sqrt(powf(A.x- B.x,2) + powf(A.y-B.y,2));
}

float Manhattan_Distance(Point<int> A,Point<int> B){
    int dx = abs(A.x - B.x);
    int dy = abs(A.y - B.y);
    return (float) (dx + dy);
}
float Diagonal_Distance(Point<int> A,Point<int> B){
    int dx = abs(A.x - B.x);
    int dy = abs(A.y - B.y);
    float dist = dx + dy + (sqrt(2.0) - 2.0) * std::min(dx,dy);
    return dist;
}

float slope_between(Node* init, Node* goal){
    float dy = goal->j - init->j;
    float dx = goal->i - init->i;
    float angle = wrap_to_pi(atan2(dy,dx));
    return angle;
}

void reconstruct_path(Node* ptr,
                        std::vector<std::vector<Node*>>& grid,
                        robot_path_t& path,
                        const ObstacleDistanceGrid& distances)
{   
    using namespace std;
    stack<pose_xyt_t> storage;
    while(ptr != NULL){
        pose_xyt_t pose;
        pose.utime = 0;
        Point<double> global_coord =  grid_position_to_global_position(Point<double>(ptr->i,ptr->j),distances);
        pose.x = global_coord.x;
        pose.y = global_coord.y;
        pose.theta = wrap_to_pi( ptr->theta);
        storage.push(pose);
        ptr = ptr->parent;
    }
    storage.pop();
    while(!storage.empty()){
        path.path.push_back(storage.top());
        storage.pop();
    }
}