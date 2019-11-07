#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <cmath> // cos sin

using namespace std;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
	PrevPose.utime = 0;
	PrevPose.x = 0;
	PrevPose.y = 0;
	PrevPose.theta = 0;
}

void Mapping::updateMap(const lidar_t& scan, 
						const pose_xyt_t& pose,
						OccupancyGrid& map)
{

	MovingLaserScan moving_laser_scan(scan,PrevPose,pose,1);
	for(size_t i = 0; i < moving_laser_scan.size();i++){

		adjusted_ray_t this_ray = moving_laser_scan[i];
		if(this_ray.range <= kMaxLaserDistance_){

			double x_goal = this_ray.origin.x + this_ray.range* cos(this_ray.theta);
			double y_goal = this_ray.origin.y + this_ray.range* sin(this_ray.theta);

			Point<int> init_grid_cell = global_position_to_grid_cell(this_ray.origin, map);
			Point<int> goal_grid_cell = global_position_to_grid_cell(Point<double>(x_goal,y_goal), map);

			int x0 = init_grid_cell.x;
			int y0 = init_grid_cell.y;
			int x1 = goal_grid_cell.x;
			int y1 = goal_grid_cell.y;

			int sx = x0 < x1 ? 1 : -1;
			int sy = y0 < y1 ? 1 : -1;

			int dx = abs(x1 - x0);
			int dy = abs(y1 - y0);
				
			int err = dx - dy;
			int x = x0;
			int y = y0;

			int new_odd = std::min(127, map.logOdds(x1,y1) + kHitOdds_);
			map.setLogOdds(x1,y1,new_odd);
			while(x != x1 || y != y1){

				if(!map.isCellInGrid(x,y)) continue;

				int new_odd = std::max(-128, map.logOdds(x,y) - kMissOdds_);
				map.setLogOdds(x,y,new_odd);
				int e2 = 2*err;
				if (e2 >= -dy){
					err -= dy;
					x += sx;
				}
				if (e2 <= dx){
					err += dx;
					y += sy;
				}
			}
		}
	}
	PrevPose = pose;
}