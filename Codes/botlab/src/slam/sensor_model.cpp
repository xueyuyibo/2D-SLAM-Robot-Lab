#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>
#include <algorithm> //std::max


double findLogOdds(const adjusted_ray_t& ray, const OccupancyGrid& map);

SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}

double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    /////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    // Point<int> particle_position =  global_position_to_grid_cell(Point<double>(sample.pose.x,sample.pose.y), map);
    MovingLaserScan MLS_rays = MovingLaserScan(scan, sample.parent_pose, sample.pose, 1);

    double total_log_P = 0;

    for(size_t i = 0; i < MLS_rays.size(); i++)
    {
        adjusted_ray_t this_ray = MLS_rays[i];
        Point<int> init_grid = global_position_to_grid_cell(this_ray.origin, map);
        if(map.isCellInGrid(init_grid.x,init_grid.y)){
            if(map.logOdds(init_grid.x,init_grid.y) > 12){
                return 0.0;
            }
        }

        float x_goal = this_ray.origin.x + this_ray.range * cos(this_ray.theta);
        float y_goal = this_ray.origin.y + this_ray.range * sin(this_ray.theta);
        Point<int> goal_grid = global_position_to_grid_cell(Point<float>(x_goal,y_goal), map);

        if(map.isCellInGrid(goal_grid.x,goal_grid.y)){
            total_log_P += std::max(0, (int) map.logOdds(goal_grid.x, goal_grid.y));
        }
    }
    return total_log_P/ (double) MLS_rays.size()/127.0;

}
