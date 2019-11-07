
#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <random>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

struct particle_t;

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
* 
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal 
* distribution for the particle filter.
*/
class ActionModel
{
public:
    
    ActionModel(void);
    
    bool updateAction(const pose_xyt_t& odometry);
    
    particle_t applyAction(const particle_t& sample);

    void set_prev_odometry(const pose_xyt_t& pose);

private:
    
    int64_t latest_time;
    pose_xyt_t prev_odometry;
    float del_rot1;
    float del_trans;
    float del_rot2;
    float alpha_1;
    float alpha_2;
    float alpha_3;
    float alpha_4;

    std::default_random_engine generator;
        
    float Gaussian(float mean,float stddev);
 };

#endif // SLAM_ACTION_MODEL_HPP
