#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#define PI 3.14159265358979f
using namespace std;

ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    latest_time = 0;

    prev_odometry.utime = 0;
    prev_odometry.x = 0.0;
    prev_odometry.y = 0.0;
    prev_odometry.theta = 0.0;

    del_rot1 = 0.0;
    del_trans = 0.0;
    del_rot2 = 0.0;

    // alpha_1 = 0.15;
    // alpha_2 = 0.03;
    // alpha_3 = 0.10;
    // alpha_4 = 0.02;

    alpha_1 = 1.5;
    alpha_2 = 0.3;
    alpha_3 = 1.0;
    alpha_4 = 0.2;

    srand(time(0));
}

float ActionModel::Gaussian(float mean, float stddev)
{
    std::normal_distribution<float> my_distribution(mean, sqrt(stddev)); // @ Roger
    return my_distribution(generator);
}

void ActionModel::set_prev_odometry(const pose_xyt_t& pose){
    prev_odometry = pose;
}

bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    latest_time = odometry.utime;

    float dx = odometry.x - prev_odometry.x;
    float dy = odometry.y - prev_odometry.y;
    float slope = atan2(dy, dx);

    del_rot1 = wrap_to_pi(slope - prev_odometry.theta);
    del_trans = sqrt(powf(dx,2.0f) + powf(dy,2.0f));
    del_rot2 = wrap_to_pi(odometry.theta - slope);

    if(fabs(del_rot1) > PI/2 && fabs(del_rot2)>PI/2){
        del_rot1 = wrap_to_pi(del_rot1 + PI);
        del_trans *= -1.0f;
        del_rot2 = wrap_to_pi(del_rot2 + PI);
    }

    // printf("%f %f %f\n",del_rot1,del_trans,del_rot2);

    if( fabs(del_rot1 + del_rot2)> 0.02 || fabs(del_trans) >0.01){
        prev_odometry = odometry; 
        return true;
    }
    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
	///// Probablistic Robotics -> P. 136 -> Table 5.6
    //////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    //// Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_particle;

	float noise_rot1  = Gaussian(0,alpha_1*powf(del_rot1,2.0f)+alpha_2*powf(del_trans,2.0f));
    float noise_trans = Gaussian(0,alpha_3*powf(del_trans,2.0f)+alpha_4*powf(del_rot1 ,2.0f)+alpha_4*powf(del_rot2,2.0f));
    float noise_rot2  = Gaussian(0,alpha_1*powf(del_rot2,2.0f)+alpha_2*powf(del_trans,2.0f));
    
    float del_rot1_hat  = del_rot1 - noise_rot1;
    float del_trans_hat = del_trans - noise_trans;
    float del_rot2_hat  = del_rot2 - noise_rot2;

    // printf("%10.5f %10.5f %10.5f\n", del_rot1_hat, del_trans_hat, del_rot2_hat);

    new_particle.parent_pose = sample.pose;
    new_particle.weight = sample.weight;

    new_particle.pose.x = sample.pose.x + del_trans_hat * cos(sample.pose.theta + del_rot1_hat);
    new_particle.pose.y = sample.pose.y + del_trans_hat * sin(sample.pose.theta + del_rot1_hat);
    new_particle.pose.theta = wrap_to_pi(sample.pose.theta + del_rot1_hat + del_rot2_hat); //@Wei: bug was here
    new_particle.pose.utime = latest_time;
    return new_particle;
}
