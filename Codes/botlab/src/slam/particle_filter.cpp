#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    actionModel_.set_prev_odometry(pose);
    posteriorPose_ = pose;
    // Initialize particles
    std::default_random_engine generator;
    std::normal_distribution<float> distribution_x (pose.x, 0.1); // 0.10
    std::normal_distribution<float> distribution_y (pose.y, 0.1); // 0.10
    std::normal_distribution<float> distribution_theta (pose.theta, 0.03); // 0.03
    pose_xyt_t rand_pose;
    for(int i = 0; i < kNumParticles_; i++){
        rand_pose.x = distribution_x(generator);
        rand_pose.y = distribution_y(generator);
        rand_pose.theta = distribution_theta(generator);
        posterior_[i].pose = rand_pose;
        posterior_[i].parent_pose = pose;
        posterior_[i].weight = 1.0/kNumParticles_;
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

/*@Wei*/
std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution /////////////////// 
    // Low Variance Resampling Lecture 15 Page 11
    std::vector<particle_t> prior;
    prior.clear();

    double r = (double) std::rand()/(double) RAND_MAX / (double) kNumParticles_;
    double c = posterior_[0].weight;
    int i = 0;
    for(int m = 0; m < kNumParticles_; m++){
        double U = r + (double) m/ (double) kNumParticles_;
        while(U > c){
            i++;
            c += posterior_[i].weight;
        }
        prior.push_back(posterior_[i]);
    }
    assert((int)prior.size() == kNumParticles_);
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    proposal.resize(kNumParticles_);

    for(int i = 0; i < kNumParticles_; i++)
        proposal[i]  = actionModel_.applyAction(prior[i]);

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    // std::vector<particle_t> posterior = proposal;
    std::vector<particle_t> posterior = proposal;
    assert((int) posterior.size() == kNumParticles_);

    double total_weight = 0;
    for(int i = 0; i < kNumParticles_; i++){
        posterior[i].weight = sensorModel_.likelihood(posterior[i], laser, map);
        total_weight += posterior[i].weight;
    }
    for(int i = 0; i < kNumParticles_; i++){
        posterior[i].weight /= total_weight;
        // printf("%10.7f %10.7f %10.7f %10.7f\n",
            // posterior[i].weight, 
            // posterior[i].pose.x,
            // posterior[i].pose.y, 
            // posterior[i].pose.theta);
    }

    return posterior;

}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior) // no time
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    //////// @Wei Weighted mean is not implemented!!!
    pose_xyt_t pose;

    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    pose.utime = posterior[0].pose.utime; // @Wei

    float COS(0);
    float SIN(0);
    for(int i = 0; i < kNumParticles_; i++){
        pose.x  += posterior[i].weight * posterior[i].pose.x;
        pose.y  += posterior[i].weight * posterior[i].pose.y;
        SIN     += posterior[i].weight * sin(posterior[i].pose.theta);
        COS     += posterior[i].weight * cos(posterior[i].pose.theta);
    }
    pose.theta = wrap_to_pi(atan2(SIN,COS)); 

    // printf("angle : %8.4f\n",pose.theta);
    return pose;
}
