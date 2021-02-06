/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    if (is_initialized)
        return;

    // standard deviations
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2]; // yaw rate stdev

    // create normal(Guassinan) distribution for x
    normal_distribution<double> dist_x(x, std_x);
    // create normal(Guassinan) distribution for y
    normal_distribution<double> dist_y(y, std_y);
    // create normal(Guassinan) distribution for theta
    normal_distribution<double> dist_theta(theta, std_theta);

    // set the number of particles
    num_particles = 100;
    // create particles
    for (int i = 0; i < num_particles; ++i)
    {
        Particle particle;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1.0;
        particles.emplace_back(particle);
    }
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    // standards deviations
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];


    // create normal(Guassinan) distribution for x
    normal_distribution<double> dist_x(0, std_x);
    // create normal(Guassinan) distribution for y
    normal_distribution<double> dist_y(0, std_y);
    // create normal(Guassinan) distribution for theta
    normal_distribution<double> dist_theta(0, std_theta);

    for (auto &particle: particles)
    {
        double theta = particle.theta;
        if (fabs(yaw_rate) < 0.00001)
        {
            particle.x += velocity * delta_t * cos(theta);
            particle.y += velocity * delta_t * sin(theta);
        }
        else
        {
            particle.x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
            particle.y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
            particle.theta += yaw_rate * delta_t;
        }

        // Adding noises
        particle.x += dist_x(gen);
        particle.y += dist_y(gen);
        particle.theta += dist_theta(gen);

    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs> &observations)
{
    for (auto &observation : observations)
    {
        double mindistance = std::numeric_limits<const double>::max();
        int mapID = -1;
        for (auto &predict : predicted)
        {
            double x = observation.x - predict.x;
            double y = observation.y - predict.y;

            double distance = x * x + y * y;

            // if the distance is less than min, store the ID and udate min.
            if (distance < mindistance)
            {
                mindistance = distance;
                mapID = predict.id;
            }
        }

        // update observation identifier
        observation.id = mapID;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
    /**
     * TODO: Update the weights of each particle using a mult-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     * NOTE: The observations are given in the VEHICLE'S coordinate system.
     *   Your particles are located according to the MAP'S coordinate system.
     *   You will need to transform between the two systems. Keep in mind that
     *   this transformation requires both rotation AND translation (but no scaling).
     *   The following is a good resource for the theory:
     *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
     *   and the following is a good resource for the actual equation to implement
     *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
     */

}

void ParticleFilter::resample()
{
    /**
     * TODO: Resample particles with replacement with probability proportional
     *   to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */

}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
    vector<double> v;

    if (coord == "X")
    {
        v = best.sense_x;
    }
    else
    {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}