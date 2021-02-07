/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * Modified(Implemented): Sajith Nandasena Feb, 2021
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
using std::uniform_real_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    if (is_initialized)
        return;

    // create normal(Guassinan) distribution for x
    normal_distribution<double> dist_x(x, std[0]);
    // create normal(Guassinan) distribution for y
    normal_distribution<double> dist_y(y, std[1]);
    // create normal(Guassinan) distribution for theta
    normal_distribution<double> dist_theta(theta, std[2]);

    // set the number of particles
//    num_particles = 100;
    // create particles
    for (int i = 0; i < num_particles; ++i)
    {
        Particle particle;
        particle.id = i;
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
    // create normal(Guassinan) distribution for x
    normal_distribution<double> dist_x(0, std_pos[0]);
    // create normal(Guassinan) distribution for y
    normal_distribution<double> dist_y(0, std_pos[1]);
    // create normal(Guassinan) distribution for theta
    normal_distribution<double> dist_theta(0, std_pos[2]);

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

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{

    double stdland_x = std_landmark[0];
    double stdland_y = std_landmark[1];

    for (auto &particle: particles)
    {
        double x, y, theta;
        x = particle.x;
        y = particle.y;
        theta = particle.theta;

        // find landmarks in the map that are in particle's sensor range.
        vector<LandmarkObs> inRangeLandmarks; // predictions
        for (const auto &landmark : map_landmarks.landmark_list)
        {
            float landmark_x = landmark.x_f;
            float landmark_y = landmark.y_f;
            int id = landmark.id_i;

            if (fabs(landmark_x - x) <= sensor_range && fabs(landmark_y - y) <= sensor_range)
            {
                inRangeLandmarks.emplace_back(LandmarkObs{id, landmark_x, landmark_y});
            }
        }

        // transform observation coodinates(measured in particle's coordinate system) to map  coodinates
        vector<LandmarkObs> mappedObservations;
        for (const auto &observation : observations)
        {
            double mapped_x = cos(theta) * observation.x - sin(theta) * observation.y + x;
            double mapped_y = sin(theta) * observation.x + cos(theta) * observation.y + y;
            mappedObservations.emplace_back(LandmarkObs{observation.id, mapped_x, mapped_y});
        }


        // update the nearest landmark for each observation
        dataAssociation(inRangeLandmarks, mappedObservations);

        // reseting the weight
        particle.weight = 1.0;

        // calculate  weights
        for (const auto &mappedObservation: mappedObservations)
        {
            double obs_x = mappedObservation.x;
            double obs_y = mappedObservation.y;
            int landmark_id = mappedObservation.id;

            double landmark_x, landmark_y;
            for (const auto &landmark : inRangeLandmarks)
            {
                if (landmark.id == landmark_id)
                {
                    landmark_x = landmark.x;
                    landmark_y = landmark.y;
                    break;
                }
            }

            // calculate weight
            double dx = obs_x - landmark_x; // todo
            double dy = obs_y - landmark_y; // todo

            // Multivariate Gaussian probabilty

            double gaussian_norm = (1.0 / 2 * M_PI * stdland_x * stdland_y);
            double exponent = dx * dx / (2 * pow(stdland_x, 2)) + dy * dy / (2 * pow(stdland_y, 2));
            double weight = gaussian_norm * exp(-exponent);

            if (weight == 0)
                particle.weight *= 0.00001;
            else
                particle.weight *= weight;

        }
    }
}

void ParticleFilter::resample()
{
    // retrive initial index
    std::uniform_int_distribution<int> uni_dist(0, num_particles - 1);
    int index = uni_dist(gen);

    double beta = 0.0;

    // get weights and max weight
    vector<double> weights;
    double max_weight = std::numeric_limits<double>::min();
    for (const auto &particle: particles)
    {
        weights.emplace_back(particle.weight);
        if (particle.weight > max_weight)
        {
            max_weight = particle.weight;
        }
    }

    // creates distribution
    uniform_real_distribution<double> dist(0.0, max_weight);

    vector<Particle> resampledParticles;
    for (int i = 0; i < num_particles; ++i)
    {
        beta += dist(gen) * 2.0;
        while (beta > weights[index])
        {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }

        std::cout << beta << "\t" << index << "\n";
        resampledParticles.emplace_back(particles[index]);
    }

    particles = resampledParticles;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{

    // cleart the preovious associans
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

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