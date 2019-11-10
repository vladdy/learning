/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 100;

    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for(int i = 0; i < num_particles; i++) {
        struct Particle p{i, dist_x(gen), dist_y(gen), dist_theta(gen), 1.};
        particles.push_back(p);
        weights.push_back(p.weight);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    normal_distribution<double> noise_x(0, std_pos[0]);
    normal_distribution<double> noise_y(0, std_pos[1]);
    normal_distribution<double> noise_theta(0, std_pos[2]);

    for (auto& p: particles) {
        if (fabs(yaw_rate) < 1e-5) {
            p.x += velocity * delta_t * cos(p.theta) + noise_x(gen);
            p.y += velocity * delta_t * sin(p.theta) + noise_y(gen);
        } else {
            double theta0 = p.theta;
            p.x += velocity * (sin(theta0 + yaw_rate * delta_t) - sin(theta0)) / yaw_rate;
            p.x += noise_x(gen);
            p.y += velocity * (cos(theta0) - cos(theta0 + yaw_rate * delta_t)) / yaw_rate;
            p.y += noise_y(gen);
            p.theta += yaw_rate * delta_t + noise_theta(gen);
        }
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    for (auto& o: observations) {
        double min_d = 10000000.0;

        for (auto& p : predicted) {
            double d = dist(p.x, p.y, o.x, o.y);
            if(d < min_d) {
                o.id = p.id;
                min_d = d;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    double sx = std_landmark[0];
    double sy = std_landmark[1];

    std::vector<LandmarkObs> landmarks;
    transform(begin(map_landmarks.landmark_list), end(map_landmarks.landmark_list), back_inserter(landmarks),
        [](const Map::single_landmark_s& l) {
        return LandmarkObs{l.id_i, l.x_f, l.y_f};
    });

    weights.clear();

    for (auto& p: particles) {
        std::vector<LandmarkObs> obs;

        for (const auto& o: observations) {
            LandmarkObs l;

            l.x = o.x*cos(p.theta) - o.y*sin(p.theta) + p.x;
            l.y = o.x*sin(p.theta) + o.y*cos(p.theta) + p.y;
            obs.push_back(l);
        }

        dataAssociation(landmarks, obs);

        p.weight = 1;

        for (const auto& o: obs) {
            LandmarkObs lm = landmarks[o.id-1];

            double dx = o.x - lm.x;
            double dy = o.y - lm.y;

            double w = exp(-(dx * dx)/(2 * sx * sx) - (dy * dy)/(2 * sy * sy))/(2 * sx * sy * M_PI);

            p.weight *= w;
        }
        weights.push_back(p.weight);
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    std::vector<Particle> p_samples;

    random_device rd;
    mt19937 gen(rd());

    discrete_distribution<int> d(weights.begin(), weights.end());

    for (int n = 0; n < particles.size(); n++) {
        int i = d(gen);
        p_samples.push_back(particles[i]);
    }

    weights.clear();
    particles.clear();
    particles = p_samples;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
