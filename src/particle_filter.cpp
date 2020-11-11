/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iterator>
#include <string>
#include <vector>
#include <random>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    std::random_device r;
    std::default_random_engine gen(r());
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    /**
     * Set the number of particles. Initialize all particles to
     *   first position (based on estimates of x, y, theta and their uncertainties
     *   from GPS) and all weights to 1.
     **/
    num_particles = 200;

    // Add random Gaussian noise to each particle.
    for (int i = 0; i < num_particles; ++i) {
        particles.push_back(Particle{i, dist_x(gen), dist_y(gen), dist_theta(gen), 1});
        weights.push_back(1);
    }
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    std::default_random_engine gen;

    // generate random Gaussian noise
    std::normal_distribution<double> dist_x(0, std_pos[0]);
    std::normal_distribution<double> dist_y(0, std_pos[1]);
    std::normal_distribution<double> dist_theta(0, std_pos[2]);
    /**
    * Add measurements to each particle and add random Gaussian noise.
    */
    for (auto &p: particles) {
        if (abs(yaw_rate) < 0) {
            // avoid division by 0
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
        } else {
            p.x += (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            p.theta += yaw_rate * delta_t;
        }
        p.x += dist_x(gen);
        p.y += dist_y(gen);
        p.theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations) {
    for (auto &o: observations) {
        auto minimal_distance = std::numeric_limits<double>::max();
        for (const auto &p: predicted) {
            auto distance = dist(o.x, o.y, p.x, p.y);
            if (distance < minimal_distance) {
                o.id = p.id;
                minimal_distance = distance;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    /**
     * Update the weights of each particle using a multi-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     */
    for (auto &p: particles) {
        p.weight = 1.0;

        // homogeneous transformation
        vector<LandmarkObs> obs_translated;
        for (const auto &obs: observations) {
            LandmarkObs tmp;
            tmp.x = obs.x * cos(p.theta) - obs.y * sin(p.theta) + p.x;
            tmp.y = obs.x * sin(p.theta) + obs.y * cos(p.theta) + p.y;
            tmp.id = obs.id;
            obs_translated.push_back(tmp);
        }
        // put the landmarks into a predictions vector to associate them
        vector<LandmarkObs> predictions;
        for (const auto &lm: map_landmarks.landmark_list) {
            if (dist(p.x, p.y, lm.x_f, lm.y_f) < sensor_range) {
                predictions.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
            }
        }
        // Associate them
        dataAssociation(predictions, obs_translated);

        //Calculate the weights
        for (const auto &o: obs_translated) {
            auto landmark = map_landmarks.landmark_list[o.id - 1];
            auto w = multivariate_prob(std_landmark[0], std_landmark[1], o.x, o.y, landmark.x_f, landmark.y_f);
            p.weight *= w;
        }
        weights.push_back(p.weight);
    }
}

void ParticleFilter::resample() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> discrete_distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles;
    // resample given a discrete distribution position
    for (int i = 0; i < num_particles; i++) {
        resampled_particles.push_back(particles[discrete_distribution(gen)]);
    }
    particles = resampled_particles;
    weights.clear();
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y) {
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
    vector<double> v;

    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

double
ParticleFilter::multivariate_prob(double sig_x, double sig_y, double x_obs, double y_obs, double mu_x, double mu_y) {
    // calculate normalization term
    auto gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

    // calculate exponent
    auto exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
                    + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

    return gauss_norm * exp(-exponent);
}
