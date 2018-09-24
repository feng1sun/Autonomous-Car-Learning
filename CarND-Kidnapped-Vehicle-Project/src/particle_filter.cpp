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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if(!is_initialized){
		num_particles = 500;
		normal_distribution<double> dist_x(x, std[0]);
		normal_distribution<double> dist_y(y, std[1]);
		normal_distribution<double> dist_theta(theta, std[2]);
		default_random_engine gen;
		weights.resize(num_particles, 1.0f);
		for (int i=0; i<num_particles; i++){
			Particle p;
			p.id = i;
			p.x = dist_x(gen);
			p.y = dist_y(gen);
			p.theta = dist_theta(gen);
			p.weight = 1.0f;

			particles.push_back(p);
		}
		is_initialized = true;
	}
	else{
		return;
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);
	default_random_engine gen;
	for (int i=0; i<num_particles; i++){
		double theta = particles[i].theta;
		double noise_x = dist_x(gen);
		double noise_y = dist_y(gen);
		double noise_theta = dist_theta(gen);
		if(abs(yaw_rate) < 0.00001){
			particles[i].x += (velocity * delta_t * cos(theta) + noise_x);
			particles[i].y += (velocity * delta_t * sin(theta) + noise_y);
			particles[i].theta += noise_theta;
		}else{
			double phi_theta = theta + delta_t * yaw_rate;
			particles[i].x += (velocity / yaw_rate * (sin(phi_theta) - sin(theta)) + noise_x);	
			particles[i].y += (velocity / yaw_rate * (cos(theta) - cos(phi_theta)) + noise_y);	
      particles[i].theta += (delta_t * yaw_rate + noise_theta);
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i=0; i<observations.size(); i++){
		int p_id = -1;	
		double min_dist = numeric_limits<double>::max();
		for (int j=0; j<predicted.size(); j++){
			double delta_x = predicted[j].x - observations[i].x;
			double delta_y = predicted[j].y - observations[i].y;
			double dist = delta_x * delta_x + delta_y * delta_y;
			if (dist < min_dist){
				p_id = predicted[j].id;
				min_dist = dist;
			}
		}
		observations[i].id = p_id;
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
		double std_x = std_landmark[0];
		double std_y = std_landmark[1];
		double d = sqrt(2.0 * M_PI * std_x * std_y);
		double a_d = 2 * std_x * std_x;
		double b_d = 2 * std_y * std_y;
		for (int i=0; i<num_particles; i++){
			double p_x = particles[i].x;
			double p_y = particles[i].y;
			double p_theta = particles[i].theta;
			vector<LandmarkObs> t_landmark;
			for (int j=0; j<observations.size(); j++){
				int obs_id = observations[j].id;
				double obs_x = observations[j].x;
				double obs_y = observations[j].y;
				double t_x = p_x + obs_x * cos(p_theta) - obs_y * sin(p_theta);
				double t_y = p_y + obs_y * cos(p_theta) + obs_x * sin(p_theta);
				LandmarkObs temp;
				temp.id = obs_id;
				temp.x = t_x;
				temp.y = t_y;
				t_landmark.push_back(temp);
			}

			vector<LandmarkObs> landmark_in_range;
			for(int j=0; j<map_landmarks.landmark_list.size(); j++){
				int l_id = map_landmarks.landmark_list[j].id_i;
				double l_x = map_landmarks.landmark_list[j].x_f;
				double l_y = map_landmarks.landmark_list[j].y_f;
				double dist_x = l_x - p_x;
				double dist_y = l_y - p_y;
				double dist = sqrt(dist_x * dist_x + dist_y * dist_y);
				if(dist <= sensor_range){
					LandmarkObs temp;
					temp.id = l_id;
					temp.x = l_x;
					temp.y = l_y;
					landmark_in_range.push_back(temp);
				}
			}

			dataAssociation(landmark_in_range, t_landmark);
			particles[i].weight = 1.0;

			double w = 1.0;
			for (int j=0; j<t_landmark.size(); j++){
				int obs_id = t_landmark[j].id;
				double obs_x = t_landmark[j].x;
				double obs_y = t_landmark[j].y;
				double x = 0; 
				double y = 0; 
				for (int k=0; k<landmark_in_range.size(); k++){
					if(landmark_in_range[k].id == obs_id){
						x = landmark_in_range[k].x;
						y = landmark_in_range[k].y;
						break;
					}
				}
				double d_x = obs_x - x;
				double d_y = obs_y - y;
				double a = d_x * d_x / a_d;
				double b = d_y * d_y / b_d;
				w *= exp(-(a+b))/d;
			 	particles[i].weight = w;
			}
			// if(w == 0){
			// 	particles[i].weight = 0.00001;
			// 	weights[i] = 0.00001;
			// }else{
			// 	particles[i].weight = w;
			// 	weights[i] = w;
			// }
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	// vector<Particle> sample_p;
	// default_random_engine gen;
	// discrete_distribution<int> index(weights.begin(), weights.end());

	vector<Particle> sample_p;
	vector<double> weights;
	default_random_engine gen;
	for(int i=0; i<num_particles; i++){
		weights.push_back(particles[i].weight);
	}
	discrete_distribution<int> dist(0, num_particles);
	int index = dist(gen);
	double max_weight = *max_element(weights.begin(), weights.end());
	uniform_real_distribution<double> real_dist(0.0, 1.0);
	double beta = 0.0;
	for (int i=0; i<num_particles; i++){
		beta += real_dist(gen) * 2.0 * max_weight;
		while(beta > weights[index]){
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		sample_p.push_back(particles[index]);
	}
	particles = sample_p;
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
