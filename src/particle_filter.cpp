/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
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

static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[])
{

  if (is_initialized == false)
  {
    num_particles = 101; //  Set the number of particles
    std::default_random_engine gen;

    // Create the normal distribution for the noise
    std::normal_distribution<double> dist_x(0, std[0]);
    std::normal_distribution<double> dist_y(0, std[1]);
    std::normal_distribution<double> dist_theta(0, std[2]);

    // Create the sample particles
    for (int i = 0; i < num_particles; i++)
    {
      Particle sampleParticle;
      sampleParticle.x = x + dist_x(gen);
      sampleParticle.y = y + dist_y(gen);
      sampleParticle.theta = theta + dist_theta(gen);
      sampleParticle.weight = 1.0;
      particles.push_back(sampleParticle);
    }
    // Set the filter to initialzed
    is_initialized = true;
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);
  for (std::size_t i = 0; i < particles.size(); i++)
  {
    // calculate new state
    if (fabs(yaw_rate) < 0.00001)
    {
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    else
    {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // add noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations)
{
  for (std::size_t i = 0; i < observations.size(); i++)
  {
    double distance = 10000000000000;
    int map_id = -1;
    for (std::size_t j = 0; j < predicted.size(); j++)
    {
      double calculated_distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if (calculated_distance < distance)
      {
        distance = calculated_distance;
        map_id = predicted[j].id;
      }
    }
    observations[i].id = map_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  for (std::size_t i = 0; i < particles.size(); i++)
  {
    Particle particle = particles[i];
    vector<LandmarkObs> predicted;
    // Find the predicted landmarks from the map in the given sensor range.
    for (std::size_t k = 0; k < map_landmarks.landmark_list.size(); k++)
    {
      double distance = dist(particle.x, particle.y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
      if (distance <= sensor_range)
      {
        LandmarkObs landmark;
        landmark.x = map_landmarks.landmark_list[k].x_f;
        landmark.y = map_landmarks.landmark_list[k].y_f;
        landmark.id = map_landmarks.landmark_list[k].id_i;
        predicted.push_back(landmark);
      }
    }
    vector<LandmarkObs> observation_map;
    // Transfer the observations to map coordinates and calculate the weight
    for (std::size_t j = 0; j < observations.size(); j++)
    {
      double x_map = particle.x + (cos(particle.theta) * observations[j].x) - (sin(particle.theta) * observations[j].y);
      double y_map = particle.y + (sin(particle.theta) * observations[j].x) + (cos(particle.theta) * observations[j].y);
      LandmarkObs landmark;
      landmark.id = observations[j].id;
      landmark.x = x_map;
      landmark.y = y_map;
      observation_map.push_back(landmark);
    }
    // Find the nearest Landmark to the given object
    dataAssociation(predicted, observation_map);
    // calculate the weight
    particles[i].weight = 1.0;
    for (std::size_t l = 0; l < observation_map.size(); l++)
    {
      for (std::size_t m = 0; m < predicted.size(); m++)
      {
        if (predicted[m].id == observation_map[l].id)
        {
          particles[i].weight *= multiv_prob(std_landmark[0], std_landmark[1],
                                             observation_map[l].x, observation_map[l].y, predicted[m].x, predicted[m].y);
        }
      }
    }
  }
}

void ParticleFilter::resample()
{
  std::random_device rd;
  std::vector<double> weights;
  std::vector<Particle> new_particles;
  for (std::size_t i = 0; i < particles.size(); i++)
  {
    weights.push_back(particles[i].weight);
  }
  std::discrete_distribution<> d(weights.begin(), weights.end());
  // Set of new list of particles
  int new_id = -1;
  // Sample new particles
  for (std::size_t i = 0; i < particles.size(); i++)
  {
    Particle new_particle;
    new_id = d(gen);
    new_particle.x = particles[new_id].x;
    new_particle.y = particles[new_id].y;
    new_particle.theta = particles[new_id].theta;
    new_particle.weight = particles[new_id].weight;
    new_particle.associations = particles[new_id].associations;
    new_particle.sense_x = particles[new_id].sense_x;
    new_particle.sense_y = particles[new_id].sense_y;
    new_particles.push_back(new_particle);
  }
  particles = new_particles;
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
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
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
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}