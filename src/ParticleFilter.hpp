#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "DistancePercepts.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

struct Particle {
    int id;
    double x;
    double y;
    double weight;
};

class ParticleFilter {
public:
    ParticleFilter(int num_particles);

    void init(double x, double y, double std[]);

    void predict(double delta_t, double std_pos[], double delta_x, double delta_y);

    // Helper function to calculate distance from a point (px, py) to a line segment (x1, y1) - (x2, y2)
    double distanceToLine(double px, double py, double x1, double y1, double x2, double y2);

    void updateWeights(double sensor_range, double std_landmark[], const Model::PointCloud observations);

    void resample();

    std::vector<Particle> getParticles() const;

private:
    int num_particles;
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen;
};

#endif