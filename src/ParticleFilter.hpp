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
    ParticleFilter(int num_particles) : num_particles(num_particles), gen(rd()) {}

    void init(double x, double y, double std[]) {
        std::normal_distribution<double> dist_x(x, std[0]);
        std::normal_distribution<double> dist_y(y, std[1]);
        
        particles.clear();
        for (int i = 0; i < num_particles; ++i) {
            Particle particle;
            particle.id = i;
            particle.x = dist_x(gen);
            particle.y = dist_y(gen);
            particle.weight = 1.0;
            particles.push_back(particle);
        }
    }

    void predict(double delta_t, double std_pos[], double delta_x, double delta_y) {
        std::normal_distribution<double> dist_x(0, std_pos[0]);
        std::normal_distribution<double> dist_y(0, std_pos[1]);
        
        for (auto& particle : particles) {
            // Update particle position based on delta_x and delta_y inputs
            particle.x += delta_x * delta_t + dist_x(gen);
            particle.y += delta_y * delta_t + dist_y(gen);
        }
    }

    // Helper function to calculate distance from a point (px, py) to a line segment (x1, y1) - (x2, y2)
    double distanceToLine(double px, double py, double x1, double y1, double x2, double y2) {
        double A = px - x1;
        double B = py - y1;
        double C = x2 - x1;
        double D = y2 - y1;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = (len_sq != 0) ? (dot / len_sq) : -1;

        double closest_x, closest_y;

        if (param < 0) {
            closest_x = x1;
            closest_y = y1;
        } else if (param > 1) {
            closest_x = x2;
            closest_y = y2;
        } else {
            closest_x = x1 + param * C;
            closest_y = y1 + param * D;
        }

        return std::sqrt((px - closest_x) * (px - closest_x) + (py - closest_y) * (py - closest_y));
    }

    void updateWeights(double sensor_range, double std_landmark[], const Model::PointCloud observations) {
        std::vector<Model::WallPtr> landmarks = Model::RobotWorld::getRobotWorld().getWalls();
        for (auto& particle : particles) {
            double weight = 1.0;
            for (const auto& obs : observations) {
                // Transform observation to map coordinates
                double obs_x = particle.x + obs.distance * cos(obs.angle);
                double obs_y = particle.y + obs.distance * sin(obs.angle);

                // Find the closest distance from observation point to any landmark line segment
                double min_dist = sensor_range;
                for (const auto& landmark : landmarks) {
                    double distance = distanceToLine(obs_x, obs_y, landmark->getPoint1().x, landmark->getPoint1().y, landmark->getPoint2().x, landmark->getPoint2().y);
                    if (distance < min_dist) {
                        min_dist = distance;
                    }
                }

                // If the observation is within sensor range, calculate the weight based on its proximity to the nearest line
                if (min_dist <= sensor_range) {
                    double obs_weight = (1 / (sqrt(2 * M_PI) * std_landmark[0])) *
                                        exp(- (min_dist * min_dist) / (2 * std_landmark[0] * std_landmark[0]));
                    weight *= obs_weight;
                }
            }
            particle.weight = weight;
        }
    }

    void resample() {
        std::vector<Particle> new_particles;
        std::vector<double> weights;
        for (const auto& particle : particles) {
            weights.push_back(particle.weight);
        }

        std::discrete_distribution<int> dist(weights.begin(), weights.end());
        
        for (int i = 0; i < num_particles; ++i) {
            new_particles.push_back(particles[dist(gen)]);
        }
        particles = new_particles;
    }

    std::vector<Particle> getParticles() const {
        return particles;
    }

private:
    int num_particles;
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen;
};