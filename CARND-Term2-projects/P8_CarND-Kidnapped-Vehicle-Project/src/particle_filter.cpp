/////////////////////////////////////////////////////////////
//
//   particle_filter.cpp
//
//////////////////////////////////////////////////////////////
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

const int NUM_PARTICLES = 100;
static default_random_engine gen;

/////////////////////////////////////////////////////////////
//   init
/////////////////////////////////////////////////////////////
void ParticleFilter::init(double x, double y, double theta, double std[]) {
    //cout <<"---------------------Init ------------------------"<< endl;
    
    if (is_initialized) {
        return;
    }

    num_particles = NUM_PARTICLES;
    
    weights = vector<double>(num_particles);
    //particles = vector<Particle>(num_particles);

    // Create normal (Gaussian) distributions for x, y, theta
    default_random_engine gen;
    
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    for (int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y= dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0f;
        
        particles.push_back(p);
        weights.push_back(1.0f);
    }
    
    is_initialized = true;
}


/////////////////////////////////////////////////////////////
//   prediction
/////////////////////////////////////////////////////////////
void ParticleFilter::prediction(double delta_t,
                                double std_pos[],
                                double velocity,
                                double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
    //cout <<"---------------------Prediction Calc------------------------"<< endl;

    default_random_engine gen;
    for (int i = 0; i < num_particles; i++) {
        
        double pred_x;
        double pred_y;
        double pred_theta;

        if (fabs(yaw_rate) < 1e-10) {
            pred_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            pred_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
            pred_theta = particles[i].theta;
        }
        else {
            pred_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            pred_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t) );
            pred_theta = particles[i].theta + yaw_rate * delta_t;
        }
        
        normal_distribution<double> gauss_noise_x(pred_x, std_pos[0]);
        normal_distribution<double> gauss_noise_y(pred_y, std_pos[1]);
        normal_distribution<double> gauss_noise_theta(pred_theta, std_pos[2]);

        // Add noise
        particles[i].x =  gauss_noise_x(gen);
        particles[i].y =  gauss_noise_y(gen);
        particles[i].theta =  gauss_noise_theta(gen);
    }
}

/////////////////////////////////////////////////////////////
//   dataAssociation
/////////////////////////////////////////////////////////////X
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement closest to each observed measurement and assign the
	// observed measurement to this particular landmark.

    int iterations = predicted.size();
    
    for (int i=0; i < observations.size(); i++)
    {
        int map_elem = -1;

        LandmarkObs obs = observations[i];
        double nearest = numeric_limits<double>::max();

        for (int j=0; j < iterations; j++)
        {
            double dx = predicted[j].x - obs.x;
            double dy = predicted[j].y - obs.y;
            double dist = sqrt(dx * dx + dy * dy);
            
            if (dist < nearest)
            {
                //map_elem = predicted[j].id;
                map_elem = j; // store index of landmark instead of id field)
                nearest = dist;
            }
        }
        observations[i].id = map_elem;
        //cout << "index: (" << observations[i].id << ") | observation(x,y): " << "["<< observations[i].x << "," << observations[i].y << "] | predicted(x,y) [" << predicted[map_elem].x << "," << predicted[map_elem].y << "] | nearest: " << nearest << endl;
    }
}

/////////////////////////////////////////////////////////////
//   updateWeights
/////////////////////////////////////////////////////////////
// Update the weights of each particle using a mult-variate Gaussian distribution.

void ParticleFilter::updateWeights(double sensor_range,
                                   double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {

    double sig_x_square = std_landmark[0] * std_landmark[0];
    double sig_y_square = std_landmark[1] * std_landmark[1];
    double sig_xy = std_landmark[0] * std_landmark[1];

    for (int i = 0; i < num_particles; i++) {
        
        ///////////////////////////////////////////////////////////////////////////////////////////
        // STEP 1: transform from car coordinates to map coordinates all observations (for current particle)
        ///////////////////////////////////////////////////////////////////////////////////////////
        //cout << "----STEP 1------------Obs Transformations----------------- " << endl;
        
        auto p = particles[i];
        vector<LandmarkObs> obs_transformed; // transformed Observations Vector
        LandmarkObs obs;
        
        for (int j = 0; j < observations.size(); j++)
        {
            LandmarkObs t_obs;                              //  single transformed observation
            obs = observations[j];
            
            t_obs.x = particles[i].x + obs.x * cos(particles[i].theta) - obs.y * sin(particles[i].theta);
            t_obs.y = particles[i].y + obs.x * sin(particles[i].theta) + obs.y * cos(particles[i].theta);
            t_obs.id = obs.id;
            
            obs_transformed.push_back(t_obs);
            
            //cout << "[" << obs.x << " " << obs.y << "] " << "-> " << "[" << t_obs.x << " " << t_obs.y << "]" << endl;
        }
        
        particles[i].weight = 1.0f;
        weights[i] = 1.0f;

        
        ///////////////////////////////////////////////////////////////////////////////////////////
        // STEP 2: restrict to only landmarks within the sensor range
        ///////////////////////////////////////////////////////////////////////////////////////////
        //cout<<"----STEP 2------------Restrict landmarks in range----------------- " << i << endl;
        
        vector<LandmarkObs> landmarks_within_range;

        for (int m = 0;  m < map_landmarks.landmark_list.size(); m++)
        {
            
            auto lm = map_landmarks.landmark_list[m];
            
            LandmarkObs lm_pred;
            lm_pred.x = lm.x_f;
            lm_pred.y = lm.y_f;
            lm_pred.id = lm.id_i;
            
            //double dx = lm_pred.x - obs_transformed[i].x;
            //double dy = lm_pred.y - obs_transformed[i].y;
            double dx = lm_pred.x - p.x;
            double dy = lm_pred.y - p.y;
            
            double dist = sqrt(dx * dx + dy * dy);
            
            // in range ?
            if (dist <= sensor_range)
            {
                landmarks_within_range.push_back(lm_pred);
                //cout << "landmark in range: [" << lm_pred.x << " " << lm_pred.y << " " << lm_pred.id << "]" << endl;
            }
        }
        
        ///////////////////////////////////////////////////////////////////////////////////////////
        // STEP 3: find closest predicted measurement each observed measurement and assign to this landmark
        ///////////////////////////////////////////////////////////////////////////////////////////
        //cout << "----STEP 3------------Data Association------------------------" << endl;
        dataAssociation(landmarks_within_range, obs_transformed);
        
        ///////////////////////////////////////////////////////////////////////////////////////////
        // STEP 4: calculate probability for each observation and multply them to find total probability
        ///////////////////////////////////////////////////////////////////////////////////////////
        //cout << "----STEP 4------------Weight Calculaction------------------------" << endl;
        
        double w = 1;

        for (int t = 0; t < obs_transformed.size(); t++)
        {
            int oid = obs_transformed[t].id; //-1;
            double ox = obs_transformed[t].x;
            double oy = obs_transformed[t].y;
            
            double predicted_x = landmarks_within_range[oid].x;
            double predicted_y = landmarks_within_range[oid].y;
            
            double x_diff_square = pow((ox - predicted_x), 2);
            double y_diff_square = pow((oy - predicted_y), 2);
            
            double gauss_norm = 1 / (2 * M_PI * sig_xy);
            double exponent = x_diff_square / (2 * sig_x_square) + y_diff_square / (2 * sig_y_square);
            double prob = gauss_norm * exp(-exponent);
            
            w *= prob;
        }
        
        particles[i].weight = w;
        weights[i] = w;

    }
    
}

/////////////////////////////////////////////////////////////
//   resample
/////////////////////////////////////////////////////////////
void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
    //cout << "-----------------------Resample------------------------" << endl;
    
 
    vector<Particle> new_particles;
    
    vector<double> weights;
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }
    
    uniform_int_distribution<int> uniintdist(0, num_particles-1);
    int index = uniintdist(gen);
    double max_weight = *max_element(weights.begin(), weights.end());
    
    // uniform random distribution [0...max_weight]
    uniform_real_distribution<double> unirealdist(0.0, max_weight);
    
    double beta = 0.0;
    for (int i = 0; i < num_particles; i++) {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }
    particles = new_particles;
}

/////////////////////////////////////////////////////////////
//   SetAssociations
/////////////////////////////////////////////////////////////
Particle ParticleFilter::SetAssociations(Particle& particle,
                                         const std::vector<int>& associations,
                                         const std::vector<double>& sense_x,
                                         const std::vector<double>& sense_y) {
    // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    //   associations: The landmark id that goes along with each listed association
    //   sense_x: the associations x mapping already converted to world coordinates
    //   sense_y: the associations y mapping already converted to world coordinates
    
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();
    
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    
    return particle;
    
}

/////////////////////////////////////////////////////////////
//   getAssociations
/////////////////////////////////////////////////////////////
string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

/////////////////////////////////////////////////////////////
//   getSenseX
////////////////////////////////////////////////////////////
string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

/////////////////////////////////////////////////////////////
//   getSenseY
/////////////////////////////////////////////////////////////
string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
