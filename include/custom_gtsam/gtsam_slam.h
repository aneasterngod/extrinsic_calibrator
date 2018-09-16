#ifndef Gtsam_SLAM_H_
#define Gtsam_SLAM_H_
// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
// trust-region method known as Powell's Degleg
#include <gtsam/nonlinear/DoglegOptimizer.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>
// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

#include <vector>

using namespace std;
using namespace gtsam;

class GtsamSLAM
{
  public:
    GtsamSLAM();
    ~GtsamSLAM();
    void initialization(const Eigen::Matrix3d &K);
    void addPose(const Pose3 &pose, const vector<Point3> &obs, const int &sequenceid);

  private:
    // Define the camera calibration parameters
    // Cal3_S2::shared_ptr m_K;
    // // Define the camera observation noise model
    // noiseModel::Isotropic::shared_ptr m_measurementNoise;

    // // Create a factor graph
    // NonlinearFactorGraph m_slamgraph;
    // noiseModel::Diagonal::shared_ptr m_poseNoise;
    // ISAM2Params m_isamparameters;
    // std::shared_ptr<ISAM2> m_isam;
    // Values m_initialEstimate;
};

#endif