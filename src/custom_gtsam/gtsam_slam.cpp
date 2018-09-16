#include "custom_gtsam/gtsam_slam.h"

GtsamSLAM::GtsamSLAM()
{
}

GtsamSLAM::~GtsamSLAM()
{
}

// void GtsamSLAM::initialization(const Eigen::Matrix3d &K)
// {
//     m_K = Cal3_S2::shared_ptr(new Cal3_S2(K(0, 0), K(1, 1), 0.0, K(0, 2), K(1, 2)));
//     m_poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
//     m_measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);                                                            // one pixel in u and v
//     m_isamparameters.relinearizeThreshold = 0.01;
//     m_isamparameters.relinearizeSkip = 1;
//     m_isam = std::shared_ptr<ISAM2>(new ISAM2(m_isamparameters));
// }

// void GtsamSLAM::addPose(const Pose3 &pose, const vector<Point3> &obs, const int &sequenceid)
// {
//     // Add factors for each landmark observation
//     // Add initial guesses to all observed landmarks
//     for (size_t i = 0; i < obs.size(); ++i)
//     {
//         SimpleCamera camera(pose, *m_K);
//         Point2 measurement = camera.project(obs[i]);
//         m_slamgraph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(measurement, m_measurementNoise, Symbol('x', sequenceid), Symbol('l', i), m_K);
//     }
//     m_initialEstimate.insert(Symbol('x', sequenceid), pose);
//     if (sequenceid == 0)
//     {
//         // Add a prior on pose x0
//         Eigen::Matrix<double, 3, 4> initmat;
//         initmat << 1, 0, 0, 0,
//             0, 1, 0, 0,
//             0, 0, 1, 0;
//         Pose3 initialpose(initmat);
//         m_slamgraph.emplace_shared<PriorFactor<Pose3>>(Symbol('x', 0), initialpose, m_poseNoise); // add directly to graph

//         // Add a prior on landmark l0
//         noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
//         m_slamgraph.emplace_shared<PriorFactor<Point3>>(Symbol('l', 0), obs[0], pointNoise); // add directly to graph

//         // Add initial guesses to all observed landmarks
//         for (size_t i = 0; i < obs.size(); ++i)
//         {
//             m_initialEstimate.insert<Point3>(Symbol('l', i), obs[i]);
//         }
//     }
//     else
//     {
//         // Update iSAM with the new factors
//         m_isam->update(m_slamgraph, m_initialEstimate);
//         // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
//         // If accuracy is desired at the expense of time, update(*) can be called additional times
//         // to perform multiple optimizer iterations every step.
//         m_isam->update();
//         m_initialEstimate.clear();
//     }
//}