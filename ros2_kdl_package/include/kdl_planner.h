#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <stdio.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

class KDLPlanner
{

public:

    KDLPlanner();
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius,KDL::Frame _EEInitFrame);
    KDLPlanner(double _trajDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);




    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    //////////////////////////////////

    trajectory_point compute_trajectory(double time,std::string traj_type );

    void compute_trapezoidal_velocity_point(double t, double tc,double & s,double & sdot,double & sdotdot);
    void cubic_polynomial(double t,double & s,double & sdot,double & sdotdot);
    trajectory_point compute_trajectoryTrapezoidal(double time, double tc, std::string traj_type );
private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_, accDuration_,a0_,a1_,a2_,a3_, trajRadius_;
    
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;
    KDL::Frame EEInitFrame_;//to retrieve the center of the circular path

};

#endif
