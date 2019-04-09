/**
 *  @file   PseudorangeFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for pseudorange factor
 **/

#include "MotionModel_factor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
Vector MotionModel_factor::evaluateError(const Pose3& pre_pose, const Pose3& pose, const Vector3& pre_vel,
                     boost::optional<Matrix&> H1,
                     boost::optional<Matrix&> H2,
                     boost::optional<Matrix&> H3) const {

        
        // double r = sqrt( ((satXYZ_.x()-nomXYZ_.x()))*(satXYZ_.x()-nomXYZ_.x()) + ((satXYZ_.y()-nomXYZ_.y())*(satXYZ_.y()-nomXYZ_.y())) + ((satXYZ_.z()-nomXYZ_.z())*(satXYZ_.z()-nomXYZ_.z())) );

        // Vector6 H_matrix_ps; H_matrix_ps << 1*(satXYZ_.x()-nomXYZ_.x())/r, 1*(satXYZ_.y()- nomXYZ_.y())/r,

        double error_x = pose.translation().x() - (pre_pose.translation().x() + pre_vel(0) * delta_t); // estimated - measure
        double error_y = pose.translation().y() - (pre_pose.translation().y() + pre_vel(1) * delta_t);
        double error_z = pose.translation().z() - (pre_pose.translation().z() + pre_vel(2) * delta_t);

        Matrix H1_matrix =Matrix::Zero(3,6);
        H1_matrix(0,0) = -1;
        H1_matrix(1,1) = -1;
        H1_matrix(2,2) = -1;

        Matrix H2_matrix =Matrix::Zero(3,6);
        H2_matrix(0,0) = 1;
        H2_matrix(1,1) = 1;
        H2_matrix(2,2) = 1;

        Matrix H3_matrix =Matrix::Zero(3,3);
        H3_matrix(0,0) = -1 * delta_t;
        H3_matrix(1,1) = -1 * delta_t;
        H3_matrix(2,2) = -1 * delta_t;

        // cout<< "H3_matrix-> " << H3_matrix << endl; 

        if (H1) { (*H1) = (Matrix(3,6) << H1_matrix).finished(); }
        if (H2) { (*H2) = (Matrix(3,6) << H2_matrix).finished(); }
        if (H3) { (*H3) = (Matrix(3,3) << H3_matrix).finished(); }

        // cout<< "acc_x_estimated - ACC_.x()-> " << acc_x_estimated - ACC_.x() << endl; 
        // measured_(0) = error_x;
        // measured_(1) = error_y;
        // measured_(2) = error_z;
	    
        return (Vector(3) << (error_x), (error_y), (error_z) ).finished();
}
} // namespace

