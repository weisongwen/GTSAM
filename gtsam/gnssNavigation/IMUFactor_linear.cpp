/**
 *  @file   PseudorangeFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for pseudorange factor
 **/

#include "IMUFactor_linear.h"

using namespace std;

namespace gtsam {

//***************************************************************************
Vector IMUFactor_linear::evaluateError(const Vector3& pre_Vel, const Vector3& Vel, const imuBias::ConstantBias& imu_bias,
                     boost::optional<Matrix&> H1,
                     boost::optional<Matrix&> H2,
                     boost::optional<Matrix&> H3) const {

        
        // double r = sqrt( ((satXYZ_.x()-nomXYZ_.x()))*(satXYZ_.x()-nomXYZ_.x()) + ((satXYZ_.y()-nomXYZ_.y())*(satXYZ_.y()-nomXYZ_.y())) + ((satXYZ_.z()-nomXYZ_.z())*(satXYZ_.z()-nomXYZ_.z())) );

        // Vector6 H_matrix_ps; H_matrix_ps << 1*(satXYZ_.x()-nomXYZ_.x())/r, 1*(satXYZ_.y()- nomXYZ_.y())/r,
        Matrix33 H1_matrix =Matrix33::Identity(3,3) * -1 * (1.0/ delta_t);
        Matrix33 H2_matrix =Matrix33::Identity(3,3) * 1 * (1.0/ delta_t);
        Matrix H3_matrix =Matrix::Zero(3,6);
        H3_matrix(0,0) = 1;
        H3_matrix(1,1) = 1;
        H3_matrix(2,2) = 1;
        // cout<< "H3_matrix-> " << H3_matrix << endl; 

        // 1*(satXYZ_.z()-nomXYZ_.z())/r, 0.0, 0.0, 0.0;
        if (H1) { (*H1) = (Matrix(3,3) << H1_matrix).finished(); }
        if (H2) { (*H2) = (Matrix(3,3) << H2_matrix).finished(); }
        if (H3) { (*H3) = (Matrix(3,6) << H3_matrix).finished(); }

        Vector3 biasAcc = imu_bias.accelerometer();
        double acc_x_estimated = ((Vel(0) - pre_Vel(0)) / delta_t) + biasAcc(0); // estimated accx
        double acc_y_estimated = ((Vel(1) - pre_Vel(1)) / delta_t) + biasAcc(1); // estimated accy 
        double acc_z_estimated = ((Vel(2) - pre_Vel(2)) / delta_t) + biasAcc(2); // estimated accz

        // cout<< "acc_x_estimated - ACC_.x()-> " << acc_x_estimated - ACC_.x() << endl; 
	    
        return (Vector(3) << (acc_x_estimated - ACC_.x()), (acc_y_estimated - ACC_.y()), (acc_z_estimated - ACC_.z()) ).finished();
}
} // namespace

