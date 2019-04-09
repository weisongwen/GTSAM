/**
 *  @file   PseudorangeFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for pseudorange factor
 **/

#include "PseudorangeFactor_spp.h"

using namespace std;

namespace gtsam {

//***************************************************************************
Vector PseudorangeFactor_spp::evaluateError(const Pose3& q, const Point2& point2_cb,
                     boost::optional<Matrix&> H1,
                     boost::optional<Matrix&> H2) const {

        
        // double r = sqrt( ((satXYZ_.x()-nomXYZ_.x()))*(satXYZ_.x()-nomXYZ_.x()) + ((satXYZ_.y()-nomXYZ_.y())*(satXYZ_.y()-nomXYZ_.y())) + ((satXYZ_.z()-nomXYZ_.z())*(satXYZ_.z()-nomXYZ_.z())) );

        // Vector6 H_matrix_ps; H_matrix_ps << 1*(satXYZ_.x()-nomXYZ_.x())/r, 1*(satXYZ_.y()- nomXYZ_.y())/r,
        // 1*(satXYZ_.z()-nomXYZ_.z())/r, 0.0, 0.0, 0.0;

	    double r = sqrt( ((satXYZ_.x()-q.x()))*(satXYZ_.x()-q.x()) 
	    	+ ((satXYZ_.y()-q.y())*(satXYZ_.y()-q.y())) + ((satXYZ_.z()-q.z())*(satXYZ_.z()-q.z())) );
        Vector6 H_matrix_ps; 
        // H_matrix_ps << 1*(satXYZ_.x()-q.x())/r, 1*(satXYZ_.y()- q.y())/r,
        // 1*(satXYZ_.z()-q.z())/r, 0.0, 0.0, 0.0; 

        H_matrix_ps <<  0.0, 0.0, 0.0, -1*(satXYZ_.x()-q.x())/r, -1*(satXYZ_.y()- q.y())/r,
        -1*(satXYZ_.z()-q.z())/r; 

        Vector h = H_matrix_ps.transpose();
        // std::cout << "h.cols-> "<< h.cols() << "    h.rows -> "<< h.rows() << std::endl; // 1X6
        if (H1) { (*H1) = (Matrix(1,6) << h ).finished(); }


        // Vector6 q_eigen;
        // q_eigen << Vector3(q.x(),q.y(),q.z()), Vector3(0.1,0.1,0.1);
        double est=0;
        if(sv_prn_ <= 32 || sv_prn_ == 84) // GPS 
        {
        	// est = (h.transpose() * q_eigen) + point2_cb(0);
        	est = r + point2_cb.x();
        	double e_psr = est - measured_;
        	if (H2) { (*H2) = (Matrix(1,2) << 1, 0 ).finished(); }
        	// std::cout << "H2.cols-> "<< H2.cols() << "    H2.rows -> "<< H2.rows() << std::endl; // 1X2
        }

        else if((sv_prn_ <= 121) && (sv_prn_ >= 87)) //BeiDou
        {
        	// est = (h.transpose() * q_eigen) + point2_cb(1);
        	est = r +  point2_cb.y();
        	double e_psr = est - measured_;
        	if (H2) { (*H2) = (Matrix(1,2) << 0, 1 ).finished(); }
        }
        else
        {
        	std::cout<<"error in satellite PRN !!!!!"<<std::endl;
        }
        // std::cout<< "prn_-> "<<sv_prn_  << "   rs_-> "<< est - measured_ <<std::endl;

        return (Vector(1) << (est - measured_) ).finished();
}
} // namespace

