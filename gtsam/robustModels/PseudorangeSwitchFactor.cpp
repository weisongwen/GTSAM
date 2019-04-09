/**
 *  @file   PseudorangeSwitchFactor.cpp
 *  @author Ryan Waton and Jason Gross
 *  @brief  Implementation file for pseudorange switchable factor
 **/

#include "PseudorangeSwitchFactor.h"

using namespace std;
using namespace vertigo;

namespace gtsam {

//***************************************************************************
Vector PseudorangeSwitchFactor::evaluateError(const Pose3& q, const Point2& point2_cb,
                                              const SwitchVariableLinear& sw,
                                              boost::optional<Matrix&> H1,
                                              boost::optional<Matrix&> H2,
                                              boost::optional<Matrix&> H3) const {

        // Vector h_ = obsMap(satXYZ_, nomXYZ_, 1);
        // double error = (h_.transpose()*q)-measured_;
        // error *= s.value();
        // if (H1) { (*H1) = (Matrix(1,5) << h_.transpose() * s.value() ).finished(); }
        // if (H2) { (*H2) = (Vector(1) << error).finished(); }
        // return (Vector(1) << error).finished();




        double r = sqrt( ((satXYZ_.x()-q.x()))*(satXYZ_.x()-q.x()) 
	    	+ ((satXYZ_.y()-q.y())*(satXYZ_.y()-q.y())) + ((satXYZ_.z()-q.z())*(satXYZ_.z()-q.z())) );
        Vector6 H_matrix_ps; 
        // H_matrix_ps << 1*(satXYZ_.x()-q.x())/r, 1*(satXYZ_.y()- q.y())/r,
        // 1*(satXYZ_.z()-q.z())/r, 0.0, 0.0, 0.0; 

        H_matrix_ps <<  0.0, 0.0, 0.0, (-1*(satXYZ_.x()-q.x())/r) * sw.value(),
         (-1*(satXYZ_.y()- q.y())/r) * sw.value(),
        (-1*(satXYZ_.z()-q.z())/r) * sw.value(); 

        Vector h = H_matrix_ps.transpose();
        // std::cout << "h.cols-> "<< h.cols() << "    h.rows -> "<< h.rows() << std::endl; // 1X6
        if (H1) { (*H1) = (Matrix(1,6) << h ).finished(); }


        double est=0;
        if(sv_prn_ <= 32 || sv_prn_ == 84) // GPS 
        {
        	// est = (h.transpose() * q_eigen) + point2_cb(0);
        	est = r + point2_cb.x();
        	double e_psr = est - measured_;
        	if (H2) { (*H2) = (Matrix(1,2) << (sw.value()), 0 ).finished(); }
        	// std::cout << "H2.cols-> "<< H2.cols() << "    H2.rows -> "<< H2.rows() << std::endl; // 1X2
        }

        else if((sv_prn_ <= 121) && (sv_prn_ >= 87)) //BeiDou
        {
        	// est = (h.transpose() * q_eigen) + point2_cb(1);
        	est = r +  point2_cb.y();
        	double e_psr = est - measured_;
        	if (H2) { (*H2) = (Matrix(1,2) << 0, (sw.value()) ).finished(); }
        }
        else
        {
        	std::cout<<"error in satellite PRN !!!!!"<<std::endl;
        }
        // std::cout<< "prn_-> "<<sv_prn_  << "   rs_-> "<< est - measured_ <<std::endl;

        if (H3) { (*H3) = (Vector(1) << (est - measured_)).finished(); }

        return (Vector(1) << ((est - measured_) * sw.value()) ).finished();
}
} // namespace
