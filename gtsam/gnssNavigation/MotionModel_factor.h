/**
 *  @file   MotionModel_factor.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for Pseudorange factor
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point4.h>
#include <gtsam/geometry/Point2.h>

#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

namespace gtsam {

class GTSAM_EXPORT MotionModel_factor : public NoiseModelFactor3<Pose3, Pose3, Vector3> {

private:
    typedef NoiseModelFactor3<Pose3, Pose3, Vector3> Base; 
    Point3 nomXYZ_;
    Point3 satXYZ_;

    Point2 cb;
    Pose3 h_;
    Point3 measured_;

public:
    double sv_prn_;
    Point3 ACC_;
    double delta_t;

typedef boost::shared_ptr<MotionModel_factor> shared_ptr;
typedef MotionModel_factor This;

MotionModel_factor() : measured_(0,0,0) {
        // h_=(Matrix(1,6)<<1,1,1,1,1,1).finished();
}

virtual ~MotionModel_factor() {
}

MotionModel_factor(Key key_pre_pose, Key key_pose, Key key_pre_vel, double dt, const SharedNoiseModel& model) :
        Base(model, key_pre_pose, key_pose, key_pre_vel) {
          delta_t = dt; // time bias 
          
          std::cout<< "construct function of motion model-> " <<std::endl; 
}


virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new MotionModel_factor(*this)));
}

/// vector of errors
Vector evaluateError(const Pose3& pre_pose, const Pose3& pose, const Vector3& pre_vel,
                     boost::optional<Matrix&> H1 = boost::none,
                     boost::optional<Matrix&> H2 = boost::none,
                     boost::optional<Matrix&> H3 = boost::none) const;


private:

/// Serialization function
friend class boost::serialization::access;
template<class ARCHIVE>
void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar
        & boost::serialization::make_nvp("NoiseModelFactor3",
                                         boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measured_);
}

}; // MotionModel_factor Factor
} // namespace
