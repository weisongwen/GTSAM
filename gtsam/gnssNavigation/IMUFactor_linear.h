/**
 *  @file   IMUFactor_linear.h
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

class GTSAM_EXPORT IMUFactor_linear : public NoiseModelFactor3<Vector3, Vector3, imuBias::ConstantBias> {

private:
typedef NoiseModelFactor3<Vector3, Vector3, imuBias::ConstantBias> Base;
Point3 nomXYZ_;
Point3 satXYZ_;

Point2 cb;
Pose3 h_;
Point3 measured_;

public:
    double sv_prn_;
    Point3 ACC_;
    double delta_t;

typedef boost::shared_ptr<IMUFactor_linear> shared_ptr;
typedef IMUFactor_linear This;

IMUFactor_linear() : measured_(0,0,0) {
        // h_=(Matrix(1,6)<<1,1,1,1,1,1).finished();
}

virtual ~IMUFactor_linear() {
}

IMUFactor_linear(Key key_pre_vel_vector3, Key key_vel_vector3, Key key_imu_bias, const Point3 ACC, double dt, const SharedNoiseModel& model) :
        Base(model, key_pre_vel_vector3, key_vel_vector3, key_imu_bias) {
          measured_ = ACC;
          delta_t = dt; // time bias 
}


virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new IMUFactor_linear(*this)));
}

/// vector of errors
Vector evaluateError(const Vector3& pre_Vel, const Vector3& Vel, const imuBias::ConstantBias& imu_bias,
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

}; // IMUFactor_linear Factor
} // namespace
