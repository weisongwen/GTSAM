/**
 *  @file   PseudorangeSwitchFactor.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for Pseudorange Switchable factor
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/robustModels/switchVariableLinear.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>

using namespace std;
using namespace vertigo;

namespace gtsam {

class GTSAM_EXPORT PseudorangeSwitchFactor : public NoiseModelFactor3<Pose3, Point2,SwitchVariableLinear> {

private:
typedef NoiseModelFactor3<Pose3, Point2,SwitchVariableLinear> Base;
// Point3 nomXYZ_;
// Point3 satXYZ_;
// double measured_;
// nonBiasStates h_;

Point3 nomXYZ_;
Point3 satXYZ_;
Point2 cb;
Pose3 h_;
double measured_;
public:
    double sv_prn_;

public:

typedef boost::shared_ptr<PseudorangeSwitchFactor> shared_ptr;
typedef PseudorangeSwitchFactor This;

PseudorangeSwitchFactor() : measured_(0) {
        // h_=(Matrix(1,5)<<1,1,1,1,1).finished();
}

virtual ~PseudorangeSwitchFactor() {
}

// PseudorangeSwitchFactor(Key j, Key k, const double deltaObs, const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel& model) :
//         Base(model, j,k), measured_(deltaObs), satXYZ_(satXYZ) {
//         nomXYZ_=nomXYZ;
// }

PseudorangeSwitchFactor(Key key_pose3, Key key_point2, Key key_sw, const double deltaObs, const Point3 satXYZ, const int sv_prn, const Point3 nomXYZ, const SharedNoiseModel& model) :
        Base(model, key_pose3, key_point2, key_sw), measured_(deltaObs), satXYZ_(satXYZ) {
        nomXYZ_ = nomXYZ;
        sv_prn_ = sv_prn;
}
virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new PseudorangeSwitchFactor(*this)));
}

// /// vector of errors
// Vector evaluateError(const nonBiasStates& q,
//                      const SwitchVariableLinear& s,
//                      boost::optional<Matrix&> H1 = boost::none,
//                      boost::optional<Matrix&> H2 = boost::none) const;

/// vector of errors
Vector evaluateError(const Pose3& q, const Point2& point2_cb, const SwitchVariableLinear& sw,
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

}; // PseudorangeSwitchFactor Factor
} // namespace
