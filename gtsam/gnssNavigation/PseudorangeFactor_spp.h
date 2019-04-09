/**
 *  @file   PseudorangeFactor_spp.h
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

namespace gtsam {

class GTSAM_EXPORT PseudorangeFactor_spp : public NoiseModelFactor2<Pose3, Point2> {

private:
typedef NoiseModelFactor2<Pose3, Point2> Base;
Point3 nomXYZ_;
Point3 satXYZ_;

Point2 cb;
Pose3 h_;
double measured_;

public:
    double sv_prn_;

typedef boost::shared_ptr<PseudorangeFactor_spp> shared_ptr;
typedef PseudorangeFactor_spp This;

PseudorangeFactor_spp() : measured_(0) {
        // h_=(Matrix(1,6)<<1,1,1,1,1,1).finished();
}

virtual ~PseudorangeFactor_spp() {
}

PseudorangeFactor_spp(Key key_pose3, Key key_point2, const double deltaObs, const Point3 satXYZ, const int sv_prn, const Point3 nomXYZ, const SharedNoiseModel& model) :
        Base(model, key_pose3, key_point2), measured_(deltaObs), satXYZ_(satXYZ) {
        nomXYZ_ = nomXYZ;
        sv_prn_ = sv_prn;
}


virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new PseudorangeFactor_spp(*this)));
}

/// vector of errors
Vector evaluateError(const Pose3& q, const Point2& point2_cb,
                     boost::optional<Matrix&> H1 = boost::none,
                     boost::optional<Matrix&> H2 = boost::none) const;

/**
   * @brief satellite set validation
   * @param prn
   * @return ture/false
   @ 
   */
  const bool PRNisGPS(const int prn)
  {
    if (prn <= 32 || prn == 84)
      return true;
    else{
      return false;
    } 
  }

  /**
   * @brief satellite set validation
   * @param prn
   * @return ture/false
   @ 
   */
 const  bool PRNisGLONASS(const int prn)
  {
    if (prn > 32 && prn <= 56)
      return true;
    else{
      return false;
    }
  }

  /**
   * @brief satellite set validation
   * @param prn
   * @return ture/false
   @ 
   */
  bool PRNisBeidou(int prn)
  {
    if ((prn <= 121) && (prn >= 87))
      return true;
    else{
      return false;
    }
  }



private:

/// Serialization function
friend class boost::serialization::access;
template<class ARCHIVE>
void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
                                         boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measured_);
}

}; // PseudorangeFactor_spp Factor
} // namespace
