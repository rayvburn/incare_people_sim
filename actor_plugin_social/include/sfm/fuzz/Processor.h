/*
 * Processor.h
 *
 *  Created on: Aug 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_PROCESSOR_H_
#define INCLUDE_SFM_FUZZ_PROCESSOR_H_

#include "fl/Headers.h"
#include <ignition/math/Angle.hh> // angle degree to radian conversion
#include <tuple>

namespace sfm {
namespace fuzz {

class Processor {

public:
	Processor();

	void init();
	void updateRegions();

	// TODO: check object dynamic
	/// \brief Currently investigated model's velocity vector
//	ignition::math::Vector3d vel_beta_;

	void setVelsRelativeAngle(const double &vels_relative_angle);					// φ_αβ (FIXME: phi there...)
	void setVelsRelativeAngle(const double &vel_alpha_angle, const double &vel_beta_angle);
	void setObjectDirRelativeAngle(const double &object_dir_relative_angle);		// β position direction relative to α direction

	virtual ~Processor();


private:

	uint8_t configureTermLocationDependent(std::string name, const char &side, const ignition::math::Angle &gamma_start,
			const ignition::math::Angle &gamma_end);

	void configureTermLocationIndependent(const std::string &name, const ignition::math::Angle &gamma_center);

	bool rearrangeTerms(const std::string &name, const uint8_t &status_curr, uint8_t &status_prev);

	/// \brief Determines location of the \beta element relative to \alpha direction of motion
	char decodeRelativeLocation(ignition::math::Angle eq, const ignition::math::Angle opp, ignition::math::Angle cc) const;

	/// \brief Calculates trapezoid vertices based on a region's center value.
//	std::tuple<double, double, double, double> calculateTrapezoid(const double &center) const;
	std::string calculateTrapezoid(const double &center) const;

	/// \brief Calculates trapezoid vertices based on region's start and end values.
//	std::tuple<double, double, double, double> calculateTrapezoid(const double &start, const double &end) const;
	std::tuple<std::string, bool> calculateTrapezoid(const double &start, const double &end, bool comp_breakdown = false) const;

	/// \brief Direction of the vector which connects \alpha 's position with \beta 's position.
	double d_alpha_beta_angle_;

	/// \brief Determines \alpha 's direction of motion.
	double alpha_dir_;

	/// \brief Acts as a status register, in the .cpp there are few masks defined
	/// so every case possible can be recognized using just one 1-byte variable.
	uint8_t term_extra_status_;

	/* ----- fuzzylite-related ----- */

	/// \brief Fuzzy logic engine
	fl::Engine engine_;

	/* ----- Input variables ----- */

	/// \brief An angle which helps specify on which hand-side the object is located relative to the actor
	fl::InputVariable location_; 	//	double object_dir_relative_angle_;

	/// \brief A relative angle between 2 objects' velocities vectors; used to determine
	/// whether objects are moving in the same direction
	fl::InputVariable direction_; 		//	double vels_relative_angle_;

	/* ----- Output variables ----- */
	fl::OutputVariable social_behavior_;

	/*  ----- Rule block ----- */
	fl::RuleBlock rule_block_;

//	/// \brief A distance between 2 objects; used to determine a level of condition
// //	double d_alpha_beta_len_;
//	fl::InputVariable d_alpha_beta_len_;


};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_PROCESSOR_H_ */
