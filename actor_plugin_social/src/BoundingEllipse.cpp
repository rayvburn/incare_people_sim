/*
 * BoundingEllipse.cpp
 *
 *  Created on: Mar 31, 2019
 *      Author: rayvburn
 */

#include "BoundingEllipse.h"
#include <ignition/math/Angle.hh>
#include <ignition/math/Quaternion.hh>
#include <math.h> // fabs(), pow(), atan2(), tan()

namespace ActorUtils {

// ------------------------------------------------------------------- //

BoundingEllipse::BoundingEllipse(): a_major_(1.2), b_minor_(0.5), yaw_ellipse_(0.0), yaw_offset_(0.0) { }

// ------------------------------------------------------------------- //

BoundingEllipse::BoundingEllipse(const double &a_major, const double &b_minor, const double &yaw,
								 const ignition::math::Vector3d &center_point,
								 const ignition::math::Vector3d &offset_vector) {

	init(a_major, b_minor, yaw, center_point, offset_vector);
}

// ------------------------------------------------------------------- //

void BoundingEllipse::init(const double &a_major, const double &b_minor, const double &yaw,
		  const ignition::math::Vector3d &center_point,
		  const ignition::math::Vector3d &offset_vector) {

	setSemiMajorAxis(a_major);
	setSemiMinorAxis(b_minor);
	setCenterOffset(offset_vector);
	updatePose(ignition::math::Pose3d(center_point, ignition::math::Quaterniond(0.0, 0.0, yaw)));

}

// ------------------------------------------------------------------- //

void BoundingEllipse::setSemiMajorAxis(const double &a_major) {
	a_major_ = std::fabs(a_major);
}

// ------------------------------------------------------------------- //

void BoundingEllipse::setSemiMinorAxis(const double &b_minor) {
	b_minor_ = std::fabs(b_minor);
}

// ------------------------------------------------------------------- //

void BoundingEllipse::setYaw(const double &yaw_ellipse) {
	yaw_ellipse_ = yaw_ellipse;
	updateShiftedCenter();
}

// ------------------------------------------------------------------- //

void BoundingEllipse::setCenter(const ignition::math::Vector3d &center_point) {
	center_ = center_point;
	updateShiftedCenter();
}

// ------------------------------------------------------------------- //

// this must be set after setCenter and both setAxis
void BoundingEllipse::setCenterOffset(const ignition::math::Vector3d &offset_vector) {

	/* check if offset is lying down within the ellipse's bound -
	 * at this moment the offset is 0 and location of offset point
	 * must be investigated */

	// FIXME: shouldn't offset_ be set before isWithin?
	if ( offset_vector.Length() > 1e-06 && isWithin(offset_vector) ) {

		offset_ = offset_vector;

		// calculate yaw_offset based on offset vector
		ignition::math::Angle yaw_offset_angle;
		yaw_offset_angle.Radian( std::atan2(offset_vector.Y(), offset_vector.X()) );
		yaw_offset_angle.Normalize();
		yaw_offset_ = yaw_offset_angle.Radian();

	} else {

		// TODO: some error message

	}

}

// ------------------------------------------------------------------- //

/// ellipse orientation must match world's coordinate system -
/// a conversion from actor's coordinate system must be done!
void BoundingEllipse::updatePose(const ignition::math::Pose3d &pose) {
	center_ = pose.Pos();
	yaw_ellipse_ = pose.Rot().Yaw() - IGN_PI_2; // coordinate systems transformation
	updateShiftedCenter();
}

// ------------------------------------------------------------------- //

/// find point in which line going through ellipse's shifted center and pt_dest intersects the ellipse
/// determine if pt_dest lies within ellipse's bounds

std::tuple<bool, ignition::math::Vector3d> BoundingEllipse::getIntersection(const ignition::math::Vector3d &pt_dest) const {

//	/*
//	 * https://en.wikipedia.org/wiki/Ellipse#Shifted_Ellipse
//	 *
//	 * Using the sine and cosine functions a parametric representation
//	 * of the ellipse with shifted center is as follows:
//	 *
//	 * (c1 + a*cos(theta); c2 + b*sin(theta),
//	 * where 0 <= theta < 2pi
//	 *
//	 */
//
//	// V1
//	// 1st calculate the angle from center to the dest point
//	ignition::math::Angle theta;
//	ignition::math::Vector3d to_dest;
//	to_dest = pt_dest - (center_ + offset_);
//	theta.Radian(std::atan2(to_dest.Y(), to_dest.X()));
//	theta.Normalize();
//
//	// 2nd take the ellipse orientation into consideration
//	theta.Radian( theta.Radian() + yaw_ );
//	theta.Normalize();
//
//	// 3rd find the point of intersection of the to_dest vector and the ellipse
//	ignition::math::Vector3d pt_intersection;
//	pt_intersection.X( center_.X() + ( offset_.X() + (a_major_ * cos(theta.Radian())) ) );
//	pt_intersection.Y( center_.Y() + ( offset_.Y() + (b_minor_ * sin(theta.Radian())) ) );
//	pt_intersection.Z(0.0f);
//
//	return (pt_intersection);

	// lack of angle to check intersection from center (not shifted)
//	// V2
//	ignition::math::Vector3d pt_intersection;
//	double angle_center_dest;
//	std::tie(pt_intersection, angle_center_dest) = getIntersectionExtended(pt_dest);
//	return (pt_intersection);


	// V3
	// TODO: method similar to the ignition::math::Box's Intersects - returns a tuple:
	// Vector3d pt_of_intersection, bool is_within  ||  dist from center to intersection point

	/* calculate the angle from the ellipse's interior (its center if offset is 0)
	 * to the destination point */
	ignition::math::Angle psi;
	ignition::math::Vector3d to_dest = pt_dest - center_shifted_;
	psi.Radian(std::atan2(to_dest.Y(), to_dest.X()));
	psi.Normalize();

	/* find line's points of intersection with ellipse;
	 * line is created based on psi value (from the shifted
	 * center to the destination point */

	// returns a 3-element tuple
	unsigned int solution_num = 0;
	ignition::math::Vector3d pt_of_intersection1, pt_of_intersection2;
	std::tie(solution_num, pt_of_intersection1, pt_of_intersection2) = getIntersectionWithLine(psi.Radian());

	/* check which intersection point is 'correct' based on psi */
	ignition::math::Vector3d *pt_proper;
	ignition::math::Angle angle_test( std::atan2( (pt_of_intersection1 - center_shifted_).Y(),
												  (pt_of_intersection1 - center_shifted_).X()) );
	angle_test.Normalize();

	/* correct angle will be nearly equal to 0, wrong point's vector
	 * will produce doubled angle */

	// TODO: some error handling? 1 solution?
	if ( (angle_test - psi) < 1e-03 ) {
		pt_proper = &pt_of_intersection1;
		std::cout << "BoundingEllipse::getIntersection - ANGLE TEST PASSED" << std::endl;
	} else {

		// debug
		angle_test.Radian( std::atan2( (pt_of_intersection2 - center_shifted_).Y(),
									   (pt_of_intersection2 - center_shifted_).X() ) );
		if ( (angle_test - psi) < 1e-03 ) {
			std::cout << "BoundingEllipse::getIntersection - ANGLE TEST PASSED" << std::endl;
		}
		//

		pt_proper = &pt_of_intersection2;
	}

	/* check if destination point lies within the ellipse */
	bool is_within = false;
	if ( (pt_dest - center_shifted_).Length() - (*pt_proper - center_shifted_).Length() <= 0.0 ) {
		std::cout << "BoundingEllipse::getIntersection - POINT IS WITHIN" << std::endl;
		is_within = true;
	} else {
		std::cout << "BoundingEllipse::getIntersection - POINT IS SAFE" << std::endl;
	}

	return (std::make_tuple(is_within, *pt_proper));

}

// ------------------------------------------------------------------- //

bool BoundingEllipse::isWithin(const ignition::math::Vector3d &pt) const {

//	ignition::math::Vector3d pt_to_check;
//	double angle_center_dest;
//	std::tie(pt_to_check, angle_center_dest) = getIntersectionExtended(pt);
//
//	// pt_to_check length of (from center)
//	pt_to_check.Z(0.0);
//
//	ignition::math::Vector3d pt_on_ellipse;
//	pt_on_ellipse.X( center_.X() + ( offset_.X() + (a_major_ * cos(angle_center_dest)) ) );
//	pt_on_ellipse.Y( center_.Y() + ( offset_.Y() + (b_minor_ * sin(angle_center_dest)) ) );
//	pt_on_ellipse.Z(0.0f);
//
//	// compare length from the `center` of an ellipse to both points
//	if ( (pt_to_check - (center_ + offset_)).Length() <= (pt_on_ellipse - (center_ + offset_)).Length()  ) {
//		return (true);
//	}
//
//	return (false);

	bool is_within;
	std::tie(is_within, std::ignore) = getIntersection(pt);
	return (is_within);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d BoundingEllipse::getCenter() const {
	return (center_);
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d BoundingEllipse::getCenterOffset() const {
	return (offset_);
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker BoundingEllipse::getMarkerConversion() const {

	visualization_msgs::Marker marker;

	marker.header.frame_id = "map";
	//marker.header.stamp = 0.0;
	marker.ns = "test"; // TODO: change ns name
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::MODIFY;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = center_shifted_.X();
	marker.pose.position.y = center_shifted_.Y();
	marker.pose.position.z = 0.9; // half of a typical person height

	// convert rpy to quaternion and set marker's orientation
	ignition::math::Quaterniond rot(0.0, 0.0, yaw_ellipse_);
	marker.pose.orientation.x = rot.X();
	marker.pose.orientation.y = rot.Y();
	marker.pose.orientation.z = rot.Z();
	marker.pose.orientation.w = rot.W();

	// scale
	marker.scale.x = a_major_;
	marker.scale.y = b_minor_;
	marker.scale.z = 1.8f; 	// typical person height

	marker.color.a = 0.3; 	// alpha channel
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	return (marker);

}

// ------------------------------------------------------------------- //

void BoundingEllipse::updateShiftedCenter() {

	center_shifted_.X(center_.X() + offset_.X() * cos(yaw_offset_ + yaw_ellipse_));
	center_shifted_.Y(center_.Y() + offset_.Y() * sin(yaw_offset_ + yaw_ellipse_));
	//center_shifted_.Z(0.0);

}

// ------------------------------------------------------------------- //

// useful for simple intersection from center to destinatino point search
std::tuple<ignition::math::Vector3d, double> BoundingEllipse::getIntersectionExtended(const ignition::math::Vector3d &pt_dest)
	const {

	/*
	 * https://en.wikipedia.org/wiki/Ellipse#Shifted_Ellipse
	 *
	 * Using the sine and cosine functions a parametric representation
	 * of the ellipse with shifted center is as follows:
	 *
	 * (c1 + a*cos(theta); c2 + b*sin(theta),
	 * where 0 <= theta < 2pi
	 *
	 */

	/* 1st calculate the angle from the ellipse's interior (its center if offset is 0)
	 * to the destination point */
	ignition::math::Angle psi;
	ignition::math::Vector3d to_dest = pt_dest - center_shifted_;
	psi.Radian(std::atan2(to_dest.Y(), to_dest.X()));
	psi.Normalize();

	// 2nd take the ellipse orientation into consideration
	psi.Radian( psi.Radian() + yaw_ellipse_ );
	psi.Normalize();

	// 3rd find the point of intersection of the to_dest vector and the ellipse
	ignition::math::Vector3d pt_intersection;
	pt_intersection.X( center_.X() + ( offset_.X() + (a_major_ * cos(psi.Radian())) ) );
	pt_intersection.Y( center_.Y() + ( offset_.Y() + (b_minor_ * sin(psi.Radian())) ) );
	pt_intersection.Z(0.0f);

	return (std::make_tuple( pt_intersection, psi.Radian() ));

}

// ------------------------------------------------------------------- //

// useful for intersection from center to destination point search
/// return: number of solutions; 1st intersection point; 2nd intersection point
/// when 0 solutions found returns 0,0,0 vectors

std::tuple<unsigned int, ignition::math::Vector3d, ignition::math::Vector3d> BoundingEllipse::getIntersectionWithLine(const double &to_dest_angle) const {

	std::cout << "getIntersectionWithLine()";

	// number of solutions indicator
	unsigned int solution_num = 0;

	// slope of a line
	double a_l = std::tan(to_dest_angle);

	// intercept
	double b_l = offset_.Y() + center_shifted_.X() * a_l;

	/* based on ellipse parametric equation, using quadratic equation,
	 * find intersection points of the line and the ellipse */
	double delta = std::pow(a_major_, 4.0)*std::pow(a_l, 2.0) - 4.0 * std::pow(b_minor_, 2.0) * std::pow(a_major_, 2.0) * b_l;

	// check number of solutions
	if ( std::fabs(delta) < 1e-06 ) {
		std::cout << "\t1 solution\n";
		solution_num = 1;
	} else if ( delta > 0.0 ) {
		std::cout << "\t2 solutions\n";
		solution_num = 2;
	} else {
		std::cout << "\t0 solutions\n";
		solution_num = 0;
	}

	/* always 2 solutions are expected so in advance calculate numerator's
	 * first element and denominator for x-coordinate computation */
	double num_elem = - std::pow(a_major_, 2.0) * a_l;
	double denom = 2.0 * std::pow(b_minor_, 2.0);

	// calculate intersection point(s) based on solution_num
	if ( solution_num == 2 ) {

		ignition::math::Vector3d pt_of_intersection1, pt_of_intersection2;
		pt_of_intersection1.X( num_elem - std::sqrt(delta) );
		pt_of_intersection1.Y( a_l * pt_of_intersection1.X() + b_l );

		pt_of_intersection2.X( num_elem + std::sqrt(delta) );
		pt_of_intersection2.Y( a_l * pt_of_intersection2.X() + b_l );

		return ( std::make_tuple( 2, pt_of_intersection1, pt_of_intersection2 ) );

	} else if ( solution_num == 1 ) {

		ignition::math::Vector3d pt_of_intersection;
		pt_of_intersection.X( num_elem / denom );
		pt_of_intersection.Y( a_l * pt_of_intersection.X() + b_l );
		return ( std::make_tuple( 1, pt_of_intersection, ignition::math::Vector3d() ) );

	} else {

		return ( std::make_tuple( 0, ignition::math::Vector3d(), ignition::math::Vector3d() ) );

	}

}

// ------------------------------------------------------------------- //

BoundingEllipse::~BoundingEllipse() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ActorUtils */
