/*
 * Action.h
 *
 *  Created on: Dec 31, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_ACTION_H_
#define INCLUDE_ACTOR_CORE_ACTION_H_

#include <string>

namespace actor {
namespace core {

class Action {

public:

	/// \brief Enum constants related to one or multiple tasks
	typedef enum {

		UNKNOWN = 9999u,	 //!< UNKNOWN: initial state
		IN_PROGRESS = 9998u, //!< IN_PROGRESS: a universal state for actions that consist of an atomic task (that are `non-dividable`)
		FINISHED = 9997u,    //!< FINISHED: used by all tasks

		FAILED = 0u,         //!< FAILED: used by all tasks
		PREPARING,           //!< PREPARING: associated with alignToTargetDirection `state`; may be used by many tasks
		APPROACHING,         //!< APPROACHING: associated with the movement to the goal direction (\ref actor::core::Actor::manageTargetSingleReachment); may be used by many tasks

		FOLLOWING,           //!< FOLLOWING: related to the `object tracking` state when the actor is actually following an object (i.e. the object is moving)
		OBJECT_NON_REACHABLE,//!< OBJECT_NON_REACHABLE: used in the `object tracking` state when the followed object is located in the place considered as a lethal obstacle on the actor costmap
//		OBJECT_REACHED,      //!< OBJECT_REACHED: used in the `object tracking` state when the followed object is close enough to the actor and the object is static

		LYING,               //!< LYING: used in the `lie_down` state, associated with the internal `idle` state
		STANDING_UP,         //!< STANDING_UP: used in the `lie_down` state, enabled after state termination request

	} ActionStatus;

	/// \brief FollowObject PTF statuses. A proper status number will be set as a status ID
	/// by the FollowObject ROS Action Server
	typedef enum {
//		UNKNOWN = 0,          //!< UNKNOWN
		NOT_FOLLOWING,        //!< NOT_FOLLOWING
		UNABLE_TO_FIND_PLAN,  //!< UNABLE_TO_FIND_PLAN
		TRACKING,             //!< TRACKING: the actor is actually following an object (i.e. the object is moving)
		OBJECT_REACHED,       //!< TARGET_REACHED: the followed object is close enough to the actor and the object is static
		ROTATE_TOWARDS_OBJECT,//!< ROTATE_TOWARDS_OBJECT
		NOT_REACHABLE,        //!< NOT_REACHABLE: the followed object is located in the place considered as a lethal obstacle on the actor costmap
		WAIT_FOR_MOVEMENT,    //!< WAIT_FOR_MOVEMENT
	} FollowObjectStatus;

	/// \brief Default constructor
	Action(int status_terminal = FINISHED);

	// TODO
	void start(int status_initial = IN_PROGRESS);

	/// \brief Updates the status
	void setStatus(ActionStatus status, const std::string &description = "");
	void setStatus(int status, const std::string &description = "");

	// TODO:
	void forceTermination();

	/// \brief Returns the current status
	int getStatus() const;

	/// \brief Returns the status description
	std::string getStatusDescription() const;

	// TODO:
	bool isTerminated() const;

	/// \brief Destructor
	virtual ~Action();

private:

	/// \brief Stores the current status of the action
	ActionStatus status_;

	/// TODO:
	int status_int_;
	/// TODO:
	int status_terminal_;

	bool termination_brutal_;

	/// \brief Stores a potentially useful in debugging status description
	std::string text_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_ACTION_H_ */
