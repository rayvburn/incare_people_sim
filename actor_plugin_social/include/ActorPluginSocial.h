/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>

#include "core/Actor.h"
#include "core/Enums.h"
#include "core/CommonInfo.h"
#include "core/FSM.h"

#include "inflation/Box.h"
#include "inflation/Circle.h"
#include "inflation/Ellipse.h"

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include "sfm/core/SocialForceModel.h"
#include "sfm/core/SFMDebug.h"


#define ACTOR_SHARED_PTR

#ifdef ACTOR_SHARED_PTR
/* according to https://stackoverflow.com/questions/10534220/c11-passing-this-as-paramenter-for-stdmake-shared
 * shared from this should be called on an object that is already managed by shared_ptr
 * shared from this is used to pass a shared_ptr from an Actor class to a Connection class */
#include <memory>
#endif

// -------------------------

//#define VISUALIZE_SFM
//#define CREATE_ROS_INTERFACE

// -------------------------

#ifdef VISUALIZE_SFM
	//#define VIS_SFM_GRID
	#define VIS_SFM_POINT
#endif

// -------------------------

#ifdef CREATE_ROS_INTERFACE
#include "ActorROSInterface.h"
#endif

// -------------------------

#ifdef VISUALIZE_SFM

	#ifdef VIS_SFM_POINT
	#include <SFMVisPoint.h>
	#elif defined(VIS_SFM_GRID)
	#include <SFMVisGrid.h>
	#endif

#endif

#define REFACTOR_COMMON

namespace gazebo
{

// -------------------------

  class GAZEBO_VISIBLE ActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin();

    /// \brief Actor class object
#ifndef ACTOR_SHARED_PTR
    public: actor::core::Actor actor_object;
#else
    public: std::shared_ptr<actor::core::Actor> actor_ptr_;
#endif

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Unherited.
    public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);



    /// \brief Time of the last new target selection
	private: common::Time last_target_selection_time;

    /// \brief Time of the last reachability test.
	private: common::Time last_reachability_time;


    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    /// \brief TODO
    private: bool ReadSDF();

    private: bool controller_enabled_;


#ifdef VISUALIZE_SFM

    private: void VisualizeForceField();

#ifdef VIS_SFM_POINT
    private: static SocialForceModel::SFMVisPoint sfm_vis;
    private: SocialForceModel::SFMVisPoint sf_vis;
#elif defined(VIS_SFM_GRID)
    private: static SocialForceModel::SFMVisGrid sfm_vis;
#endif

    private: static SocialForceModel::SFMVisGrid grid_vis;

#endif


#ifdef CREATE_ROS_INTERFACE
    ActorUtils::ActorROSInterface ros_interface;
#endif

  };
}
#endif
