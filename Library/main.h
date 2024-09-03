#pragma once
#include "scene_straight.h"
#include "scene_obs.h"
#include "polynomial.h"

enum PlanType
{
	StraightStopObsT,
	StraightStationT,
	StraightFollowT,
	StraightCrossWalkT,

	ObsPassStaticT,    //Static obstacle passing
	ObsPassOvertakeT,   //Surpassing car
	ObsPassMeetingT,   //Meeting
	ObsPoly,      //Polynomial planning
};

