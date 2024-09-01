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

	ObsPassStaticT,    //静态绕障
	ObsPassOvertakeT,   //超车
	ObsPassMeetingT,   //会车
	ObsPoly,      //多项式曲线
};

