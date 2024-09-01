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

	ObsPassStaticT,    //��̬����
	ObsPassOvertakeT,   //����
	ObsPassMeetingT,   //�ᳵ
	ObsPoly,      //����ʽ����
};

