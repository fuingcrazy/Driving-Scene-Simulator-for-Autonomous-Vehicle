# Driving Scene Simulator for Autonomous Vehicles

This project was designed to simulate different scenes that autonomous vehicles may encounter, embedded with some basic speed control and path planning algorithms. 

## Catalog
- [Prerequisite](#Prerequisite)
- [Architecture](#Architecture)
- [Run](#Run)
- [Demo](#Demo)

### Prerequisite
Make sure you have downloaded Visual Studio: https://visualstudio.microsoft.com/ and installed C++ extensions 
This project needs EasyX Libaray as an API for GUI design, to download and setup EasyX: https://easyx.cn/
It's very convenient to include easyx into your c++ file, just add this line:

```cpp
#include <graphics.h>
```
### Architecture
```
filetree 
├── Gifs
├── README.md
├── x64
├── Library
|  ├── car.h
|  ├── main.h
|  ├── planning_base.h
|  ├── Polynomial.h
|  ├── road.h
|  ├── scene_base.h
|  ├── scene_obs.h
|  ├── scene_straight.h
|  ├── traffic.h
├── Src
|  ├── car.cpp
|  ├── main.cpp
|  ├── planning_base.cpp
|  ├── Polynomial.cpp
|  ├── road.cpp
|  ├── scene_base.cpp
|  ├── scene_obs.cpp
|  ├── scene_straight.cpp
|  ├── traffic.cpp
└── car_project.sln
```

### Run
To run the code, first use Visual Studio to open car_project.sln, and you should this part in main.cpp:
```cpp
if (process(PlanType::StraightCrossWalkT))
	cout << "Scene Finish! " << endl;
```
We have an enum called "PlanType" in main.h, where you can find different available scenes to simulate:
```cpp
enum PlanType
{
	StraightStopObsT,
	StraightStationT,
	StraightFollowT,
	StraightCrossWalkT,

	ObsPassOvertakeT,   //Surpassing car
	ObsPassMeetingT,   //Meeting
	ObsPoly,      //Polynomial planning
};
```
So select the type you want to try and press Run button! :) Feel free to change the values of variables
### Demo
Here are some demonstrations of this project:
#### Stop at a station
Car slows down and stops at a designated station(point)

![image](https://github.com/fuingcrazy/Driving-Scene-Simulator-for-Autonomous-Vehicle/blob/master/Gifs/StationStop.gif)

#### Stop in front of an Obstacle
Car stops to avoid collision 

![image](https://github.com/fuingcrazy/Driving-Scene-Simulator-for-Autonomous-Vehicle/blob/master/Gifs/StopObs.gif)
#### Follow a slower car

![image](https://github.com/fuingcrazy/Driving-Scene-Simulator-for-Autonomous-Vehicle/blob/master/Gifs/Follow.gif)
#### Go through a crosswalk
Adjust speed to avoid pedestrians on a crosswalk

![image](https://github.com/fuingcrazy/Driving-Scene-Simulator-for-Autonomous-Vehicle/blob/master/Gifs/CrossWalk.gif)
#### Overtake a vehicle

![image](https://github.com/fuingcrazy/Driving-Scene-Simulator-for-Autonomous-Vehicle/blob/master/Gifs/Overtake.gif)
#### Avoid collision when meeting a car driving towards us

![image](https://github.com/fuingcrazy/Driving-Scene-Simulator-for-Autonomous-Vehicle/blob/master/Gifs/MeetingObs.gif)
#### Change lane
Use 5th polynomial to generate an appropriate path for lane changing

![image](https://github.com/fuingcrazy/Driving-Scene-Simulator-for-Autonomous-Vehicle/blob/master/Gifs/Polynomial.gif)
