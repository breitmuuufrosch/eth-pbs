#pragma once

#include <vector>
using namespace std;

#include "Primitives.h"
#include "Vec2.h"

class Scene
{

public:
	// General statics
	static int xPoints;
	static int yPoints;
	static int zPoints;
	static double xSize;
	static double ySize;
	static double zSize;

	static double step;
	static double mass;
	static double stiffness;
	static double damping;
	double L;
	Vec2 p1,p2,p3;
	Vec2 v1,v2,v3;

	enum Method { INVALID_METHOD=0, EULER=1, LEAP_FROG=2, MIDPOINT=3, BACK_EULER=4, ANALYTIC=5 };
	static Method method;
	enum Testcase{ INVALID_TESTCASE=0, SPRING1D=1, FALLING=2, ERROR_MEASUREMENT=3, STABILITY_MEASUREMENT=4 };
	static Testcase testcase;

protected:
	// methods
	void timeStepReductionLoop(double stiffness,double mass,double damping,double L,double step, int numofIterations);
	void stabilityLoop(double stiffness,double mass,double damping,double L,double step,double endTime, int numofIterations);

	//Data members
	vector<MPoint> points;
	vector<MSpring> springs;

	//Data size
	int nSprings;
	int nPoints;

	//Animation
	bool pause;

	//Animation state
	double *x0, *x;
	double *v0, *v;


public:
	Scene(void);
	Scene(int argc, char* argv[]);
	~Scene(void);

	//Initialization
	void Init(void);
	void PrintSettings(void);
	void Render();
	void Update();
};
