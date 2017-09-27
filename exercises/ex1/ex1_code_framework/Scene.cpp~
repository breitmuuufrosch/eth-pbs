#include "Scene.h"
#include "Primitives.h"
#include "Vec2.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>

#include <iostream>
using namespace std;

double Scene::step=0.01f;
double Scene::mass=1.0;
double Scene::stiffness=10.0;
double Scene::damping=0.01f;

int Scene::xPoints=5;
int Scene::yPoints=5;
int Scene::zPoints=5;
double Scene::xSize=1.0;
double Scene::ySize=1.0;
double Scene::zSize=1.0;

extern void AdvanceTimeStep1(double k, double m, double d, double L, double dt, int method, double p1, double v1, double& p2, double& v2);
extern void AdvanceTimeStep3(double k, double m, double d, double L, double dt, Vec2& p1, Vec2& v1, Vec2& p2, Vec2& v2, Vec2& p3, Vec2& v3);

#define METHODS_NUM 6
#define TESTCASES_NUM 5
Scene::Method Scene::method=BACK_EULER;
char *methodNames[METHODS_NUM] = { "invalid", "euler", "symplectic_euler", "midpoint", "backwards_euler", "analytic"} ;
Scene::Testcase Scene::testcase=SPRING1D;
char *testcaseNames[TESTCASES_NUM] = { "invalid", "spring1d", "falling", "error_measurement", "stability_measurement"} ;

Scene::Scene(void)
{
   Init();
   PrintSettings();
}

//some default call: -testcase hanging -method Euler -stiff 10 -mass 0.1 -step 0.003 -damp 0.01

Scene::Scene(int argc, char* argv[])
{
	// defaults:
	testcase  = SPRING1D;
	method    = LEAP_FROG;
	stiffness = 10.0;
	mass      = 0.1f;
	step      = 0.003f;
	damping   = 0.01f;

   int arg=1;
   while(arg<argc)
   {
      //Testcase
      if(!strcmp(argv[arg], "-testcase"))
      {
		  testcase = INVALID_TESTCASE;
         arg++;

		 for (int i = 1; i < TESTCASES_NUM; i++)
			if(!strcmp(argv[arg], testcaseNames[i]))
			{
	            testcase=(Testcase)i;
			}
         if (testcase == INVALID_TESTCASE)
         {
            cerr << "Unrecognized testcase " << argv[arg] << endl;
			exit(1);
         }
         arg++;
      }
      //Integration method
      else if(!strcmp(argv[arg], "-method"))
      {
         arg++;
		  method = INVALID_METHOD;
		 //for(unsigned int i=0; i<strlen(argv[arg]); i++) argv[arg][i] = tolower(argv[arg][i]);
		 for (int i = 1; i < METHODS_NUM; i++ )
			 if (!strcmp(argv[arg],methodNames[i]))
				 method = (Method)i;
         if ( method == INVALID_METHOD && testcase != ERROR_MEASUREMENT && testcase != STABILITY_MEASUREMENT)
         {
            cerr << "Unrecognized method " << argv[arg] << endl;
			exit(1);
         }
         arg++;
      }
      //Object size
      else if(!strcmp(argv[arg], "-size"))
      {
         xPoints=atoi(argv[++arg]);
         yPoints=atoi(argv[++arg]);
         zPoints=atoi(argv[++arg]);
         arg++;
      }
      //Step size
      else if(!strcmp(argv[arg], "-step"))
      {
         step=(double)atof(argv[++arg]);
         arg++;
      }
      //Stiffness
      else if(!strcmp(argv[arg], "-stiff"))
      {
         stiffness=(double)atof(argv[++arg]);
         arg++;
      }
      //Damping
      else if(!strcmp(argv[arg], "-damp"))
      {
         damping=(double)atof(argv[++arg]);
         arg++;
      }
      //Mass
      else if(!strcmp(argv[arg], "-mass"))
      {
         mass=(double)atof(argv[++arg]);
         arg++;
      }
      //Others
      else
      {
         //Print instructions
         cerr << endl << "Unrecognized option " << argv[arg] << endl;
         cerr << "Usage: Test.exe -[option1] [settings] -[option2] [settings] ..." << endl;
         cerr << "Options:" << endl;
         cerr << "\t-testcase [";
		 for (int i = 0; i < TESTCASES_NUM-1; i++)
			 cerr << testcaseNames[i] << ",";
		 cerr << testcaseNames[TESTCASES_NUM-1] << "]" << endl; 
         cerr << "\t-method [";
		 for (int i = 0; i < METHODS_NUM-1; i++)
			 cerr << methodNames[i] << ",";
		 cerr << methodNames[METHODS_NUM-1] << "]" << endl; 
         cerr << "\t-step [step size in secs]" << endl;
         cerr << "\t-stiff [stiffness value]" << endl;
         cerr << "\t-damp [damping value]" << endl << endl;
		 exit(1);
         break;
      }
   }

   PrintSettings();
   Init();
}

Scene::~Scene(void)
{
}

void Scene::PrintSettings(void)
{
   cerr << endl << "Current Settings:" << endl;
   cerr << "\t-testcase " << testcaseNames[(int)testcase] << endl;
   cerr << "\t-method " << methodNames[(int)method] << endl;
   cerr << "\t-mass " << mass << endl;
   cerr << "\t-step " << step << endl;
   cerr << "\t-stiff " << stiffness << endl;
   cerr << "\t-damp " << damping << endl << endl;
}

void Scene::Init(void)
{
   //Animation settings
   pause=false;

   //Create points & springs
   nPoints=3; nSprings=3;
   if((testcase == SPRING1D) || (testcase == ERROR_MEASUREMENT) || testcase == STABILITY_MEASUREMENT) 
   {
	   nPoints=2; nSprings=1;
   } 
      
   //Allocate vector
   for (int i=0;i<nPoints;i++)
		points.push_back(MPoint());
		
   Vec2 c(0.0, 0.0);
   Vec2 zero(0.0, 0.0);
   p1 = c + Vec2(0, 1);
   p2 = c + Vec2(cos(210.0/180.0*M_PI), sin(210.0/180.0*M_PI));
   p3 = c + Vec2(cos(330.0/180.0*M_PI), sin(330.0/180.0*M_PI));
   v1 = v2 = v3 = zero;
   L = (p1-p2).length();
   if(testcase == SPRING1D || testcase == ERROR_MEASUREMENT || testcase == STABILITY_MEASUREMENT) {
       p1 = 0.0*c;
	   p2 = p1 + Vec2(0,-1);
       L = (p1-p2).length();
   }

   points[0].pos = p1;
   points[0].fixed=true;
   points[1].pos = p2;
   if(nPoints > 2)
      points[2].pos = p3;	 
   if(testcase == FALLING)
	   points[0].fixed = false;
   
   //Allocate vector
   for (int i=0;i<nSprings;i++)
		springs.push_back(MSpring());
		
   springs[0].set(&points[0], &points[1]);
   if(nPoints>2) {
      springs[1].set(&points[1],&points[2]);
      springs[2].set(&points[2],&points[0]);
   }
}

void Scene::timeStepReductionLoop(double stiffness,double mass,double damping,double L,double step, int numofIterations)
{
	double currstep = step;
	
	cout << "velocity change table:" << endl;	
	cout << "step ";
	for (int m = 1; m <= 5; m++ )
		cout << methodNames[m] << " ";
	cout << endl;
	double startT = 0.1;
	double startPos = -L,startV = 0;
	AdvanceTimeStep1(stiffness, mass, damping, L, startT, ANALYTIC, 0, 0, startPos, startV);
	for (int i = 0; i < numofIterations; i++)
	{
		printf("%.5lf ",currstep);
		for (int m = 1; m <= 5; m++ )
		{
			double p2y = startPos,v2y = startV;
			if ( m != ANALYTIC )
				AdvanceTimeStep1(stiffness, mass, damping, L, currstep, m, 0, 0, p2y, v2y);
			else
				AdvanceTimeStep1(stiffness, mass, damping, L, startT+currstep, m, 0, 0, p2y, v2y);
			printf("%.5e ",v2y-startV);
		}
		cout << endl;
		currstep/=2.0;
	}
	cout << "displacement table:" << endl;	
	cout << "step "; 
	for (int m = 1; m <= 5; m++ )
		cout << methodNames[m] << " ";
	currstep = step;
	cout << endl;
	for (int i = 0; i < numofIterations; i++)
	{
		printf("%.5lf ",currstep);
		for (int m = 1; m <= 5; m++ )
		{
			double p2y = startPos,v2y = startV;
			if ( m != ANALYTIC )
				AdvanceTimeStep1(stiffness, mass, damping, L, currstep, m, 0, 0, p2y, v2y);
			else
				AdvanceTimeStep1(stiffness, mass, damping, L, startT+currstep, m, 0, 0, p2y, v2y);
			printf("%.5e ",p2y-startPos);
		}
		cout << endl;
		currstep/=2.0;
	}
}

void Scene::stabilityLoop(double stiffness,double mass,double damping,double L,double step,double endTime, int numofIterations)
{
	double currstep = step;
	//damping = 0;
	cout << "Max amplitude table:" << endl;	
	cout << "step ";
	for (int m = 1; m <= 5; m++ )
		cout << methodNames[m] << " ";
	cout << endl;
	for (int i = 0; i < numofIterations; i++)
	{
		int numofSteps = (int)(endTime/step);
		cout << currstep << " ";
		for (int m = 1; m <= 5; m++ )
		{
			double p2y = -L,v2y = 0;
			double maxAmp = 0;
			for (int j = 0; j < numofSteps; j++)
			{
				AdvanceTimeStep1(stiffness, mass, damping, L, currstep, m, 0, 0, p2y, v2y);
				if ( abs(p2y-L) > maxAmp )
					maxAmp = abs(p2y-L);
			}
			cout << maxAmp << " ";
		}
		cout << endl;
		currstep*=2.0;
	}
}

void Scene::Update(void)
{
   if(pause)
   {
      return;
   }
	static double curr_time = 0;
	curr_time += step;
	//damping = 0;
	int numofIterations = 10;
	double endTime = 10;
   //Perform animation
   switch(testcase) {
	case SPRING1D:
		if ( method == ANALYTIC)
			AdvanceTimeStep1(stiffness, mass, damping, L, curr_time, method, p1.y, v1.y, p2.y, v2.y);
		else
			AdvanceTimeStep1(stiffness, mass, damping, L, step, method, p1.y, v1.y, p2.y, v2.y);
	    break;
	case FALLING:
	    AdvanceTimeStep3(stiffness, mass, damping, L, step, p1, v1, p2, v2, p3, v3);
	    break;
	case ERROR_MEASUREMENT:
		timeStepReductionLoop(stiffness, mass, damping, L, step, numofIterations);
		exit(0);
		break;
	case STABILITY_MEASUREMENT:
		stabilityLoop(stiffness, mass, damping, L, step, endTime, numofIterations);
		exit(0);
		break;
   }
   points[0].pos = p1;
   points[1].pos = p2;
   if(nPoints>2) {
	   points[2].pos = p3; 
   }
}

void Scene::Render(void)
{
   for(int i=0; i<nSprings;i++)
	  springs[i].render();
   
   for(int i=0; i<nPoints;i++)
      points[i].render();
}
