//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

// 2d fluid example, glut viewer

#define NOMINMAX
#include <iostream>
#include <string>
#include <algorithm>
#include "fluid2d.h"

// toggle velocity display
// 0 = only show density
// 1 = show density, + velocity
// 2 = show density, + pressure
// 3 = show density, + divergence
int showFields = 0;

// simulation variables
Fluid2D* fluid = nullptr;
bool pauseFlag = true;

// drawing variables
int width = 800;
int height = 600;
double camera[] = { 0.0f, 1.0f, 0.0f, 1.0f };

// interaction
const double kernel[9] = { 0.5, 0.5, 0.5, 0.5, 1 , 0.5, 0.5, 0.5, 0.5 };
const double sf = 4;
int oldx = -1, oldy = -1;

void reshapeGlut(int w, int h)
{
	if (h == 0) h = 1;

	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(camera[0], camera[1], camera[2], camera[3], -10.0f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

static void drawFields(const int xRes, const int yRes, const Array2d &red, const Array2d &green, const Array2d &blue)
{
	glPushMatrix();
	glScalef(1.0f / (double)(xRes - 1), 1.0f / (double)(yRes - 1), 1.0f);

	double redMax = 0.0f;
	double greenMax = 0.0f;
	double blueMax = 0.0f;
	double redMin = fabs(red(1, 1));
	double greenMin = fabs(green(1, 1));
	double blueMin = fabs(blue(1, 1));
	for (int i = 0; i < xRes; i++)
	{
		for (int j = 0; j < yRes; j++)
		{
			redMax = std::max(redMax, fabs(red(i, j)));
			greenMax = std::max(greenMax, fabs(green(i, j)));
			blueMax = std::max(blueMax, fabs(blue(i, j)));

			redMin = std::min(redMin, fabs(red(i, j)));
			greenMin = std::min(greenMin, fabs(green(i, j)));
			blueMin = std::min(blueMin, fabs(blue(i, j)));
		}
	}
	redMax = 1.0f / (redMax - redMin);
	greenMax = 1.0f / (greenMax - greenMin);
	blueMax = 1.0f / (blueMax - blueMin);

	for (int y = 0; y < yRes - 1; y++)
	{
		for (int x = 0; x < xRes - 1; x++)
		{
			glBegin(GL_TRIANGLE_FAN);
			double SWred = (fabs(red(x, y)) - redMin) * redMax;
			double SEred = (fabs(red(x + 1, y)) - redMin) * redMax;
			double NWred = (fabs(red(x, y + 1)) - redMin) * redMax;
			double NEred = (fabs(red(x + 1, y + 1)) - redMin) * redMax;
			double averagered = (SWred + SEred + NWred + NEred) * 0.25f;

			double SWgreen = (fabs(green(x, y)) - greenMin) * greenMax;
			double SEgreen = (fabs(green(x + 1, y)) - greenMin) * greenMax;
			double NWgreen = (fabs(green(x, y + 1)) - greenMin) * greenMax;
			double NEgreen = (fabs(green(x + 1, y + 1)) - greenMin) * greenMax;
			double averagegreen = (SWgreen + SEgreen + NWgreen + NEgreen) * 0.25f;

			double SWblue = (fabs(blue(x, y)) - blueMin) * blueMax;
			double SEblue = (fabs(blue(x + 1, y)) - blueMin) * blueMax;
			double NWblue = (fabs(blue(x, y + 1)) - blueMin) * blueMax;
			double NEblue = (fabs(blue(x + 1, y + 1)) - blueMin) * blueMax;
			double averageblue = (SWblue + SEblue + NWblue + NEblue) * 0.25f;

			glColor4f(averagered, averagegreen, averageblue, 1.0f);
			glVertex3f(x + 0.5f, y + 0.5f, 0.0f);
			glColor4f(SWred, SWgreen, SWblue, 1.0f);
			glVertex3f(x, y, 0.0f);
			glColor4f(NWred, NWgreen, NWblue, 1.0f);
			glVertex3f(x, y + 1, 0.0f);
			glColor4f(NEred, NEgreen, NEblue, 1.0f);
			glVertex3f(x + 1, y + 1, 0.0f);
			glColor4f(SEred, SEgreen, SEblue, 1.0f);
			glVertex3f(x + 1, y, 0.0f);
			glColor4f(SWred, SWgreen, SWblue, 1.0f);
			glVertex3f(x, y, 0.0f);
			glEnd();
		}
	}
	glPopMatrix();
}

// obtain idx into field from mouse position
bool unproject(int x, int y, int &rx, int &ry)
{
	double modelview[16], projection[16], ox, oy, oz;
	int viewport[4];
	float z;
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetIntegerv(GL_VIEWPORT, viewport);
	glReadPixels(x, viewport[3] - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
	gluUnProject(x, viewport[3] - y, z, modelview, projection, viewport, &ox, &oy, &oz);

	rx = (int)(ox*(fluid->xRes() - 1) + 0.5);
	ry = (int)(oy*(fluid->yRes() - 1) + 0.5);
	return !(rx < 0 || rx >= fluid->xRes() || ry < 0 || ry >= fluid->yRes());
}

void displayCallback()
{
	// setup for draw
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(camera[0], camera[1], camera[2], camera[3], -10.0f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	switch (showFields)
	{
	case 0:
		drawFields(fluid->xRes(), fluid->yRes(), fluid->density(), fluid->density(), fluid->density());
		break;
	case 1:
		drawFields(fluid->xRes(), fluid->yRes(), fluid->density(), fluid->xVelocity(), fluid->yVelocity());
		break;
	case 2:
		drawFields(fluid->xRes(), fluid->yRes(), fluid->density(), fluid->divergence(), fluid->divergence());
		break;
	case 3:
		drawFields(fluid->xRes(), fluid->yRes(), fluid->density(), fluid->divergence(), fluid->divergence());
		break;
	}

	glutSwapBuffers();
}

void updateTitle()
{
	std::string title = "Exercise 4 - show density";
	switch (showFields)
	{
	case 0:
		break;
	case 1:
		title += "(r) + velocity(g,b)";
		break;
	case 2:
		title += "(r) + pressure(b)";
		break;
	case 3:
		title += "(r) + divergence(b)";
		break;
	}
	if (fluid->_hasWind)
		title += " - wind on";
	else
		title += " - wind off";
	if (!pauseFlag)
		title += " - PAUSED, press space";
	glutSetWindowTitle(title.c_str());
}

void keybCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	// quit
	case 'q':
		exit(0);
		break;

	case 'd':
		showFields = (showFields + 1) % 4;
		updateTitle();
		break;

	// wind
	case 'w':
		fluid->_hasWind = !fluid->_hasWind;
		updateTitle();
		break;

	case ' ':
		pauseFlag = !pauseFlag;
		updateTitle();
		break;
	}

	glutPostRedisplay();
}

void mouseCallback(int button, int state, int x, int y)
{
	unproject(x, y, oldx, oldy);
}

void motionCallback(int x, int y)
{
	int ox, oy;
	if (unproject(x, y, ox, oy) && oldx >= 0)
	{
		for (int i = -1, ik = 0; i < 1; i++)
		{
			for (int j = -1; j < 1; j++, ik++)
			{
				if (ox + i < 0 || ox + i >= fluid->xRes() || oy + j < 0 || oy + j >= fluid->yRes())
					continue;
				int idx = ox + i + fluid->xRes()*(oy + j);
				fluid->xForce()(ox + i, oy + j) += sf * kernel[ik] * (ox - oldx);
				fluid->yForce()(ox + i, oy + j) += sf * kernel[ik] * (oy - oldy);
			}
		}
	}

	oldx = ox;
	oldy = oy;
}

void idleFunc()
{
	if (pauseFlag)
	{
		// bottom inlet
		fluid->addDensity(0.45, 0.55, 0.1, 0.15);

		// centered for debugging
		//fluid->addDensity(0.45, 0.55, 0.45, 0.55);

		fluid->step();
	}

	glutPostRedisplay();
}

// setup glut 
int main(int argc, char **argv)
{
	// print config
	std::cout << "2D Fluid test, res:" << simulationResolution << std::endl << std::endl;
	std::cout << "Usage: start/stop='space', quit='q', toggle display='d'" << std::endl << std::endl;
	fluid = new Fluid2D(simulationResolution, simulationResolution);

	// read command line parameters
	int arg = 1;
	while (arg < argc)
	{
		// PNG output
		if (!strcmp(argv[arg], "-png"))
		{
			fluid->_pngOutput = true;
			arg++;

			if (arg < argc)
			{
				fluid->_pngOutputInterval = atoi(argv[arg]);
				arg++;
			}
		}
		else if (!strcmp(argv[arg], "-wind"))
		{
			fluid->_hasWind = true;
			arg++;
		}
	}

	if (fluid->_pngOutput)
	{
#ifdef _WIN32
		// create output directory
		CreateDirectory(OUTPATH, NULL);
		// delete existing pngs
		std::string path = std::string(".\\" OUTPATH);
		std::string remove = std::string("del /s ") + path + std::string("\\*.png");
		system(remove.c_str());
#else
		// create output directory
		system("mkdir " OUTPATH);
		// delete existing pngs
		std::string path = std::string("./" OUTPATH);
		std::string remove = std::string("rm -f ") + path + std::string("/*.png");
		system(remove.c_str());
#endif
	}

	// intialize GLUT window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(50, 50);
	glutInitWindowSize(width, height);
	glutCreateWindow("fluid2d");

	// init GLUT callbacks
	glutDisplayFunc(displayCallback);
	glutKeyboardFunc(keybCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	glutIdleFunc(idleFunc);
	reshapeGlut(width, height);

	// init GLUT drawing params
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	updateTitle();

	glutMainLoop();

	// clean up
	delete fluid;
	return 0;
}
