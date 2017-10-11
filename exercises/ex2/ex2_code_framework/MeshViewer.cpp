//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include <stdlib.h>
#include "SimpleFEMDefs.h"
#include "MeshViewer.h"
#include "FEMMesh.h"
#include <algorithm>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#define GLUT_DISABLE_ATEXIT_HACK
#include "GL/glut.h"
#endif

MeshViewer::MeshViewer(int argc, char *argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
}

// Handling regular keyboard input
void handle_keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'q':
	case 27: // ESC
		exit(0);
		break;
	case 'v':
		MeshViewer::m_show3D = !MeshViewer::m_show3D;
		glutPostRedisplay();
		break;
	case 'e':
		MeshViewer::m_showError = !MeshViewer::m_showError;
		glutPostRedisplay();
		break;
	}
}

// Handling arrow key input
void handle_special_func(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_RIGHT:
		MeshViewer::m_rotationZ += 10;
		glutPostRedisplay();
		break;
	case GLUT_KEY_LEFT:
		MeshViewer::m_rotationZ -= 10;
		glutPostRedisplay();
		break;
	case GLUT_KEY_DOWN:
		MeshViewer::m_rotationX += 5;
		if (MeshViewer::m_rotationX > 0)
			MeshViewer::m_rotationX = 0;
		glutPostRedisplay();
		break;
	case GLUT_KEY_UP:
		MeshViewer::m_rotationX -= 5;
		if (MeshViewer::m_rotationX < -90)
			MeshViewer::m_rotationX = -90;
		glutPostRedisplay();
		break;
	}
}

void MeshViewer::InitializeVisualization()
{
	glutCreateWindow("SimpleFEM");

	glutReshapeWindow(600, 600);

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(handle_keyboard);
	glutSpecialFunc(handle_special_func);
}

void MeshViewer::RunVisualization()
{
	glutMainLoop();
}

void MeshViewer::CreateSolutionVisualization(const FEMMesh &mesh, const std::vector<double> &solution)
{
	double maxValue = 0;
	for (int i = 0; i < (int)solution.size(); i++)
		maxValue = std::max(solution[i], maxValue);

	glNewList(1, GL_COMPILE);
	createSolutionGeometry(mesh, solution, maxValue);
	glEndList();

	glNewList(2, GL_COMPILE);
	createSolutionWireframe(mesh, solution, maxValue);
	glEndList();
}

void MeshViewer::CreateErrorVisualization(const FEMMesh &mesh, const std::vector<double> &error)
{
	int n = error.size();
	double maxValue = 0;
	std::vector<double> abserror(error);
	for (int i = 0; i < n; i++)
	{
		abserror[i] = fabs(abserror[i]);
		maxValue = std::max(abserror[i], maxValue);
	}

	glNewList(3, GL_COMPILE);
	createSolutionGeometry(mesh, abserror, maxValue);
	glEndList();

	glNewList(4, GL_COMPILE);
	createSolutionWireframe(mesh, abserror, maxValue);
	glEndList();
}

bool MeshViewer::m_show3D = false;
bool MeshViewer::m_showError = false;
int MeshViewer::m_rotationX = -60;
int MeshViewer::m_rotationZ = 30;

void MeshViewer::display()
{
	// Clear the window
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	if (m_show3D)
	{
		glEnable(GL_DEPTH_TEST);

		glTranslatef(0.0f, 0.0f, -2.0f);
		glRotatef(m_rotationX, 1.0f, 0.0f, 0.0f);
		glRotatef(m_rotationZ, 0.0f, 0.0f, 1.0f);
		glTranslatef(-0.5f, -0.5f, 0.0f);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		gluPerspective(60.0f, 1.0f, 0.1f, 3.0f);

		if (m_showError)
			glCallList(3);
		else
			glCallList(1);

		// Hack: Add small offset to projection matrix so wireframe passes depth test where it should
		glTranslatef(0.0f, 0.0f, 0.002f);

		// Draw wireframe
		if (m_showError)
			glCallList(4);
		else
			glCallList(2);
	}
	else
	{
		glDisable(GL_DEPTH_TEST);

		// Map coordinates from [0,0] - [1,1] to [-1,-1] - [1,1]:
		glTranslatef(-1.0f, -1.0f, 0.0f);
		glScalef(2.0f, 2.0f, 0.0f);

		// Render the display list
		if (m_showError)
		{
			glCallList(3);
			glCallList(4);
		}
		else
		{
			glCallList(1);
			glCallList(2);
		}
	}

	if (m_show3D)
	{
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}

	glPopMatrix();

	// Bring the background buffer to the front:
	glutSwapBuffers();
}

void MeshViewer::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
}

void MeshViewer::HSV2RGB(float h, float s, float v, float &r, float &g, float &b)
{
	// Get hue in [0,1):
	h /= 360.0;
	if (h == 1)
		h = 0;

	h *= 6.0;

	// Get used segment i, i in [0,5]:
	int i = floor(h);

	// Get the fractional part of h:
	float f = h - i;
	float p = v*(1 - s);
	float q = v*(1 - (s*f));
	float t = v*(1 - (s*(1 - f)));

	// Cases for every segment:
	switch (i)
	{
	case 0: r = v; g = t; b = p; break;
	case 1: r = q; g = v; b = p; break;
	case 2: r = p; g = v; b = t; break;
	case 3: r = p; g = q; b = v; break;
	case 4: r = t; g = p; b = v; break;
	case 5: r = v; g = p; b = q; break;
	}
}

void MeshViewer::createSolutionGeometry(const FEMMesh &mesh, const std::vector<double> &solution, double maxValue)
{
	const float zStretch = 0.5f;

	// draw elements and solution
	glBegin(GL_TRIANGLES);

	// loop over elements
	for (int elID = 0; elID < mesh.GetNumElements(); elID++)
	{
		// get one
		const FEMElementTri &element = mesh.GetElement(elID);

		assert(element.GetNumElementNodes() == 3);

		// loop over nodes of element
		for (int elint = 0; elint < 3; elint++)
		{
			// get global id
			int nodeID = element.GetGlobalNodeForElementNode(elint);

			// get global position
			const Vector2 &pos = mesh.GetNodePosition(nodeID);
			float val = solution[nodeID] / maxValue;

			// convert value to color
			float s = 1.0f;
			float v = 1.0f;

			// Out of range -> black / red
			if (val < 0.0f) {
				val = 0.0f;
				v = 0.0f;
			}
			if (val > 1.0f) {
				val = 1.0f;
				v = 359.0f;
			}

			float h = (1.0f - val) * 240.0f;

			float r, g, b;
			HSV2RGB(h, s, v, r, g, b);

			// draw
			glColor3f(r, g, b);
			glVertex3f(pos[0], pos[1], val * zStretch);
		}
	}

	glEnd();
}

void MeshViewer::createSolutionWireframe(const FEMMesh &mesh, const std::vector<double> &solution, double maxValue)
{
	const float zStretch = 0.5f;

	// overdraw black lines
	glColor3f(0., 0., 0.);
	glBegin(GL_LINES);

	for (int elID = 0; elID < mesh.GetNumElements(); elID++)
	{
		const FEMElementTri &element = mesh.GetElement(elID);

		for (int elint = 0; elint < 3; elint++)
		{
			int nodeID1 = element.GetGlobalNodeForElementNode(elint);
			int nodeID2 = element.GetGlobalNodeForElementNode((elint + 1) % 3);

			const Vector2 &pos1 = mesh.GetNodePosition(nodeID1);
			const Vector2 &pos2 = mesh.GetNodePosition(nodeID2);

			// Load and clamp values
			float val1 = solution[nodeID1] / maxValue;
			float val2 = solution[nodeID2] / maxValue;
			val1 = (val1 > 1) ? 1 : ((val1 < 0) ? 0 : val1);
			val2 = (val2 > 1) ? 1 : ((val2 < 0) ? 0 : val2);

			glVertex3f(pos1[0], pos1[1], val1 * zStretch);
			glVertex3f(pos2[0], pos2[1], val2 * zStretch);
		}
	}

	glEnd();
}
