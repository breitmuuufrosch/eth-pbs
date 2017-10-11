//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#ifndef MESH_VIEWER_H
#define MESH_VIEWER_H

class FEMMesh;

// Helper class for displaying an \c FEMMesh using OpenGL/GLUT
class MeshViewer
{
public:
	MeshViewer(int argc, char *argv[]);

	// Initializes the visualization
	void InitializeVisualization();
	// Run the visualization
	void RunVisualization();

	// Displays the solution \c solution for mesh \c mesh.
	void CreateSolutionVisualization(const FEMMesh &mesh, const std::vector<double> &solution);
	// Displays the error \c erro for mesh \c mesh.
	void CreateErrorVisualization(const FEMMesh &mesh, const std::vector<double> &error);

	// Flag to show the 3D view of the mesh
	static bool m_show3D;
	// Flag to show error (instead of regular visualization)
	static bool m_showError;
	// Rotation for 3D view
	static int m_rotationX;
	static int m_rotationZ;

private:
	// GLUT display callback
	static void display();

	// GLUT reshape callback
	static void reshape(int w, int h);

	// Converts a HSV color to RGB
	static void HSV2RGB(float h, float s, float v, float &r, float &g, float &b);

	// Creates geometry that represents the solution for mesh \c mesh
	static void createSolutionGeometry(const FEMMesh &mesh, const std::vector<double> &solution, double maxValue);
	// Creates wireframe that represents the solution for mesh \c mesh
	static void createSolutionWireframe(const FEMMesh &mesh, const std::vector<double> &solution, double maxValue);
};

#endif
