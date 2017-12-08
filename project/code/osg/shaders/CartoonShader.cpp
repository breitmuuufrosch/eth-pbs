/*!
\file
* \brief Source file for the class CartoonShader.
*
* $Author: emcha1/pfafd1 $
* $Date: 2013-01-12 $
* $Revision: 1 $
*/

#include "CartoonShader.h"

#include <osg/Node>
#include <osg/StateSet>
#include <osg/Program>

using namespace pbs17;



CartoonShader::CartoonShader(osg::Vec4d color) : _color(color) {
	setVertShader(
		"varying vec3 normal;\n"
		"void main()\n"
		"{\n"
		" normal = normalize(gl_NormalMatrix * gl_Normal);\n"
		" gl_Position = ftransform();\n"
		"}\n"
	);

	setFragShader(
		"uniform vec4 color1;\n"
		"uniform vec4 color2;\n"
		"uniform vec4 color3;\n"
		"uniform vec4 color4;\n"
		"varying vec3 normal;\n"
		"void main()\n"
		"{\n"
		" float intensity = dot(vec3(gl_LightSource[0].position),normal);\n"
		" if (intensity > 0.95) gl_FragColor = color1;\n"
		" else if (intensity > 0.5) gl_FragColor = color2;\n"
		" else if (intensity > 0.25) gl_FragColor = color3;\n"
		" else gl_FragColor = color4;\n"
		"}\n"
	);
}



CartoonShader::~CartoonShader() {}



void CartoonShader::apply(osg::Node* node) {
	osg::ref_ptr<osg::Shader> vertShader = new osg::Shader(osg::Shader::VERTEX, getVertShader());
	osg::ref_ptr<osg::Shader> fragShader = new osg::Shader(osg::Shader::FRAGMENT, getFragShader());
	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->addShader(vertShader.get());
	program->addShader(fragShader.get());

	osg::StateSet* stateset = node->getOrCreateStateSet();
	stateset->setAttributeAndModes(program.get());
	stateset->addUniform(new osg::Uniform("color1", _color));
	stateset->addUniform(new osg::Uniform("color2", osg::Vec4(_color.x() / 2, _color.y() / 2, _color.z() / 2, 1.0f)));
	stateset->addUniform(new osg::Uniform("color3", osg::Vec4(_color.x() / 4, _color.y() / 4, _color.z() / 4, 1.0f)));
	stateset->addUniform(new osg::Uniform("color4", osg::Vec4(_color.x() / 8, _color.y() / 8, _color.z() / 8, 1.0f)));
}