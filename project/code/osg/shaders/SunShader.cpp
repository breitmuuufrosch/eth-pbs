/*!
\file
* \brief Source file for the class BumpmapShader.
*
* $Author: emcha1/pfafd1 $
* $Date: 2013-01-12 $
* $Revision: 1 $
*/

#include "SunShader.h"

#include <osg/Node>
#include <osg/StateSet>
#include <osg/Program>
#include "../../config.h"

using namespace pbs17;



SunShader::SunShader(osg::ref_ptr<osg::Texture2D> texture, osg::ref_ptr<osg::Texture2D> normals)
	: _texture(texture), _normals(normals) {
	setVertShader("");

	setFragShader("");
}



SunShader::~SunShader() {}



void SunShader::apply(osg::Node* node) {
	osg::ref_ptr<osg::Shader> fragShader = new osg::Shader(osg::Shader::FRAGMENT);
	fragShader->loadShaderSourceFromFile(DATA_PATH + "/sunShader.glsl");

	osg::ref_ptr<osg::Program> program = new osg::Program;
	//program->addShader(new osg::Shader(osg::Shader::VERTEX, getVertShader()));
	program->addShader(fragShader);
	//program->addBindAttribLocation("tangent", 6);
	//program->addBindAttribLocation("binormal", 7);

	osg::StateSet* stateset = node->getOrCreateStateSet();
	stateset->setAttributeAndModes(program.get());
	stateset->addUniform(new osg::Uniform("colorTex", 0));
	stateset->addUniform(new osg::Uniform("random_noise", 7));
	//stateset->addUniform(new osg::Uniform("normalTex", 1));

	osg::StateAttribute::GLModeValue value = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;
	stateset->setTextureAttributeAndModes(0, _texture.get(), value);
	stateset->setTextureAttributeAndModes(1, _normals.get(), value);

}