/*!
\file
* \brief Source file for the class BumpmapShader.
*
* $Author: emcha1/pfafd1 $
* $Date: 2013-01-12 $
* $Revision: 1 $
*/

#include "BumpmapShader.h"

#include <osg/Node>
#include <osg/StateSet>
#include <osg/Program>

using namespace pbs17;



BumpmapShader::BumpmapShader(osg::ref_ptr<osg::Texture2D> texture, osg::ref_ptr<osg::Texture2D> normals)
	: _texture(texture), _normals(normals) {
	setVertShader(
		"attribute vec3 tangent;\n"
		"attribute vec3 binormal;\n"
		"varying vec3 lightDir;\n"
		"varying vec3 position;\n"
		"void main()\n"
		"{\n"
		"    vec3 normal = normalize(gl_NormalMatrix * gl_Normal);\n"
		"    mat3 rotation = mat3(tangent, binormal, normal);\n"
		"    vec4 vertexInEye = gl_ModelViewMatrix * gl_Vertex;\n"
		"    lightDir = vec3(gl_LightSource[0].position.xyz - vertexInEye.xyz);\n"
		"    lightDir = normalize(normalize(lightDir) * rotation);\n"
		"    gl_Position = ftransform();\n"
		"	 position = gl_Vertex.xyz;\n"
		"    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
		"}\n"
	);

	setFragShader(
		"uniform sampler2D colorTex;\n"
		"uniform sampler2D normalTex;\n"
		"uniform vec3 CAMERA_POSITION;\n"
		"varying vec3 lightDir;\n"
		"varying vec3 position;\n"

		"void main (void)\n"
		"{\n"
		"    vec4 base = texture2D(colorTex, gl_TexCoord[0].xy);\n"
		"    vec3 bump = texture2D(normalTex, gl_TexCoord[0].xy).xyz;\n"
		"    bump = normalize(bump * 2.0 - 1.0);\n"

		"    float lambert = max(dot(bump, lightDir), 0.0);\n"
		"	 gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);\n"
		"	 vec4 diffuse = vec4(0.0, 0.0, 0.0, 0.0);\n"
		"	 vec4 specular = vec4(0.0, 0.0, 0.0, 0.0);\n"
		"	 vec4 ambient = gl_LightSource[0].ambient;\n"

		"    if (lambert > 0.0)\n"
		"    {\n"
		"        diffuse = base * gl_LightSource[0].diffuse * lambert;\n"
		"        specular = gl_LightSource[0].specular * pow(lambert, gl_FrontMaterial.shininess);\n"
		"    }\n"
		"    gl_FragColor = ambient + diffuse + specular;\n"
		"}\n"
	);
}



BumpmapShader::~BumpmapShader() {}



void BumpmapShader::apply(osg::Node* node) {
	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->addShader(new osg::Shader(osg::Shader::VERTEX, getVertShader()));
	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, getFragShader()));
	program->addBindAttribLocation("tangent", 6);
	program->addBindAttribLocation("binormal", 7);

	osg::StateSet* stateset = node->getOrCreateStateSet();
	stateset->setAttributeAndModes(program.get());
	stateset->addUniform(new osg::Uniform("colorTex", 0));
	stateset->addUniform(new osg::Uniform("normalTex", 1));

	osg::StateAttribute::GLModeValue value = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;
	stateset->setTextureAttributeAndModes(0, _texture.get(), value);
	stateset->setTextureAttributeAndModes(1, _normals.get(), value);

}