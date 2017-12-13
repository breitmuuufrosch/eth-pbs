#include "Sun.h"

#include <osg/Texture2D>
#include <osg/Material>
#include <osg/LightSource>

#include "../osg/visitors/ComputeTangentVisitor.h"
#include "../config.h"
#include "../osg/ImageManager.h"
#include "../osg/shaders/SunShader.h"


using namespace pbs17;


int Sun::LIGHT_ID = 0;


/**
 * \brief Constructor of Sun.
 *
 * \param size
 *      Size of the planet.
 */
Sun::Sun(double size)
    : Planet(size) {
	_lightId = LIGHT_ID;
	++LIGHT_ID;
}


Sun::Sun(json j)
    : Planet(j) {
	_lightId = LIGHT_ID;
	++LIGHT_ID;
}


osg::ref_ptr<osg::LightSource> Sun::addLight(osg::Vec4 color) {
	osg::Light *light = new osg::Light();

	// each light must have a unique number
	light->setLightNum(_lightId);

	// we set the light's position via a PositionAttitudeTransform object
	light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
	light->setDiffuse(color);
	light->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	light->setAmbient(osg::Vec4(255.0, 234.0, 160.0, 0.0) / 255.0);

	osg::StateSet *lightStateSet = _modelRoot->getOrCreateStateSet();
	osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
	lightSource->setLight(light);
	lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
	lightSource->setStateSetModes(*lightStateSet, osg::StateAttribute::ON);

	_modelRoot->addChild(lightSource);

	_light = lightSource;

	return lightSource;
}


void Sun::initTexturing() {
	bool useBumpmap = _bumpmapName != "";
	osg::ref_ptr<osg::StateSet> stateset = _convexRenderSwitch->getOrCreateStateSet();

	// Apply bumpmap-shaders
	ComputeTangentVisitor ctv;
	ctv.setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
	_modelFile->accept(ctv);

	// Load the texture
	if (_textureName != "") {
		std::string texturePath = DATA_PATH + "/texture/" + _textureName;
		std::cout << texturePath << std::endl;
		osg::ref_ptr<osg::Texture2D> colorTex = ImageManager::Instance()->loadTexture(texturePath);

		if (useBumpmap) {
			std::string bumpmapPath = DATA_PATH + "/texture/" + _bumpmapName;
			std::cout << bumpmapPath << std::endl;
			osg::ref_ptr<osg::Texture2D> normalTex = ImageManager::Instance()->loadTexture(bumpmapPath);

			SunShader bumpmapShader(colorTex, normalTex);
			bumpmapShader.apply(_convexRenderSwitch);
		} else {
			stateset->setTextureAttributeAndModes(0, colorTex.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		}
	}

	osg::ref_ptr<osg::Material> material = new osg::Material();
	material->setDiffuse(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 1.0));
	material->setSpecular(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
	material->setAmbient(osg::Material::FRONT, osg::Vec4(0.1, 0.1, 0.1, 1.0));
	material->setEmission(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
	material->setShininess(osg::Material::FRONT, 100);
	stateset->setAttribute(material);
}
