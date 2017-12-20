/**
 * \brief Implementation of the space object (base class).
 *
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-11-11
 */

#include "SpaceObject.h"

#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osg/Material>

#include "../osg/visitors/BoundingBoxVisitor.h"
#include "../osg/ImageManager.h"
#include "../osg/visitors/ComputeTangentVisitor.h"
#include "../osg/shaders/BumpmapShader.h"
#include "../config.h"
#include "../osg/FollowingRibbon.h"
#include "../osg/visitors/TrailerCallback.h"

using namespace pbs17;


//! ID's start from 0.
long SpaceObject::RunningId = 0;


/**
* \brief Constructor of SpaceObject.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
*/
SpaceObject::SpaceObject(std::string filename, int i)
    : SpaceObject(filename, "") {
    std::cout << "should not be called" << std::endl;
}

SpaceObject::SpaceObject(json j) {
	_textureName = j["texture"].is_string() ? j["texture"].get<std::string>() : "";
	_bumpmapName = j["bumpmap"].is_string() ? j["bumpmap"].get<std::string>() : "";
    
	_filename = j["obj"].is_string()? j["obj"].get<std::string>(): "";
    _id = RunningId;
    ++RunningId;

    _position = Eigen::Vector3d(0, 0, 0);
    _orientation = osg::Quat(0, osg::X_AXIS);
;

    // For visually debuggin => Make bounding-box visible
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    _aabbShape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1.0f));
    _aabbShape->setColor(osg::Vec4(1.0, 0, 0, 1.0));
    geode->addDrawable(_aabbShape);

    _aabbRendering = new osg::MatrixTransform;
    _aabbRendering->setNodeMask(0x1);
    _aabbRendering->addChild(geode.get());
    osg::StateSet* ss = _aabbRendering->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setAttributeAndModes(new osg::PolygonMode(
        osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
}


/**
* \brief Constructor of SpaceObject.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
* \param textureName
*      Relative location to the texture-file. (Relative from the data-directory in the source).
*/
SpaceObject::SpaceObject(std::string filename, std::string textureName)
	: _filename(filename), _textureName(textureName) {
	_id = RunningId;
	++RunningId;

	_position = Eigen::Vector3d(0, 0, 0);
	_orientation = osg::Quat(0, osg::X_AXIS);


	// For visually debuggin => Make bounding-box visible
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	_aabbShape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1.0f));
	_aabbShape->setColor(osg::Vec4(1.0, 0, 0, 1.0));
	geode->addDrawable(_aabbShape);

	_aabbRendering = new osg::MatrixTransform;
	_aabbRendering->setNodeMask(0x1);
	_aabbRendering->addChild(geode.get());
	osg::StateSet* ss = _aabbRendering->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	ss->setAttributeAndModes(new osg::PolygonMode(
		osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
}


/**
 * \brief Destructor of SpaceObject.
 */
SpaceObject::~SpaceObject() {
	if (_modelRoot) {
		_modelRoot = nullptr;
	}
}


/**
 * \brief Initialize the space-object for physics.
 *
 * \param mass
 *      Mass: unit = kg
 * \param linearVelocity
 *      Linear velocity: unit = m/s
 * \param angularVelocity
 *      Angular velocity: unit = rad/s
 * \param force
 *      Global force: unit = vector with norm equals to N
 * \param torque
 *      Global torque: unit = vector with norm equals to N*m (newton metre)
 */
void SpaceObject::initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) {
	_mass = mass;
	_linearVelocity = linearVelocity;
	_angularVelocity = angularVelocity;
	_force = force;
	_torque = torque;
}


void SpaceObject::updatePositionOrientation(Eigen::Vector3d p, osg::Quat newOrientation) {
	_position = p;
    _orientation = newOrientation;

	osg::Matrixd rotation;
    newOrientation.get(rotation);
	osg::Matrixd translation = osg::Matrix::translate(toOsg(p));

    _transformation->setMatrix(rotation * translation);

	updateAABB();
}


void SpaceObject::calculateAABB() {
	osg::Matrix scaling = osg::Matrix::scale(_scaling, _scaling, _scaling);
	osg::Matrix translation = osg::Matrix::translate(toOsg(_position));
	osg::Matrix rotation;
	_orientation.get(rotation);

	CalculateBoundingBox bbox(scaling * rotation * translation, scaling);
	_modelFile->accept(bbox);
	_aabbLocal = bbox.getLocalBoundBox();
	_aabbGlobal = bbox.getGlobalBoundBox();
	_aabbLocalOrig = bbox.getLocalBoundBox();
	_aabbGlobalOrig = bbox.getGlobalBoundBox();

	_aabbRendering->setMatrix(osg::Matrix::scale(_aabbGlobal._max - _aabbGlobal._min) * osg::Matrix::translate(toOsg(_position)));
}

void SpaceObject::updateAABB() {
	osg::Matrix translation = osg::Matrix::translate(toOsg(_position));
	osg::Matrix rotation;
	_orientation.get(rotation);

	osg::Matrix localToWorld = rotation * translation;

	osg::BoundingBox newGlobal;

	for (unsigned int i = 0; i < 8; ++i)
		newGlobal.expandBy(_aabbLocal.corner(i) * localToWorld);

	_aabbGlobal = newGlobal;
	_aabbRendering->setMatrix(osg::Matrix::scale(_aabbGlobal._max - _aabbGlobal._min) * osg::Matrix::translate(toOsg(_position)));
}


void SpaceObject::resetCollisionState() {
	if (_collisionState == 0) {
		//ColorVisitor colorVisitor(osg::Vec4(1, 1, 1, 1));
		//_aabbRendering->accept(colorVisitor);
		_aabbShape->setColor(osg::Vec4(1, 1, 1, 1));
	}

	_collisionState = 0;
}

void SpaceObject::setCollisionState(int c) {
	_collisionState = std::max(_collisionState, c);

	_aabbShape->setColor(c == 1 ? osg::Vec4(0, 1, 0, 1) : osg::Vec4(1, 0, 0, 1));
}


/**
 * \brief Initialize the texture-properties and shader.
 */
void SpaceObject::initTexturing() {
	bool useBumpmap = _bumpmapName != "";
	osg::ref_ptr<osg::StateSet> stateset = _convexRenderSwitch->getOrCreateStateSet();

	// Apply bumpmap-shaders
	ComputeTangentVisitor ctv;
	ctv.setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
	_modelFile->accept(ctv);

	// Load the texture
	if (_textureName != "") {
		std::string texturePath = DATA_PATH + "/texture/" + _textureName;
		osg::ref_ptr<osg::Texture2D> colorTex = ImageManager::Instance()->loadTexture(texturePath);

		if (useBumpmap) {
			std::string bumpmapPath = DATA_PATH + "/texture/" + _bumpmapName;
			osg::ref_ptr<osg::Texture2D> normalTex = ImageManager::Instance()->loadTexture(bumpmapPath);

			BumpmapShader bumpmapShader(colorTex, normalTex);
			bumpmapShader.apply(_convexRenderSwitch);
		} else {
			stateset->setTextureAttributeAndModes(0, colorTex.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		}
	}

	// Define material-properties
	osg::ref_ptr<osg::Material> material = new osg::Material();
	material->setDiffuse(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 1.0));
	material->setSpecular(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
	material->setAmbient(osg::Material::FRONT, osg::Vec4f(0.4f, 0.4f, 0.4f, 1.0f));
	material->setEmission(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
	material->setShininess(osg::Material::FRONT, 100);
	stateset->setAttribute(material);
}

void SpaceObject::initFollowingRibbon(osg::Vec3 color, unsigned int numPoints, float halfWidth) {
	FollowingRibbon* ribbon = new FollowingRibbon();
	osg::Geometry* geometry = ribbon->init(toOsg(_position), color, numPoints, halfWidth);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geometry);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	_transformation->addUpdateCallback(new TrailerCallback(ribbon, geometry));
	_modelRoot->addChild(geode);
}

