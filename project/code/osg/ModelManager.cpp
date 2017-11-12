/**
* \brief Functionality for managing loaded models to prevent loading multiple times the same model.
*
* \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
* \Date:   2017-11-11
*/

#include "ModelManager.h"
#include "ModelLoader.h"

using namespace pbs17;

ModelManager* ModelManager::_pInstance = nullptr;


/**
 * \brief Singleton instance of the CGameState-class.
 */
ModelManager* ModelManager::Instance() {
	// singleton-implementation => if there is not yet an instance initialized, create one.
	if (!_pInstance)
		_pInstance = new ModelManager();

	return _pInstance;
}


/**
 * \brief Load a Node-object from a model-file. It's implemented in a way that the model is loaded only once.
 * 
 * \param filePath
 *	    Complete path to the model to load.
 * \param ratio
 *      Ratio of the simplifier. (Supported values: [0..1])
 * \param scaling
 *      Scaling-factor to scale the model. (Supported values: ]0..inf]
 *
 *  \return Model-node to attach to osg-nodes.
 */
osg::ref_ptr<osg::Node> ModelManager::loadModel(std::string filePath, float ratio, float scaling) {
	// try to find the model, if it's found => return it and otherwise load it new
	osg::ref_ptr<osg::Node> retModel;
	std::map<std::string, osg::ref_ptr<osg::Node> >::iterator found = _loaded.find(filePath);

	if (found == _loaded.end()) {
		// model wasn't found => load and store it in the manager and return it
		retModel = ModelLoader::loadModel(filePath, ratio, scaling);
		_loaded.insert(std::pair<std::string, osg::ref_ptr<osg::Node> >(filePath, retModel));
	} else {
		// take found.
		retModel = _loaded[filePath];
	}

	return retModel;
}
