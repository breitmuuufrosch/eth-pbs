/**
* \brief Functionality for managing loaded models to prevent loading multiple times the same model.
*
* \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
* \Date:   2017-11-11
*/

#pragma once

#include <map>

#include <osg/Node>


namespace pbs17 {

	/**
	 * \brief ModelManager manages already loaded models.
	 * This class prevents to load the same model several times. If a model is requested which was already loaded, it will return the already loaded model. Otherwise it will load it into the cache.
	 */
	class ModelManager {
	public:

		/**
		 * \brief Singleton instance of the CGameState-class.
		 */
		static ModelManager* Instance();


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
		osg::ref_ptr<osg::Node> loadModel(std::string filePath, float ratio = 1.0, float scaling = 1.0);


	private:

		//! All models which have been loaded already.
		std::map<std::string, osg::ref_ptr<osg::Node>> _loaded;

		//! Private constructor to be sure the class can't be created outside of this class.
		ModelManager() {};

		//! Private copy-constructor to prevent copying the class.
		ModelManager(ModelManager const&) {};

		//! Private assignment-operator to prevent copying/saving referencec to the class.
		ModelManager& operator=(ModelManager const&) {};

		//! Private member-var which contains the singleton of the class.
		static ModelManager* _pInstance;
	};
}
