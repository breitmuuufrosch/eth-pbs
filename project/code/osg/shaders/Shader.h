/*!
\file
* \brief Header file for the class Shader.
*
* $Author: emcha1/pfafd1 $
* $Date: 2013-01-12 $
* $Revision: 1 $
*/

#pragma once

#include <osg/Program>


namespace pbs17 {
	//! The Shader is an interface for using shaders on a node.
	/*!
	First the shader-programms needs to be initialized before it can be applied to the node.
	*/
	class Shader {

	public:

		//! Constructor.
		Shader();


		//! Destructor.
		virtual ~Shader();


		//! Apply the shader-programm to the given node.
		/*!
		\param node The node which will contain the shader.
		*/
		virtual void apply(osg::Node* node) = 0;


		//! Get the vertex-shader-programm.
		/*!
		\return The vertex-shader-programm.
		*/
		const char* getVertShader() {
			return _vertSource;
		}


		//! Set the vertex-shader-programm.
		/*!
		\param shader The vertex-shader-programm.
		*/
		void setVertShader(char* shader) {
			_vertSource = shader;
		}


		//! Get the fragment-shader-programm.
		/*!
		\return The fragment-shader-programm.
		*/
		const char* getFragShader() {
			return _fragSource;
		}


		//! Set the fragment-shader-programm.
		/*!
		\param shader The fragment-shader-programm.
		*/
		void setFragShader(char* shader) {
			_fragSource = shader;
		}


	private:

		//! The program for the vertex-shader.
		const char* _vertSource;

		//! The porgramm for the fragment-shader.
		const char* _fragSource;

	};
}
