/**
* \brief Conversions between json- and eigen-datatypes.
*
* \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
* \Date:   2017-11-29
*/

#pragma once

#include <Eigen/Core>
#include <json.hpp>

using json = nlohmann::json;

namespace pbs17 {
	inline Eigen::Vector3d fromJson(json j) {
		return Eigen::Vector3d(
			j["x"].get<double>(),
			j["y"].get<double>(),
			j["z"].get<double>()
		);
	}
}