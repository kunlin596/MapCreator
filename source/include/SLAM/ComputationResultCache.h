//
// Created by LinKun on 10/26/15.
//

#ifndef NIS_COMPUTATIONRESULTCACHE_H
#define NIS_COMPUTATIONRESULTCACHE_H


#include <Core/Serialize.h>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "SLAM/Option.h"

namespace NiS {

	struct ComputationResultCache
	{
		std::string               data_set_name;
		int                       computation_time;
		Options                   options;
		std::vector < int >       indices;
		std::vector < glm::mat4 > estimation_matrices;
		std::vector < glm::mat4 > marker_matrices;

		template < typename Archive >
		void serialize ( Archive & ar , const unsigned int version ) {

			ar & data_set_name;
			ar & computation_time;
			ar & options;
			ar & indices;
			ar & estimation_matrices;
			ar & marker_matrices;
		}

	};

	bool SaveComputationResultCache ( const std::string & file_name , const ComputationResultCache & cache );

	bool LoadComputationResultCache ( const std::string & file_name , ComputationResultCache & cache );

}

#endif //NIS_COMPUTATIONRESULTCACHE_H
