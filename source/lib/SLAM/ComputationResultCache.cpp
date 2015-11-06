//
// Created by LinKun on 10/26/15.
//

#include "SLAM/ComputationResultCache.h"


namespace NiS {

    bool SaveComputationResultCache(const std::string &file_name, const ComputationResultCache &cache) {

        std::ofstream out(file_name, std::ios::binary);

        if (out) {

            namespace bio = boost::iostreams;
            bio::filtering_ostream f;
            f.push(bio::gzip_compressor());
            f.push(out);

            boost::archive::binary_oarchive ar(out);
            ar << cache;

            return true;
        }

        return false;
    }

    bool LoadComputationResultCache(const std::string &file_name, ComputationResultCache &cache) {

        std::ifstream in(file_name, std::ios::binary);

        if (in) {

            namespace bio = boost::iostreams;
            bio::filtering_istream f;
            f.push(bio::gzip_decompressor());
            f.push(in);

            boost::archive::binary_iarchive ar(in);

            try {
                ar >> cache;
            }
            catch (const boost::archive::archive_exception &e) {
                return false;
            }
            return true;
        }

        return false;
    }
};