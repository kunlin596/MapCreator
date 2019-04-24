//
// Created by LinKun on 9/12/15.
//

#include "Core/Feature.h"

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

namespace MapCreator {

    Feature::Feature()
            : type_(kTypeUnknown) { }

    Feature::Feature(const cv::Mat_<uchar> &image, Type type)
            : type_(type) {

        switch (type_) {
            case kTypeORB: { detector_ = cv::ORB::create(); break; }
#ifdef ENABLE_OPENCV_CONTRIB
            case kTypeFREAK: { detector_ = cv::xfeatures2d::FREAK::create(); break; }
            case kTypeSIFT: { detector_ = cv::xfeatures2d::SIFT::create(); break; }
            case kTypeSURF: { detector_ = cv::xfeatures2d::SURF::create(); break; }
#endif
            default: break;
        }

        detector_->detectAndCompute(image, cv::Mat(), key_points_, descriptors_);
    }

    Feature::~Feature() { }

    bool SaveFeature(const std::string &file_name, const Feature &feature) {

        std::ofstream out(file_name, std::ios::binary);

        if (out) {

            namespace bio = boost::iostreams;

            bio::filtering_ostream f;
            f.push(bio::gzip_compressor());
            f.push(out);

            boost::archive::binary_oarchive ar(out);
            ar << feature;
            return true;
        }
        return false;
    }

    bool LoadFeature(const std::string &file_name, Feature &feature) {

        std::ifstream in(file_name, std::ios::binary);

        if (in) {

            namespace bio = boost::iostreams;

            bio::filtering_istream f;
            f.push(bio::gzip_decompressor());
            f.push(in);

            boost::archive::binary_iarchive ar(in);
            ar >> feature;
            return true;
        }

        return false;
    }

}
