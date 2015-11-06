//
// Created by LinKun on 9/12/15.
//

#include "Core/Feature.h"

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

namespace NiS {

    template<>
    void Feature::Detect<cv::SiftFeatureDetector, cv::SiftDescriptorExtractor>(const cv::Mat_<uchar> &image,
                                                                               KeyPoints *key_points,
                                                                               Descriptors *descriptors) {

        if (!image.empty()) {

            // detecting keypoints
            cv::SiftFeatureDetector detector;
            detector.detect(image, *key_points);

            // computing descriptors
            cv::SiftDescriptorExtractor extractor;
            extractor.compute(image, *key_points, *descriptors);
        }
    };


    Feature::Feature()
            : type_(kTypeUnknown) { }

    Feature::Feature(const cv::Mat_<uchar> &image, Type type)
            : type_(type) {

        switch (type_) {

            case kTypeORB:
                Detect<cv::OrbFeatureDetector, cv::OrbDescriptorExtractor>(image, &key_points_, &descriptors_);
                break;

            case kTypeFREAK:
                Detect<cv::SurfFeatureDetector, cv::FREAK>(image, &key_points_, &descriptors_);
                break;

            case kTypeSIFT:
                Detect<cv::SiftFeatureDetector, cv::SiftDescriptorExtractor>(image, &key_points_, &descriptors_);
                break;

            case kTypeSURF:
                Detect<cv::SurfFeatureDetector, cv::SurfDescriptorExtractor>(image, &key_points_, &descriptors_);
                break;

            default:
                break;
        }
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
