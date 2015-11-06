//
// Created by LinKun on 9/13/15.
//

#ifndef LK_SLAM_MATCHER_H
#define LK_SLAM_MATCHER_H

#include <Core/Feature.h>
#include <Core/PointCloud.h>

#include <numeric>

namespace NiS {

    class Matcher {

    public:
        typedef std::pair<int, int> Match;        ///< マッチングを行った２つの Feature のキーポイントのインデックスの組
        typedef std::vector<Match> Matches;    ///< ２つの Feature から得られる Match たち
        typedef PointCloud::PointImage PointImage;

        Matcher(const Feature &feature1, const Feature &feature2, bool cross_check);

        Matcher(const Feature &feature1, const Feature &feature2, const PointImage &point_image1,
                const PointImage &point_image2, bool cross_check);

        Matcher(const Matches &matches);

        Matcher();

        ~Matcher();

        const Matches &GetMatches() const { return matches_; }

    private:

        Matches matches_;

        Matches CreateMatches(const Feature &feature1, const Feature &feature2, bool cross_check) const;

        Matches CreateValidMatches(const Feature &feature1, const Feature &feature2,
                                   const PointImage &point_image1, const PointImage &point_image2,
                                   bool cross_check) const;
    };
};

#endif //LK_SLAM_MATCHER_H
