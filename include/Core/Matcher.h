//
// Created by LinKun on 9/13/15.
//

#pragma once

#include "Core/Feature.h"
#include "Core/PointCloud.h"

namespace MapCreator {

/**
 * @brief      Class for matcher.
 */
class Matcher2D {
 public:
  typedef std::pair<int, int> Match;  ///< マッチングを行った２つの Feature
                                      ///< のキーポイントのインデックスの組
  using Matches =
      std::vector<Match>;  ///< ２つの Feature から得られる Match たち

  Matcher2D(const Feature &feature1, const Feature &feature2, bool cross_check);

  Matcher2D(const Matches &matches);

  virtual ~Matcher2D() {};

  /**
   * @brief      Gets the matches.
   *
   * @return     The matches.
   */
  const Matches &GetMatches() const { return matches_; }

 protected:
  Matches matches_;

  /**
   * @brief      Creates matches.
   *
   * @param[in]  feature1     The feature 1
   * @param[in]  feature2     The feature 2
   * @param[in]  cross_check  The cross check
   *
   * @return     { description_of_the_return_value }
   */
  Matches CreateMatches(const Feature &feature1, const Feature &feature2,
                        bool cross_check) const;
};

class Matcher3D : public Matcher2D {
 public:
  Matcher3D(const Feature &feature1, const Feature &feature2,
            const PointImage &point_image1, const PointImage &point_image2,
            bool cross_check);

 protected:
  /**
   * @brief      Creates valid matches.
   *
   * @param[in]  feature1      The feature 1
   * @param[in]  feature2      The feature 2
   * @param[in]  point_image1  The point image 1
   * @param[in]  point_image2  The point image 2
   * @param[in]  cross_check   The cross check
   *
   * @return     { description_of_the_return_value }
   */
  Matches CreateValidMatches(const Feature &feature1, const Feature &feature2,
                             const PointImage &point_image1,
                             const PointImage &point_image2,
                             bool cross_check) const;
};

}  // namespace MapCreator
