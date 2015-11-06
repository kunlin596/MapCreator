//
// Created by LinKun on 10/26/15.
//

#include "SLAM/Option.h"

namespace NiS {

    template<>
    Options::Options_OneByOne Options::GetOptions() { return options_one_by_one; }

    template<>
    Options::Options_FixedFrameCount Options::GetOptions() { return options_fixed_frame_count; }

    template<>
    Options::Options_PcaKeyFrame Options::GetOptions() { return options_pca_keyframe; }
}