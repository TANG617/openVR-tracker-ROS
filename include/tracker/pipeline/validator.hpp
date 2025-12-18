#pragma once

#include "tracker/types.hpp"
#include "tracker/pipeline/types.hpp"

namespace tracker::pipeline
{

class Validator
{
public:
    [[nodiscard]] ValidateResult validate(const TrackerPose& pose) const;
};

}


