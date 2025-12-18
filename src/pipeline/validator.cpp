#include "tracker/pipeline/validator.hpp"
#include <cmath>

namespace tracker::pipeline
{

ValidateResult Validator::validate(const TrackerPose& pose) const
{
    if (!pose.is_valid)
    {
        return ValidateResult::kInvalidPose;
    }

    if (pose.device_id < 0)
    {
        return ValidateResult::kDisconnected;
    }

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (!std::isfinite(pose.matrix(i, j)))
            {
                return ValidateResult::kInvalidPose;
            }
        }
    }

    return ValidateResult::kValid;
}

}


