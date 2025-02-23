#include "time.h"

namespace CamSim::Time {

double Timestamp::get_utc_leap_seconds(const ulong seconds)
{
    // Get first leap second timestamp that is less than seconds
    for (auto it = leap_seconds_table.rbegin(); it != leap_seconds_table.rend(); ++it)
    {
        const auto& [valid_after_timestamp, leap_seconds] = *it;
        if (seconds >= valid_after_timestamp)
        {
            return leap_seconds;
        }
    }

    return 0.0;
}

}
