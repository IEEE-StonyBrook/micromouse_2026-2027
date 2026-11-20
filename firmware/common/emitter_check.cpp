#include "common/emitter_check.h"

CrosstalkMatrix captureCrosstalk(IrBank& bank)
{
    CrosstalkMatrix m;
    m.channels = bank.channels();
    if (m.channels > IrBank::kMaxChannels)
        m.channels = IrBank::kMaxChannels;

    for (std::size_t e = 0; e < m.channels; ++e)
    {
        int32_t sum[IrBank::kMaxChannels] = {};
        for (int s = 0; s < CROSSTALK_SAMPLES; ++s)
        {
            // A dark read just before each lit one tracks drifting ambient light.
            uint16_t dark[IrBank::kMaxChannels] = {};
            uint16_t lit[IrBank::kMaxChannels]  = {};
            bank.readAll(IrBank::kAllOff, dark);
            bank.readAll(static_cast<int>(e), lit);
            for (std::size_t r = 0; r < m.channels; ++r)
                sum[r] += static_cast<int32_t>(lit[r]) - static_cast<int32_t>(dark[r]);
        }
        for (std::size_t r = 0; r < m.channels; ++r)
            m.counts[e][r] = sum[r] / CROSSTALK_SAMPLES;
    }
    return m;
}
