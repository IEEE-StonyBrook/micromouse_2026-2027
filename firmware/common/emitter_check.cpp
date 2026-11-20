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

CrosstalkVerdict judgeCrosstalk(const CrosstalkMatrix& m)
{
    CrosstalkVerdict v;
    for (std::size_t r = 0; r < m.channels; ++r)
    {
        const int32_t own = m.counts[r][r];
        if (own < CROSSTALK_MIN_DIAGONAL_COUNTS)
        {
            v.weak_mask |= static_cast<uint8_t>(1u << r);
            continue;
        }
        for (std::size_t e = 0; e < m.channels; ++e)
        {
            if (e != r && m.counts[e][r] * 100 > own * CROSSTALK_MAX_LEAK_PERCENT)
                v.leaks[v.leak_count++] = {static_cast<uint8_t>(e), static_cast<uint8_t>(r)};
        }
    }
    v.pass = m.channels > 0 && v.weak_mask == 0 && v.leak_count == 0;
    return v;
}
