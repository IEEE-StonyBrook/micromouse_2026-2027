#include "drivers/motor_bridge.h"

#include <algorithm>
#include <cmath>

#include "hal/irq.h"

MotorBridge::MotorBridge(hal::HBridgePort& port, float amps_per_count)
    : port_(port), amps_per_count_(amps_per_count)
{
    sleepNow();
}

void MotorBridge::set_duty(float left, float right)
{
    hal::irq::Lock lock;
    duty_[0] = std::clamp(left, -1.0f, 1.0f);
    duty_[1] = std::clamp(right, -1.0f, 1.0f);
}

void MotorBridge::wake()
{
    hal::irq::Lock lock;
    wake_request_ = true;
}

void MotorBridge::sleep()
{
    hal::irq::Lock lock;
    duty_[0]      = 0.0f;
    duty_[1]      = 0.0f;
    wake_request_ = false;
    sleepNow();
}

void MotorBridge::sleepNow()
{
    // Inputs low first: if nSLEEP is slow to fall, the bridge coasts meanwhile.
    port_.setInputs(0, 0.0f, 0.0f);
    port_.setInputs(1, 0.0f, 0.0f);
    port_.setAwake(false);
    state_.store(static_cast<uint8_t>(State::kAsleep));
    amps_[0].store(0.0f);
    amps_[1].store(0.0f);
    asleep_ticks_ = 0;
}

void MotorBridge::driveOutputs()
{
    for (int m = 0; m < 2; m++)
    {
        // Slow decay: one input stays high and the other is low for |duty| of
        // the period, so the bridge alternates drive and brake. 0 is 11, a brake.
        const float off = 1.0f - std::fabs(duty_[m]);
        if (duty_[m] >= 0.0f)
            port_.setInputs(m, 1.0f, off);
        else
            port_.setInputs(m, off, 1.0f);
    }
}

void MotorBridge::update()
{
    switch (state())
    {
        case State::kAsleep:
        {
            if (wake_request_ || duty_[0] != 0.0f || duty_[1] != 0.0f)
            {
                wake_request_ = false;
                port_.setAwake(true);
                state_.store(static_cast<uint8_t>(State::kWaking));
                wake_ticks_ = 0;
                return;
            }
            // Asleep, IPROPI sources nothing: what the ADC reads is its offset.
            if (asleep_ticks_ < SLEEP_SETTLE_TICKS)
            {
                asleep_ticks_++;
                return;
            }
            for (int m = 0; m < 2; m++)
            {
                const float counts = static_cast<float>(port_.senseCounts(m));
                if (asleep_ticks_ == SLEEP_SETTLE_TICKS)
                    offset_counts_[m] = counts; // first settled sample seeds it
                else
                    offset_counts_[m] += (counts - offset_counts_[m]) * (1.0f / 16.0f);
            }
            asleep_ticks_ = SLEEP_SETTLE_TICKS + 1;
            return;
        }
        case State::kWaking:
            // The inputs are ignored until tWAKE; the command waits in duty_.
            if (++wake_ticks_ < WAKE_TICKS)
                return;
            state_.store(static_cast<uint8_t>(State::kAwake));
            idle_ticks_   = 0;
            wake_request_ = false; // a wake() while waking is already served
            break;
        case State::kAwake:
            wake_request_ = false; // already awake: nothing left to wake
            break;
    }

    driveOutputs();
    for (int m = 0; m < 2; m++)
    {
        const float counts = static_cast<float>(port_.senseCounts(m)) - offset_counts_[m];
        amps_[m].store(std::max(counts, 0.0f) * amps_per_count_);
    }

    idle_ticks_ = (duty_[0] == 0.0f && duty_[1] == 0.0f) ? idle_ticks_ + 1 : 0;
    if (idle_ticks_ >= IDLE_SLEEP_TICKS)
        sleepNow();
}

bool MotorBridge::readAmps(void* bridge, float* left_amps, float* right_amps)
{
    const MotorBridge* b = static_cast<const MotorBridge*>(bridge);
    if (b->state() != State::kAwake)
        return false;
    *left_amps  = b->amps(0);
    *right_amps = b->amps(1);
    return true;
}
