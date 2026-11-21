#ifndef CONTROL_RUN_LOG_H
#define CONTROL_RUN_LOG_H

#include <cstddef>
#include <cstdint>

/**
 * @brief The run log: what the control loop did during a run, for dumping
 * after the mouse is disarmed.
 *
 * Motion records one sample every `dtMs()` while the log is active. Each
 * sample packs 18 physical fields into int16 steps plus a word of event bits
 * (38 bytes), so the ring holds CAPACITY samples, 13.4 s at the default 8 ms,
 * in under 63 KB. On the STM32 the firmware's log (runLog()) sits in the
 * 64 KB CCM, which DMA cannot reach: a dump unpacks each sample into an SRAM
 * copy and formats it into an SRAM row before the console queues it.
 *
 * The header says what the run was: a run counter, the battery and the
 * command text, set by the main loop with set_header() just before it arms
 * the trace. The main loop also names the PATH segment executing with
 * set_segment(); every sample carries it in event bits 8-13.
 *
 * Threading: set_header(), set_segment(), the readers and formatHeader() run
 * on the main loop; start() and stop() run in the tick (Motion's latch
 * applies them); due() and record() run in the control tick. Read only while
 * the log is stopped.
 */
class RunLog
{
  public:
    /// One sample in physical units. Unpacked values are rounded to the field's step.
    struct Sample
    {
        float    target_deg    = 0.0f; // rotation profile position, relative to turn start
        float    yaw_deg       = 0.0f; // IMU robot angle (absolute)
        float    rot_error_deg = 0.0f; // rotation PD accumulated error
        float    rot_volts     = 0.0f; // rotation PD output
        float    hold_err_deg  = 0.0f; // heading hold error
        float    wall_degps    = 0.0f; // ToF wall steering fed to the rotation PD
        float    hold_degps    = 0.0f; // IMU heading hold fed to the rotation PD
        float    fwd_mmps      = 0.0f; // forward profile velocity
        float    left_mmps     = 0.0f; // measured wheel velocity
        float    right_mmps    = 0.0f;
        float    left_volts    = 0.0f;
        float    right_volts   = 0.0f;
        float    left_tof_mm   = 0.0f; // filtered ToF distances as last set by the main loop
        float    front_tof_mm  = 0.0f;
        float    right_tof_mm  = 0.0f;
        float    pos_mm        = 0.0f; // forward profile position (the search's cell frame)
        float    enc_l_mm      = 0.0f; // encoder distance per wheel since the trace started
        float    enc_r_mm      = 0.0f;
        uint16_t events        = 0; // Event bits
    };

    /// Event bits: what the loop was doing when the sample was taken.
    enum Event : uint16_t
    {
        ENCODER_ROTATION = 1u << 0, // rotation PD fed from encoders, not the IMU
        WALL_LEFT        = 1u << 1, // walls Motion saw at this sample
        WALL_FRONT       = 1u << 2,
        WALL_RIGHT       = 1u << 3,
        VOLTS_SATURATED  = 1u << 4,    // a wheel command hit MAX_VOLTAGE
        FORWARD_ACTIVE   = 1u << 5,    // forward profile still moving
        ROTATION_ACTIVE  = 1u << 6,    // rotation profile still moving
        SEGMENT_MASK     = 0x3Fu << 8, // bits 8-13: set_segment()'s index mod 64
        CLIPPED          = 1u << 15,   // a field fell outside its range and was clamped
    };
    static constexpr int SEGMENT_SHIFT = 8;
    /// The segment index (mod 64) packed into a sample's event bits.
    static uint8_t segment(uint16_t events)
    {
        return static_cast<uint8_t>((events & SEGMENT_MASK) >> SEGMENT_SHIFT);
    }

    static constexpr size_t CMD_BYTES = 96; // command text, NUL included
    /// What the run was, for the dump's header line.
    struct Header
    {
        uint32_t run_id         = 0; // counts set_header() calls since boot
        float    battery_v      = 0.0f;
        char     cmd[CMD_BYTES] = {};
    };

    /// A packed sample. Each field stores round(value * steps per unit),
    /// clamped to +-32767; INT16_MIN stands for NaN.
    static constexpr int FIELDS = 18;
    struct Packed
    {
        int16_t  field[FIELDS];
        uint16_t events;
    };

    static constexpr uint16_t CAPACITY          = 1680; // 13.44 s at 8 ms, 63840 bytes
    static constexpr uint16_t DEFAULT_PERIOD_MS = 8;
    static constexpr size_t   ROW_BYTES         = 224; // formatRow() buffer size
    static constexpr size_t   HEADER_BYTES      = 448; // formatHeader() buffer size
    /// Column names of formatRow(): the old turn-trace columns, the encoder
    /// distances and `ev`. Parsers go by name: columns only ever get inserted.
    static constexpr const char* COLUMNS =
        "t_ms,target_deg,yaw_deg,rot_err_deg,rot_v,hold_err_deg,wall_degps,hold_degps,"
        "fwd_mmps,left_mmps,right_mmps,left_v,right_v,enc,tof_l_mm,tof_f_mm,tof_r_mm,pos_mm,"
        "enc_l_mm,enc_r_mm,ev";

    static Packed pack(const Sample& sample);
    static Sample unpack(const Packed& packed);

    /// One CSV row, prefixed "T,", for the sample at `index` of a dump taken every `dt_ms`.
    static void formatRow(char* out, size_t size, uint16_t index, uint16_t dt_ms,
                          const Sample& sample);

    /// The dump's first line: `TRACE n=.. dt_ms=.. cols=.. run=.. loop_hz=..
    /// batt_v=.. wrapped=0|1 cmd="..."`. Pico and STM32 dumps both print it.
    void formatHeader(char* out, size_t size) const;

    /// Main loop, before arming a trace: counts a new run, records the battery
    /// and the command (truncated; quotes and control characters replaced so
    /// the header stays one line) and resets the segment to 0.
    void set_header(const char* cmd, float battery_v);
    /// Main loop: the segment index the following samples carry.
    void          set_segment(uint8_t index) { segment_ = static_cast<uint8_t>(index % 64u); }
    const Header& header() const { return header_; }

    /// Empties the log and records from now on, one sample per `period_ms`
    /// (rounded to whole control ticks, at least one).
    void start(uint16_t period_ms = DEFAULT_PERIOD_MS);
    void stop();
    bool active() const { return active_; }

    /// Control tick: call once per tick; true when a sample is due.
    bool due();
    /// Control tick: stores a sample, overwriting the oldest once full.
    void record(const Sample& sample);

    uint16_t count() const { return count_; }
    /// True once the ring overwrote samples: the start of the run is lost.
    bool wrapped() const { return wrapped_; }
    /// The sample period, ms, as whole control ticks give it.
    uint16_t dtMs() const;
    /// Sample `index`, oldest first, unpacked into a copy.
    Sample sample(uint16_t index) const;

  private:
    // Every member is zero until start(): the firmware's log lives in CCM,
    // where startup zeroes memory but never copies an initialiser from flash.
    Packed            samples_[CAPACITY];
    volatile uint16_t count_        = 0;
    uint16_t          next_         = 0; // ring write index
    uint16_t          tick_         = 0;
    uint16_t          period_ticks_ = 0;
    volatile bool     active_       = false;
    volatile bool     wrapped_      = false;
    volatile uint8_t  segment_      = 0;
    Header            header_;
};

/// The firmware's run log: in CCM on the STM32, in .bss elsewhere.
RunLog& runLog();

#endif
