// STM32F405 bring-up entry point: blinks the Feather's red LED at 1 Hz, or
// the clock-fault pattern (three quick flashes every 2 s) when the 12 MHz
// crystal failed to start and the core runs from the HSI instead. A flash
// store fault shows instead of the 1 Hz blink until the next good save: two
// flashes when a damaged slot was skipped, four when no settings loaded.
//
// At boot the board reads its chip UID and loads the profile it saved under
// it from the flash store (sectors 1-2). An unknown board prints the presets
// and refuses to arm until one is picked.
//
// A bring-up shell runs on the USART3 console (Feather TX/RX, 1 Mbaud, 8N1):
// it echoes what it receives and answers these lines:
//   stats        tick timing, encoders, IMU, console drop counters, memory use
//   flood        queues 64 KB at once, to show the TX ring dropping and counting
//   profile      this board's UID and profile, or the presets to pick from
//   profile <n>  saves preset n as this board's profile (erases a flash sector)
//   arm          says whether this board may arm; nothing moves in bring-up
//   logfill      fills the run log (in CCM) with a full ring of synthetic samples
//   log          dumps the run log as TRACE CSV rows
// Memory use and tick timing are also left in `memory_usage` and `tick_stats`
// for a debugger to read.

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <libopencm3/stm32/gpio.h>

#include "common/board_identity.h"
#include "common/flash_store.h"
#include "config/board_feather.h"
#include "control/run_log.h"
#include "drivers/lsm6dsr.h"
#include "hal/console.h"
#include "hal/stm32/clock.h"
#include "hal/stm32/console.h"
#include "hal/stm32/flash.h"
#include "hal/stm32/gpio.h"
#include "hal/stm32/memory.h"
#include "hal/stm32/quad_counter.h"
#include "hal/stm32/spi.h"
#include "hal/stm32/tick.h"
#include "hal/stm32/uid.h"
#include "hal/time.h"

namespace
{
constexpr uint32_t BLINK_HALF_MS = 500;
constexpr uint32_t CORE_MHZ      = hal::stm32::BUSES.hclk_hz / 1000000;

hal::stm32::QuadCounter encoder_left;
hal::stm32::QuadCounter encoder_right;
hal::stm32::SpiBus      imu_spi;
Lsm6dsr                 imu(imu_spi, board::IMU_YAW_SIGN);
volatile bool           imu_ready = false;

/// The bring-up firmware never arms and drives no motors, fan or IR
/// emitters, so nothing can block a save yet. The tickets that add those
/// outputs must force them off here and report any that stay on.
class BringupSaveGuard final : public SaveGuard
{
  public:
    SaveBlocker prepareForSave() override { return saveBlocker({false, false, false, false}); }
};

hal::stm32::StoreFlash store_flash;
BringupSaveGuard       save_guard;
hal::stm32::ChipUid    chip_uid;
BoardIdentity*         identity          = nullptr;
uint8_t                store_led_flashes = 0;

// The always-on 1 kHz tick: widen the encoder counts before anything reads them.
void onTick()
{
    encoder_left.update();
    encoder_right.update();
    if (imu_ready)
        imu.readFifo();
}

char     line[64];
uint32_t line_len = 0;

void printStats()
{
    namespace console                   = hal::console;
    const hal::stm32::tick::Stats ticks = hal::stm32::tick::stats();
    const hal::stm32::MemoryUsage mem   = hal::stm32::memoryUsage();
    std::printf("clock %s, %" PRIu32 " MHz\n",
                hal::stm32::clockSource() == hal::stm32::ClockSource::kHse ? "HSE" : "HSI (fault)",
                CORE_MHZ);
    std::printf("tick count %" PRIu32 " isr last %" PRIu32 " max %" PRIu32
                " cycles (budget %" PRIu32 ")\n",
                ticks.count, ticks.last_cycles, ticks.max_cycles,
                hal::stm32::BUSES.hclk_hz / hal::stm32::tick::TICK_HZ);
    std::printf("tick period min %" PRIu32 " max %" PRIu32 " cycles\n", ticks.min_period_cycles,
                ticks.max_period_cycles);
    std::printf("encoders left %" PRId32 " right %" PRId32 " counts\n", encoder_left.count(),
                encoder_right.count());
    std::printf("imu %s yaw %.2f dps accel z %.3f g, %" PRIu32 " gyro %" PRIu32
                " accel samples, %" PRIu32 " fifo overruns\n",
                imu_ready ? "ok" : "off", static_cast<double>(imu.yawRateDegps()),
                static_cast<double>(imu.accelZG()), imu.gyroSamples(), imu.accelSamples(),
                imu.fifoOverruns());
    std::printf("console ring dropped %" PRIu32 " gate dropped %" PRIu32 " rx idle %" PRIu32
                " overruns %" PRIu32 "\n",
                console::stm32::txRingDroppedBytes(), console::txGateDroppedBytes(),
                console::stm32::rxIdleEvents(), console::stm32::rxOverruns());
    std::printf("heap %" PRIu32 "/%" PRIu32 " stack %" PRIu32 "/%" PRIu32 " bytes\n",
                mem.heap_peak_bytes, mem.heap_limit_bytes, mem.stack_peak_bytes,
                mem.stack_limit_bytes);
}

void flood()
{
    // 64 KB in one go against a 2 KB ring: most of it must be dropped and
    // counted, and the call must return at once rather than wait for the UART.
    static const char chunk[] = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ-\n";
    const uint32_t    before  = hal::console::stm32::txRingDroppedBytes();
    const uint64_t    start   = hal::time::nowUs();
    for (int i = 0; i < 1024; ++i)
        hal::console::write(chunk, sizeof chunk - 1);
    const uint64_t took_us = hal::time::nowUs() - start;
    std::printf("\nflood: %" PRIu32 " bytes dropped, queued in %" PRIu32 " us\n",
                hal::console::stm32::txRingDroppedBytes() - before, static_cast<uint32_t>(took_us));
}

// A ramp in every field, so a dump shows the CCM ring survives a full wrap:
// row i reads pos_mm = i (mod CAPACITY + 7).
void fillRunLog()
{
    RunLog& log = runLog();
    log.start();
    for (int i = 0; i < RunLog::CAPACITY + 7; ++i)
    {
        RunLog::Sample s;
        s.pos_mm   = static_cast<float>(i);
        s.yaw_deg  = static_cast<float>(i % 360 - 180);
        s.fwd_mmps = static_cast<float>(i % 1500);
        s.events   = static_cast<uint16_t>(i & 0x7f);
        log.record(s);
    }
    log.stop();
    std::printf("run log: %u samples, %u bytes in CCM at %p\n", log.count(),
                static_cast<unsigned>(sizeof(RunLog)), static_cast<void*>(&log));
}

// Each sample is unpacked into a stack copy and formatted into a stack row
// (both SRAM) before the console copies it into its DMA ring: DMA cannot
// read CCM. Paced so the 2 KB TX ring never drops a row.
void dumpRunLog()
{
    RunLog& log = runLog();
    log.stop();
    const uint16_t count = log.count();
    const uint16_t dt_ms = log.dtMs();
    std::printf("TRACE n=%u dt_ms=%u cols=%s\n", count, dt_ms, RunLog::COLUMNS);
    for (uint16_t i = 0; i < count; ++i)
    {
        char row[RunLog::ROW_BYTES];
        RunLog::formatRow(row, sizeof row, i, dt_ms, log.sample(i));
        std::printf("%s\n", row);
        hal::time::sleepMs(1); // ~100 bytes take 1 ms at 1 Mbaud
    }
    std::printf("TRACE end\n");
}

void printProfile()
{
    std::printf("board UID %08" PRIX32 "-%08" PRIX32 "-%08" PRIX32 "\n", chip_uid.words[0],
                chip_uid.words[1], chip_uid.words[2]);
    if (identity->identified())
    {
        const board::BoardProfile& p = identity->profile();
        std::printf("profile %s: %s, IR set %u, battery divider %.3f\n", p.name,
                    p.has_fan ? "fan" : "no fan", static_cast<unsigned>(p.ir_set),
                    static_cast<double>(p.battery_divider));
        return;
    }
    std::printf("unknown board: arming refused until a profile is saved. Pick one:\n");
    for (std::size_t i = 0; i < board::PROFILE_PRESET_COUNT; ++i)
        std::printf("  profile %u  %s\n", static_cast<unsigned>(i), board::PROFILE_PRESETS[i].name);
}

void pickProfile(const char* arg)
{
    char*               end   = nullptr;
    const unsigned long index = std::strtoul(arg, &end, 10);
    if (end == arg || *end != '\0')
    {
        std::printf("usage: profile <n>\n");
        return;
    }
    // The erase stalls every interrupt that runs from flash, the 1 kHz tick
    // included, for up to 500 ms; the tick's max period in stats shows it.
    static const char* const STATUS_NAMES[]  = {"saved", "refused", "too large", "flash error"};
    static const char* const BLOCKER_NAMES[] = {"no such preset", "armed", "motors awake", "fan on",
                                                "emitters on"};
    const SaveResult         result          = identity->pick(index);
    if (result.status == SaveStatus::kRefused)
        std::printf("profile not saved: %s\n", BLOCKER_NAMES[static_cast<int>(result.blocker)]);
    else
        std::printf("profile %s (record %" PRIu32 ")\n",
                    STATUS_NAMES[static_cast<int>(result.status)], result.seq);
    if (result.status == SaveStatus::kOk)
        store_led_flashes = 0;
    printProfile();
}

void runLine()
{
    line[line_len] = '\0';
    if (std::strcmp(line, "stats") == 0)
        printStats();
    else if (std::strcmp(line, "flood") == 0)
        flood();
    else if (std::strcmp(line, "profile") == 0)
        printProfile();
    else if (std::strncmp(line, "profile ", 8) == 0)
        pickProfile(line + 8);
    else if (std::strcmp(line, "arm") == 0)
        std::printf(identity->armAllowed() ? "arm: allowed\n"
                                           : "arm: refused, unknown board (see profile)\n");
    else if (std::strcmp(line, "logfill") == 0)
        fillRunLog();
    else if (std::strcmp(line, "log") == 0)
        dumpRunLog();
    else if (line_len > 0)
        std::printf("unknown: %s (try stats, flood, profile, arm, logfill, log)\n", line);
    line_len = 0;
}

/// Reads the UID and loads this board's profile; prints who it is.
void identifyBoard()
{
    chip_uid = hal::stm32::chipUid();
    if (!store_flash.init())
        std::printf("flash store: linker region is not whole equal sectors\n");
    // Built here, once the UID is known: every record carries its hash.
    static FlashStore    flash_store(store_flash, save_guard, SETTINGS_VERSION,
                                     uidHash(chip_uid.words));
    static BoardIdentity board_identity(flash_store);
    identity = &board_identity;

    static const char* const LOAD_NAMES[] = {"ok", "empty", "fell back to the other slot",
                                             "no valid record, defaults"};
    const LoadStatus         status       = identity->boot();
    store_led_flashes                     = ledFlashes(status);
    std::printf("flash store: %s\n", LOAD_NAMES[static_cast<int>(status)]);
    printProfile();
}

// Echoes received bytes and collects them into a line.
void pollConsole()
{
    int c;
    while ((c = hal::console::getcNonBlocking()) >= 0)
    {
        if (c == '\r' || c == '\n')
        {
            hal::console::write("\r\n", 2);
            runLine();
            continue;
        }
        const char ch = static_cast<char>(c);
        hal::console::write(&ch, 1);
        if (line_len < sizeof line - 1)
            line[line_len++] = ch;
    }
}
} // namespace

// Read with `print memory_usage` and `print tick_stats` in gdb.
volatile hal::stm32::MemoryUsage memory_usage;
volatile hal::stm32::tick::Stats tick_stats;

int main()
{
    const hal::stm32::ClockSource clock = hal::stm32::setupClocks();
    hal::stm32::gpioOutput(board::LED);
    encoder_left.init(board::ENCODER_PINS[0], board::ENCODER_PINS[1]);
    encoder_right.init(board::ENCODER_PINS[2], board::ENCODER_PINS[3]);
    hal::stm32::tick::start(onTick);
    hal::console::stm32::init();
    std::printf("\nJurababa STM32 bring-up, %" PRIu32 " MHz from the %s\n", CORE_MHZ,
                clock == hal::stm32::ClockSource::kHse ? "crystal" : "HSI (crystal fault)");

    imu_spi.init(board::IMU_SPI);
    hal::time::sleepMs(20); // LSM6DSR boot time is 10 ms from power-up
    const uint8_t            who            = imu.whoAmI();
    const Lsm6dsr::Status    status         = imu.init();
    static const char* const STATUS_NAMES[] = {"ok", "wrong WHO_AM_I", "reset timeout"};
    std::printf("LSM6DSR WHO_AM_I 0x%02X (expect 0x%02X): %s\n", who, lsm6dsr::WHO_AM_I_VALUE,
                STATUS_NAMES[static_cast<int>(status)]);
    imu_ready = status == Lsm6dsr::Status::kOk;

    identifyBoard();

    const uint32_t led_port       = hal::stm32::gpioPort(board::LED);
    const uint16_t led_bit        = hal::stm32::gpioBit(board::LED);
    uint32_t       last_report_ms = 0;
    while (true)
    {
        __asm__ volatile("wfi");
        pollConsole();
        const uint32_t now_ms = hal::time::nowMs();
        bool           led_on = (now_ms / BLINK_HALF_MS) % 2 == 0;
        if (clock != hal::stm32::ClockSource::kHse)
            led_on = hal::stm32::clockFaultLedOn(now_ms);
        else if (store_led_flashes > 0)
            led_on = ledCodeOn(now_ms, store_led_flashes);
        if (led_on)
            gpio_set(led_port, led_bit);
        else
            gpio_clear(led_port, led_bit);

        if (now_ms - last_report_ms >= BLINK_HALF_MS)
        {
            last_report_ms += BLINK_HALF_MS;
            const hal::stm32::MemoryUsage usage = hal::stm32::memoryUsage();
            memory_usage.heap_peak_bytes        = usage.heap_peak_bytes;
            memory_usage.heap_limit_bytes       = usage.heap_limit_bytes;
            memory_usage.stack_peak_bytes       = usage.stack_peak_bytes;
            memory_usage.stack_limit_bytes      = usage.stack_limit_bytes;

            const hal::stm32::tick::Stats ticks = hal::stm32::tick::stats();
            tick_stats.count                    = ticks.count;
            tick_stats.last_cycles              = ticks.last_cycles;
            tick_stats.max_cycles               = ticks.max_cycles;
            tick_stats.min_period_cycles        = ticks.min_period_cycles;
            tick_stats.max_period_cycles        = ticks.max_period_cycles;
        }
    }
}
