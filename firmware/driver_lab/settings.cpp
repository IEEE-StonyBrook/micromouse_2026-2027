#include "driver_lab/settings.h"

#include "hal/console.h"

void DriverLabSettings::print() const
{
    hal::console::printFormat("\n");
    hal::console::printFormat("=== Settings ===\n");

    // Motor model
    hal::console::printFormat("kM   = %8.2f  mm/s/V\n", kM);
    hal::console::printFormat("Tm   = %8.5f  s\n", tm);
    hal::console::printFormat("kS   = %8.4f  V\n", kS);

    // Per-motor (only show if any per-motor field diverges from combined)
    if (kM_L != kM || kM_R != kM || kS_L != kS || kS_R != kS || kA_L != kA || kA_R != kA)
    {
        hal::console::printFormat("  L: kM=%.1f kS=%.4f kA=%.7f  R: kM=%.1f kS=%.4f kA=%.7f\n",
                                  kM_L, kS_L, kA_L, kM_R, kS_R, kA_R);
    }

    // Feedforward
    hal::console::printFormat("kV   = %.7f  V/(mm/s)\n", kV);
    hal::console::printFormat("kA   = %.7f  V/(mm/s^2)\n", kA);

    // Forward PD
    hal::console::printFormat("zeta = %8.4f\n", zeta);
    hal::console::printFormat("Td   = %8.5f  s\n", td);
    hal::console::printFormat("kP   = %8.5f\n", kP);
    hal::console::printFormat("kD   = %8.5f\n", kD);

    // Rotation plant + PD
    hal::console::printFormat("rot_kM = %8.2f  deg/s/V\n", rot_kM);
    hal::console::printFormat("rot_tm = %8.5f  s\n", rot_tm);
    hal::console::printFormat("rot_zeta = %.4f\n", rot_zeta);
    hal::console::printFormat("rot_td   = %.5f  s\n", rot_td);
    hal::console::printFormat("turnKP = %.4f\n", turnKP);
    hal::console::printFormat("turnKD = %.4f\n", turnKD);
    hal::console::printFormat("\n");
}
