# the-cube

Monorepo for the full self-balancing reaction-wheel project.

## Structure

- `firmware/esp32-s3/` contains the ESP32-S3 firmware built with ESP-IDF.
- `control/simulation/` contains control-system experiments, math models, and tuning work.
- `control/telemetry/` contains scripts and tools for log parsing and telemetry analysis.
- `hardware/` contains wiring notes, pin maps, datasheets, and hardware-specific references.
- `docs/` contains system-level documentation, decisions, and bring-up notes.

## Recommended workflow

1. Create the ESP-IDF project inside `firmware/esp32-s3/`.
2. Keep motor tests, PWM bring-up, and embedded code inside the firmware folder only.
3. Keep balancing logic experiments and plots in `control/`.
4. Document wiring, pin assignments, and verified motor behavior in `hardware/` and `docs/`.

## First milestones

1. Bring up an ESP32-S3 project and flash a known-good example.
2. Generate stable 20 kHz PWM on the selected GPIO pins.
3. Verify one motor at a time with safe current limits.
4. Add telemetry and basic control experiments before attempting self-balancing.
