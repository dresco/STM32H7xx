Import("env")

from pathlib import Path


# -----------------------------------------------------------------------------
# Voron Cascade / BTT Scylla factory-default overlay
# -----------------------------------------------------------------------------
#
# grblHAL normally keeps runtime settings in NVS. These defaults are only used
# when NVS is initialized/restored (for example after $RST=* or an incompatible
# settings-version change). Keeping the machine-critical values here prevents a
# factory reset from falling back to generic grblHAL / GS20 VFD defaults.
#
# WiFi SSIDs/passwords are intentionally NOT embedded in this public firmware.
# -----------------------------------------------------------------------------

SCYLLA_BASE = "btt_scylla_h723_tmc5160"
SCYLLA_FULL = "btt_scylla_h723_tmc5160_wifi_full"
SCYLLA_ENVS = {SCYLLA_BASE, SCYLLA_FULL}

pioenv = env.subst("$PIOENV")
project_dir = Path(env.subst("$PROJECT_DIR"))


def inject_after(path, anchor, marker, block):
    """Inject a build-time source overlay once, failing loudly if upstream moved."""
    path = Path(path)
    text = path.read_text(encoding="utf-8")

    if marker in text:
        return

    if anchor not in text:
        raise RuntimeError(
            f"Scylla factory-default overlay failed: anchor not found in {path}: {anchor}"
        )

    text = text.replace(anchor, anchor + "\n\n" + block.strip() + "\n", 1)
    path.write_text(text, encoding="utf-8")


def replace_exact(path, old, new):
    """Replace one known upstream default, idempotently."""
    path = Path(path)
    text = path.read_text(encoding="utf-8")

    if new in text:
        return

    if old not in text:
        raise RuntimeError(
            f"Scylla factory-default overlay failed: expected text not found in {path}: {old}"
        )

    text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")


if pioenv in SCYLLA_ENVS:
    # Core grblHAL and Trinamic defaults. These match the known-good
    # Voron Cascade firmware-settings backup from 2026-09-09.
    core_defaults = r"""
// SCYLLA_VORON_FACTORY_DEFAULTS
// Machine profile: Voron Cascade / BTT Scylla H723
// Secrets and work offsets are intentionally excluded.

#define DEFAULT_STEP_PULSE_MICROSECONDS          5.0f
#define DEFAULT_STEPPER_IDLE_LOCK_TIME            255
#define DEFAULT_STEP_SIGNALS_INVERT_MASK          0
#define DEFAULT_DIR_SIGNALS_INVERT_MASK           3
#define DEFAULT_ENABLE_SIGNALS_INVERT_MASK        7
#define DEFAULT_STEPPER_DEENERGIZE_MASK           7

#define DEFAULT_LIMIT_SIGNALS_INVERT_MASK         7
#define DEFAULT_LIMIT_SIGNALS_PULLUP_DISABLE_MASK 0
#define DEFAULT_PROBE_SIGNAL_INVERT               1
#define DEFAULT_CONTROL_SIGNALS_INVERT_MASK       0
#define DEFAULT_DISABLE_CONTROL_PINS_PULL_UP_MASK 0

#define DEFAULT_JUNCTION_DEVIATION                0.010f
#define DEFAULT_ARC_TOLERANCE                     0.002f
#define DEFAULT_REPORT_INCHES                     0

#define DEFAULT_SOFT_LIMIT_ENABLE                 0
#define DEFAULT_HARD_LIMIT_ENABLE                 1
#define DEFAULT_JOG_LIMIT_ENABLE                  1

// $22 = 1029: homing enabled + init lock + startup scripts only after homing.
#define DEFAULT_HOMING_ENABLE                     1
#define DEFAULT_HOMING_INIT_LOCK                  1
#define DEFAULT_HOMING_SINGLE_AXIS_COMMANDS       0
#define DEFAULT_HOMING_FORCE_SET_ORIGIN           0
#define DEFAULT_HOMING_ALLOW_MANUAL               0
#define DEFAULT_HOMING_OVERRIDE_LOCKS             0
#define DEFAULT_HOMING_KEEP_STATUS_ON_RESET       0
#define DEFAULT_HOMING_USE_LIMIT_SWITCHES         0
#define DEFAULT_RUN_STARTUP_SCRIPTS_ONLY_ON_HOMED 1
#define DEFAULT_HOMING_DIR_MASK                   2
#define DEFAULT_HOMING_FEED_RATE                  100.0f
#define DEFAULT_HOMING_SEEK_RATE                  1000.0f
#define DEFAULT_HOMING_DEBOUNCE_DELAY             250
#define DEFAULT_HOMING_PULLOFF                    5.0f
#define DEFAULT_N_HOMING_LOCATE_CYCLE             2
#define DEFAULT_HOMING_CYCLE_0                    4
#define DEFAULT_HOMING_CYCLE_1                    3
#define DEFAULT_HOMING_CYCLE_2                    0

#define DEFAULT_PARKING_ENABLE                    1
#define DEFAULT_PARKING_AXIS                      2
#define DEFAULT_PARKING_PULLOUT_INCREMENT         5.0f
#define DEFAULT_PARKING_PULLOUT_RATE              100.0f
#define DEFAULT_PARKING_TARGET                    -5.0f
#define DEFAULT_PARKING_RATE                      250.0f

#define DEFAULT_RESET_OVERRIDES                   1
#define DEFAULT_SLEEP_ENABLE                      0
#define DEFAULT_RESTORE_AFTER_FEED_HOLD           1
#define DEFAULT_FORCE_INITIALIZATION_ALARM        1
#define DEFAULT_LEGACY_RTCOMMANDS                 1

#define DEFAULT_SPINDLE_RPM_MAX                   24000.0f
#define DEFAULT_SPINDLE_RPM_MIN                   600.0f

#define DEFAULT_X_STEPS_PER_MM                    853.33294f
#define DEFAULT_Y_STEPS_PER_MM                    853.33294f
#define DEFAULT_Z_STEPS_PER_MM                    853.33294f
#define DEFAULT_X_MAX_RATE                        6000.0f
#define DEFAULT_Y_MAX_RATE                        6000.0f
#define DEFAULT_Z_MAX_RATE                        3000.0f
#define DEFAULT_X_ACCELERATION                    500.0f
#define DEFAULT_Y_ACCELERATION                    400.0f
#define DEFAULT_Z_ACCELERATION                    250.0f
#define DEFAULT_X_MAX_TRAVEL                      260.0f
#define DEFAULT_Y_MAX_TRAVEL                      220.0f
#define DEFAULT_Z_MAX_TRAVEL                      100.0f

// INVT/RS485 transport: 19200, 8E1.
#define DEFAULT_MODBUS_STREAM_BAUD                3
#define DEFAULT_MODBUS_STREAM_DATA_BITS           0
#define DEFAULT_MODBUS_STREAM_STOP_BITS           0
#define DEFAULT_MODBUS_STREAM_PARITY              1

// TMC2160 hardware is driven through the grblHAL TMC5160 compatibility profile.
#define TRINAMIC_DEFAULT_MICROSTEPS               16
#define TMC_X_CURRENT                             3500
#define TMC_Y_CURRENT                             3500
#define TMC_Z_CURRENT                             3500
#define TMC_X_HOLD_CURRENT_PCT                    50
#define TMC_Y_HOLD_CURRENT_PCT                    50
#define TMC_Z_HOLD_CURRENT_PCT                    60

// Current EventOut trigger defaults: event 1 = spindle, event 2 = motion.
#define EVENTOUT_1_ACTION                         1
#define EVENTOUT_2_ACTION                         8
"""

    inject_after(
        project_dir / "grbl" / "config.h",
        "#define _GRBL_CONFIG_H_",
        "SCYLLA_VORON_FACTORY_DEFAULTS",
        core_defaults,
    )


if pioenv == SCYLLA_FULL:
    # MODVFD upstream defaults target a GS20. Override the restore values with
    # the known-good INVT mapping used by this machine.
    vfd_source = project_dir / "spindle" / "vfd" / "spindle.c"
    replace_exact(
        vfd_source,
        "vfd_config.get_freq_reg = 8451; //0x2103",
        "vfd_config.get_freq_reg = 12288; //0x3000 - INVT output frequency",
    )
    replace_exact(
        vfd_source,
        "vfd_config.run_cw_cmd = 18; //0x12",
        "vfd_config.run_cw_cmd = 1; // INVT forward",
    )
    replace_exact(
        vfd_source,
        "vfd_config.run_ccw_cmd = 34; //0x22",
        "vfd_config.run_ccw_cmd = 2; // INVT reverse",
    )
    replace_exact(
        vfd_source,
        "vfd_config.stop_cmd = 1; //0x02",
        "vfd_config.stop_cmd = 5; // INVT stop",
    )
    replace_exact(
        vfd_source,
        "vfd_config.in_multiplier = 50;",
        "vfd_config.in_multiplier = 100; // INVT RPM -> 0.01 Hz scaling",
    )

    # The upstream homing-pulloff plugin currently hardcodes X to 12 mm.
    # Make all three machine axes restore to the verified 5 mm value.
    homing_source = project_dir / "plugins" / "homing_pulloff.c"
    replace_exact(
        homing_source,
        "#define DEFAULT_X_HOMING_PULLOFF 12.0f",
        "#define DEFAULT_X_HOMING_PULLOFF 5.0f // Scylla/Voron factory profile\n"
        "#define DEFAULT_Y_HOMING_PULLOFF 5.0f\n"
        "#define DEFAULT_Z_HOMING_PULLOFF 5.0f",
    )


# Keep the existing Intel HEX generation used by the Scylla workflow.
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(
        '"$OBJCOPY" -O ihex "$BUILD_DIR/${PROGNAME}.elf" "$BUILD_DIR/${PROGNAME}.hex"',
        "Building $BUILD_DIR/${PROGNAME}.hex",
    ),
)
