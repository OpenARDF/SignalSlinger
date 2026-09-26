#!/usr/bin/env python3
"""Protect RF calibration persistence and serial-command ordering contracts."""
from pathlib import Path

root = Path(__file__).resolve().parent.parent
defs = (root / "SignalSlinger/defs.h").read_text()
header = (root / "SignalSlinger/include/eeprommanager.h").read_text()
si5351 = (root / "SignalSlinger/include/si5351.h").read_text()
storage = (root / "SignalSlinger/src/eeprommanager.cpp").read_text()
main = (root / "SignalSlinger/main.cpp").read_text()
transmitter = (root / "SignalSlinger/src/transmitter.cpp").read_text()

assert "#define EEPROM_INITIALIZED_FLAG_V0135 (uint16_t)0x0135" in defs
assert "#define EEPROM_INITIALIZED_FLAG (uint16_t)0x0136" in defs
assert "uint16_t clock_calibration;\n\tint32_t si5351_correction_ppb;\n\tuint8_t days_to_run;" in header
assert "Reserved_31" not in header
assert "static_assert(sizeof(EE_prom) == 288" in storage

migration_start = storage.index("if(initialization_flag == EEPROM_INITIALIZED_FLAG_V0135)")
migration_end = storage.index("return false;", migration_start)
migration = storage[migration_start:migration_end]
correction_write = migration.index("avr_eeprom_write_dword(Si5351_Correction")
marker_write = migration.index("avr_eeprom_write_word(Eeprom_initialization_flag")
assert correction_write < marker_write
assert "migrateEEPROMEventSetting" not in migration

assert "*/\n#define APPLY_XTAL_CALIBRATION_VALUE\n#define SUPPORT_FOUT_BELOW_1024KHZ" in si5351
assert "#define SUPPORT_FOUT_BELOW_1024KHZ" in si5351[si5351.index("*/") :]
assert "#define DO_BOUNDS_CHECKING" in si5351[si5351.index("*/") :]

handler_start = main.index("static bool handleRfCalibrationCommand")
handler_end = main.index("/**\n * Consume and handle", handler_start)
handler = main[handler_start:handler_end]
cf_start = handler.index('if(strcmp(selector, "CF")')
cf_end = handler.index('if(strcmp(selector, "C")', cf_start)
cf_handler = handler[cf_start:cf_end]
assert cf_handler.index("eventRunning()") < cf_handler.index("txStartCalibrationCarrier()")
assert cf_handler.index("txSetCalibrationFrequency(frequency_hz)") < cf_handler.index(
    "txStartCalibrationCarrier()"
)
assert "previous_frequency_hz" in cf_handler
assert "txSetCalibrationFrequency(previous_frequency_hz)" in cf_handler
assert "updateEEPROMVar" not in cf_handler
assert handler.index("eventRunning()") < handler.index("txApplyCalibrationOffsetHz(offset_hz)")
assert handler.index("txApplyCalibrationOffsetHz(offset_hz)") < handler.index(
    "updateEEPROMVar(Si5351_Correction"
)

transaction_start = transmitter.index("static bool applyCalibrationCorrectionPpb")
apply_start = transmitter.index("bool txApplyCalibrationOffsetHz")
exit_start = transmitter.index("bool txExitCalibrationCarrier", apply_start)
transaction = transmitter[transaction_start:apply_start]
apply = transmitter[apply_start:exit_start]
exit_calibration = transmitter[exit_start:transmitter.index("/**\n * Globally enable", exit_start)]
assert transaction.index("powerToTransmitter(true)") < transaction.index("si5351_set_freq(g_calibration_frequency_hz")
assert transaction.index("si5351_set_freq(g_calibration_frequency_hz") < transaction.index("keyTransmitter(true)")
assert "g_calibration_restore_initialized" in transaction
assert "g_calibration_restore_keyed" in transaction
assert "return applyCalibrationCorrectionPpb(requested_correction_ppb)" in apply
assert "txStartCalibrationCarrier" in transaction
assert "applyCalibrationCorrectionPpb(si5351_get_correction())" in transaction
assert "powerToTransmitter(false)" in exit_calibration

button_edge_start = main.index("if(holdSwitch) /* Switch was open")
button_edge_end = main.index("else /* Switch is now open */", button_edge_start)
button_edge = main[button_edge_start:button_edge_end]
assert button_edge.index("txCalibrationCarrierActive()") < button_edge.index(
    "g_foreground_exit_calibration = true"
)
assert "g_consume_current_press_for_calibration_exit = true" in button_edge
button_exit_start = main.index("if(g_foreground_exit_calibration)")
button_exit_end = main.index("uint16_t counted_presses", button_exit_start)
button_exit = main[button_exit_start:button_exit_end]
assert "txExitCalibrationCarrier()" in button_exit
assert "g_foreground_handle_counted_presses" in button_exit
assert "g_long_button_press = false" in button_exit

clone_start = main.index("void handleSerialCloning(void)", handler_end)
assert "Si5351_Correction" not in main[clone_start:]

print("RF calibration contract: layout, migration, Si5351 support, safety and persistence ordering passed.")
