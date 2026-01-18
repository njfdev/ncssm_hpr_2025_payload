# Pico Logger Troubleshooting

## Common Issues

### Firmware Flashes But Device Stays in BOOTSEL Mode

**Symptoms**: After `cargo run --release`, picotool reports success but device remains as USB mass storage.

**Cause**: Wrong chip configuration. RP2040 and RP2350 have different:
- Cargo features (`rp2040` vs `rp235xa`)
- Targets (`thumbv6m-none-eabi` vs `thumbv8m.main-none-eabihf`)
- Memory layouts (BOOT2 section vs start_block/end_block)

**Solution**:
1. Check `.cargo/config.toml` has `target = "thumbv6m-none-eabi"`
2. Check `Cargo.toml` has `embassy-rp = { ..., features = ["rp2040", ...] }`
3. Check `memory.x` has BOOT2 section at 0x10000000

### Build Error: `target may not be installed`

```
error[E0463]: can't find crate for `core`
  = note: the `thumbv6m-none-eabi` target may not be installed
```

**Solution**:
```bash
rustup target add thumbv6m-none-eabi
```

### Build Error: `compare_exchange requires atomic CAS`

```
error[E0277]: `compare_exchange` requires atomic CAS but not available on this target
```

**Cause**: RP2040 (Cortex-M0+) lacks hardware atomic CAS. The `static_cell` crate needs `portable-atomic` with critical-section support.

**Solution**: Add to Cargo.toml:
```toml
portable-atomic = { version = "1", features = ["critical-section"] }
```

### Build Error: `undefined symbol: __bi_entries_start`

**Cause**: Missing binary info section in memory.x linker script.

**Solution**: Add to memory.x:
```
SECTIONS {
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;
```

### USB Serial Device Not Appearing

**Symptoms**: Device flashed successfully, not in BOOTSEL, but no `/dev/cu.usbmodem*` appears.

**Possible causes**:
1. USB driver task not spawned - ensure `spawner.spawn(usb_task(usb)).unwrap()`
2. Missing USB interrupt binding - use `bind_interrupts!` macro
3. Panic during initialization - try adding LED blink before USB init to verify execution

### pyserial Import Error

```
AttributeError: module 'serial' has no attribute 'Serial'
```

**Cause**: Wrong `serial` package installed (not pyserial).

**Solution**:
```bash
pip3 uninstall serial
pip3 install pyserial
```

## Identifying the Chip

Before starting work, verify the chip variant:

```bash
# In BOOTSEL mode
picotool info

# Look for:
# type: RP2040  -> Use thumbv6m-none-eabi, rp2040 feature
# type: RP2350  -> Use thumbv8m.main-none-eabihf, rp235xa feature
```

## RP2040 vs RP2350 Quick Reference

| Aspect | RP2040 | RP2350 |
|--------|--------|--------|
| Core | Cortex-M0+ (ARMv6-M) | Cortex-M33 (ARMv8-M) |
| Target | `thumbv6m-none-eabi` | `thumbv8m.main-none-eabihf` |
| Feature | `rp2040` | `rp235xa` |
| RAM | 264KB | 520KB |
| Boot | BOOT2 section (256B) | start_block/end_block |
| Atomics | No HW CAS (need portable-atomic) | Full atomic support |
