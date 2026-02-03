# SD Logging Architecture

This document describes the planned binary SD logging system for Alphaflight. The goal is a compact, deterministic, low-overhead flight data recorder optimized for STM32H7 + SDMMC + IDMA.

This is *not* a filesystem. The SD card is treated as a linear byte stream.

---

## Core Ideas

* Linear append-only byte stream
* Typed binary frames (no self-describing structs)
* Delta-by-default, absolute on overflow
* Batch timestamps (never per-packet)
* One master clock (gyro)
* Multiple logging tiers derived by integer division
* User-assignable packet types per tier

The format is designed to be:

* deterministic
* bandwidth efficient
* power-loss tolerant
* simple to decode on PC

---

## Master Clock

The gyro runs at the highest frequency (e.g. 800 Hz) and defines the master tick.

All logging tiers are derived from this base rate via integer dividers (preferably powers of two):

Example:

* Tier 0: 800 Hz
* Tier 1: 400 Hz
* Tier 2: 200 Hz
* Tier 3: 100 Hz
* Tier 4: 50 Hz
* Tier 5: 12.5 Hz

Each tier owns its own counter and timestamp accumulator.

Firmware uses decrementing counters (not modulo) to trigger tiers:

* Each tier has `counter` and `reload`
* On each gyro tick: `if(--counter == 0) { counter = reload; fire tier }`

Multiple tiers may fire on the same tick; they are always emitted in fixed order (Tier0 → TierN).

---

## Batching + Timestamps

Each tier produces a *batch*:

1. Emit a timestamp delta frame for that tier
2. Emit all packets assigned to that tier

All packets following a timestamp implicitly share that time until the next timestamp of the same tier.

Timestamps are never stored per packet.

Timestamp frames exist in two variants:

* Absolute timestamp (used on startup / resync)
* Delta timestamp (normal operation)

Overflow automatically falls back to absolute.

Each tier maintains its own timestamp state.

---

## Packet Format

The log is a continuous stream of frames:

```
[FrameID][Payload]
[FrameID][Payload]
...
```

FrameID is normally 8-bit.

An escape ID allows optional 16-bit extended IDs if ever needed.

Each semantic packet has two frame types:

* Absolute
* Delta

Example:

* Gyro absolute
* Gyro delta
* Accel absolute
* Accel delta

Decoder maintains state per packet type.

---

## Delta Encoding

Sensor data is stored as fixed-point integers (never floats):

* gyro / accel: int16
* GPS: scaled int32
* baro: fixed-point
* PID: scaled int16

Delta frames use smaller types (typically int8).

Workflow:

* Try delta
* If any field overflows → emit absolute instead

This guarantees bounded bandwidth while remaining robust during aggressive motion.

---

## Logging Tiers (User Configurable)

Firmware supports N logging tiers (e.g. 10).

Each tier has:

* divider (relative to gyro)
* timestamp accumulator
* packet assignment mask

GUI presents these tiers as frequencies (derived from divider):

Example:

* Tier 0: 800 Hz
* Tier 1: 400 Hz
* Tier 2: 200 Hz
* ...
* Tier 9: ~0.4 Hz

User assigns each packet type (gyro, PID, GPS, etc.) to exactly one tier.

This allows flexible bandwidth vs fidelity tradeoffs without changing firmware.

---

## Control Loop vs Telemetry

High-rate control-loop data (gyro, attitude, PID) is phase-aligned and logged synchronously.

Peripheral telemetry (GPS, CRSF, baro, flags) is allowed to lag and is logged in slower tiers.

Precise timing is preserved via batch timestamps.

Small GPS or CRSF jitter is considered acceptable in exchange for bandwidth reduction.

---

## Sync / Recovery

A periodic sync frame is emitted:

* magic bytes
* absolute timestamp

Used by PC decoder to resynchronize after power loss or corrupted blocks.

No padding or block alignment is used.

Frames may cross SD block boundaries freely.

---

## SD Writing

* Data is accumulated in RAM buffers
* Flushed in large chunks via SDMMC + IDMA
* No awareness of 512-byte block boundaries at protocol level

SD is treated as a raw sequential device.

---

## Decoder Model (PC Side)

Decoder is a state machine:

* Parse frames
* Maintain per-packet state
* Apply deltas
* Apply tier timestamps
* Reconstruct absolute timeline

No filesystem metadata is required.

---

## Summary

This system implements a small multi-rate flight data recorder:

* master gyro clock
* tiered batch logging
* delta encoding
* typed frames
* timestamp scoping

The design prioritizes determinism, simplicity, and bandwidth efficiency over generic compression or filesystem compatibility.

