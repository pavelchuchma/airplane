/*
 * Diagnostic lines: the '#' lines of the air USB serial, also queued for the
 * radio as LogMessage (doc/plans/2026-10-20-air-diag-log.md). USB and the
 * flight battery are never connected together, so without the radio copy
 * every diagnostic of a battery-only problem would be lost.
 *
 * print() is safe from any task (sensorTask on core 0, loop() on core 1);
 * only loop() sends. A full buffer drops the oldest line and counts it in
 * LogMessageHeader.dropped.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace diag {

// Prints "# <text>" on USB serial and queues the text (truncated to
// LOG_TEXT_MAX) for the radio.
void print(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Builds the LogMessage for the oldest queued line into out (room for
// sizeof(LogMessageHeader) + LOG_TEXT_MAX) and returns its length, 0 = none
// queued. The line stays queued until commit(seq) - a failed send loses
// nothing.
size_t peek(uint8_t *out, uint16_t &seq);

// Removes the line peek() returned, unless overflow already dropped it.
void commit(uint16_t seq);

}  // namespace diag
