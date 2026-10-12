#pragma once

#include <cstdint>

/** Atlas application feature switches. */
constexpr bool kEnableUi = false;
#define HAS_REFEREE

/**
 * Chassis motion-frame calibration in degrees.
 *
 * Atlas application commands use forward/right axes.  The common chassis
 * library SetSpeed() API uses x=right and y=forward.  chassis_task.cpp always
 * performs that fixed axis adaptation first, then applies this additional
 * physical installation offset.
 *
 *   0.0f    : physical chassis front matches the configured front
 *   90.0f   : rotate the calibrated chassis frame by +90 degrees
 *  -90.0f   : rotate the calibrated chassis frame by -90 degrees
 *  180.0f   : reverse both translation axes
 *
 * Tune only while the robot is lifted or in KILL.  This is unrelated to
 * control::chassis_t::offset, which changes turn-center distribution.
 */
constexpr float kDefaultChassisFrameOffsetDeg = 90.0f;


/**
 * No-referee bench power mode.
 *
 * This is a compile-time TEST switch, not a competition fallback.  The
 * delivered package keeps it disabled.  Enable it only for an explicit bench
 * build with no referee system:
 *
 *   -DATLAS_ENABLE_NO_REFEREE_POWER_TEST=1
 *
 * When disabled, Atlas never invents 100 W / 60 J while the referee is
 * offline.  It waits for valid referee data and otherwise uses the conservative
 * 45 W / 0 J fail-safe below.
 */
#ifndef ATLAS_ENABLE_NO_REFEREE_POWER_TEST
#define ATLAS_ENABLE_NO_REFEREE_POWER_TEST 0
#endif

static_assert(ATLAS_ENABLE_NO_REFEREE_POWER_TEST == 0 ||
                  ATLAS_ENABLE_NO_REFEREE_POWER_TEST == 1,
              "ATLAS_ENABLE_NO_REFEREE_POWER_TEST must be 0 or 1");

#if ATLAS_ENABLE_NO_REFEREE_POWER_TEST
constexpr uint8_t kLocalChassisPowerLimitW = 100U;
constexpr float kLocalChassisBufferJoule = 60.0f;
#endif

/**
 * Referee link is online but its limit/buffer payload is not usable.  Atlas is
 * an infantry robot, so 45 W is the lowest 2026 infantry chassis limit.
 * Zero buffer forbids relying on temporary excess power until valid data returns.
 */
constexpr uint8_t kInvalidRefereePowerLimitW = 45U;
constexpr float kInvalidRefereeBufferJoule = 0.0f;
constexpr float kMaxValidRefereePowerW = 200.0f;
