/**
 * @file version.h
 * @brief KSC TCU Firmware and Hardware Version Definitions
 *
 * Update FW_VERSION_* fields before tagging a release commit.
 * Both firmware and hardware version strings are published to
 * ThingsBoard as server attributes on every boot.
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef VERSION_H_
#define VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * FIRMWARE VERSION
 * ========================================================================= */

#define FW_VERSION_MAJOR     1
#define FW_VERSION_MINOR     1
#define FW_VERSION_PATCH     0

/** Stringified firmware version for JSON payload and log output */
#define FW_VERSION_STR       "1.1.0"

/** Build timestamps injected by the compiler preprocessor */
#define FW_BUILD_DATE        __DATE__
#define FW_BUILD_TIME        __TIME__

/* =========================================================================
 * HARDWARE VERSION
 * ========================================================================= */

/**
 * Hardware PCB revision — matches BOARD_HW_REVISION in board_config.h.
 * Kept here as well so version.h is the single file to check for identity.
 */
#define HW_VERSION_STR       "V1.0.0"

#ifdef __cplusplus
}
#endif

#endif /* VERSION_H_ */
