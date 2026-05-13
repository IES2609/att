/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _ENVIRONMENTAL_H_
#define _ENVIRONMENTAL_H_

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SAMPLES_PER_BATCH 10  /* Publish every x samples to batch and reduce messaging load */


/* Environmental data flows via dedicated message queue (environmental_msgq.h) 
 * with no zbus pub/sub. Channel below is only for storage_data compatibility.
 */
ZBUS_CHAN_DECLARE(
	environmental_chan
);

enum environmental_msg_type {
	/* Output message types */

	/* Response message to a request for current environmental sensor values.
	 * The sampled values are found in the respective fields of the message structure.
	 */
	ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE = 0x1,

	/* Input message types */

	/* Request to sample the current environmental sensor values.
	 * The response is sent as a ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE message.
	 */
	ENVIRONMENTAL_SENSOR_SAMPLE_REQUEST,
};

/* Fixed-point scaling: store float as int by multiplying by scale factor */
#define ACCEL_SCALE 1000        /* ±16g range at 0.001 g/LSB */
#define GYRO_SCALE 100          /* ±2000 dps range at 0.01 dps/LSB */
#define PRESSURE_SCALE 1        /* Pa as int32_t */

struct environmental_msg {
	enum environmental_msg_type type;

	/** Number of samples currently in this batch message (0 to SAMPLES_PER_BATCH) */
	uint8_t sample_count;

	/** Latest pressure in Pa (updated at 0.1 Hz, sparse).
	 *  Store once per batch instead of per-sample to save RAM.
	 */
	int32_t pressure;

	/** Timestamp of first sample in batch (milliseconds: unix time or uptime) */
	uint32_t batch_timestamp_ms;

	/** Per-sample time deltas from batch_timestamp_ms in milliseconds (assuming ~10 Hz = ~100 ms apart) */
	uint16_t timestamp_delta_ms[SAMPLES_PER_BATCH];

	/** Contains the current acceleration values in fixed-point (value * 1000 = g).
	 *  Fixed-point int16_t: range ±32.767g, resolution 0.001g
	 */
	int16_t accel_hp[3][SAMPLES_PER_BATCH];

	/** Contains the current gyroscope values in fixed-point (value * 100 = dps).
	 *  Fixed-point int16_t: range ±327.67 dps, resolution 0.01 dps
	 */
	int16_t gyro_hp[3][SAMPLES_PER_BATCH];

	/** Contains the current low-power acceleration values in fixed-point (value * 1000 = g).
	 *  Fixed-point int16_t: range ±32.767g, resolution 0.001g
	 */
	int16_t accel_lp[3][SAMPLES_PER_BATCH];
};

#define MSG_TO_ENVIRONMENTAL_MSG(_msg)	(*(const struct environmental_msg *)_msg)

/**
 * @brief Print environmental sensor data from storage to terminal in CSV format
 * 
 * Reads the entire environmental_stream.bin file and outputs sensor data
 * as CSV with columns: timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
 * pressure, accel_lp_x, accel_lp_y, accel_lp_z
 * 
 * Values are converted from fixed-point format to float for readability.
 * Call this function on demand (e.g., long button press) to debug/export data.
 */
void environmental_stream_print_to_terminal(void);

/**
 * @brief Send yellow LED indicator for file deletion in progress
 */
void environmental_led_yellow_blinking(void);

/**
 * @brief Send green LED indicator for storage full state
 */
void environmental_led_green_full(void);

/**
 * @brief Pause environmental sampling (stop collecting data without restarting).
 *
 * Stops sensor triggers, cancels pending work, and clears any partial batch.
 * Used when user requests to view/export data without restarting.
 */
void environmental_sampling_pause(void);

/**
 * @brief Pause environmental sampling and restart it after a delay.
 *
 * This is used after file deletion to enforce a pre-sampling wait window
 * before sampling resumes.
 *
 * @param delay_sec Delay in seconds before sampling is restarted.
 */
void environmental_sampling_restart_with_delay(uint32_t delay_sec);


#ifdef __cplusplus
}
#endif

#endif /* _ENVIRONMENTAL_H_ */
