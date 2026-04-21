/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <net/nrf_cloud_coap.h>

#include "cloud_environmental.h"

LOG_MODULE_DECLARE(cloud, CONFIG_APP_CLOUD_LOG_LEVEL);


int cloud_environmental_send(const struct environmental_msg *env,
                             int64_t timestamp_ms,
                             bool confirmable)
{
    int err;

    if (!env || env->sample_count == 0) {
        return 0;
    }

    if (env->pressure != 0) {
        err = nrf_cloud_coap_sensor_send(NRF_CLOUD_JSON_APPID_VAL_AIR_PRESS,
                                         (double)env->pressure,
                                         timestamp_ms,
                                         false);
        if (err) {
            LOG_ERR("Failed to send pressure: %d", err);
        }
    }

    double accel_x = (double)env->accel_hp[0][0] / ACCEL_SCALE;
    err = nrf_cloud_coap_sensor_send("ACCEL_X", 
                                     accel_x, 
                                     timestamp_ms, 
                                     false);
    if (err) {
        LOG_ERR("Failed to send Accel-X: %d", err);
    }

    double gyro_x = (double)env->gyro_hp[0][0] / GYRO_SCALE;
    err = nrf_cloud_coap_sensor_send("GYRO_X", 
                                     gyro_x, 
                                     timestamp_ms, 
                                     false);
    if (err) {
        LOG_ERR("Failed to send Gyro-X: %d", err);
    }

    return err;
}