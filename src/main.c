/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#if defined(CONFIG_NRF_MODEM_LIB)
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>
#include <nrf_modem.h>
#endif
#include <net/aws_iot.h>
#include <zephyr/sys/reboot.h>
#include <date_time.h>
#include <zephyr/dfu/mcuboot.h>
#include <cJSON.h>
#include <cJSON_os.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#if defined(CONFIG_AWS_IOT_SAMPLE_DEVICE_ID_USE_HW_ID)
#include <hw_id.h>
#endif

#include "sensor.h"

#define I2C_RST			DT_ALIAS(rstpin)
#define ENABLE_3V6 		DT_ALIAS(enable3v6)

#define SENSOR_SAMPLE_INTERVAL_MS		3000
LOG_MODULE_REGISTER(aws_iot_sample, CONFIG_AWS_IOT_SAMPLE_LOG_LEVEL);

BUILD_ASSERT(!IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT),
	     "This sample does not support LTE auto-init and connect");

#define APP_TOPICS_COUNT CONFIG_AWS_IOT_APP_SUBSCRIPTION_LIST_COUNT
#define SEND_TOPIC	"cagewatch/v1/ingestion"

static struct k_work_delayable publish_work;
static struct k_work_delayable connect_work;
static struct k_work shadow_update_version_work;

static bool cloud_connected;

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(date_time_obtained, 0, 1);

static const struct i2c_dt_spec	tca9548a = I2C_DT_SPEC_GET( DT_NODELABEL( tca9548a_70 ) );
static const struct gpio_dt_spec i2c_rst_dt = GPIO_DT_SPEC_GET(I2C_RST, gpios);
static const struct gpio_dt_spec enable_3v6_dt = GPIO_DT_SPEC_GET(ENABLE_3V6, gpios);

static float lux[8] = {0.0};
static int range[8] = {0};
static uint8_t channel_mask = 0x00;

void sensors_init(void)
{
	uint8_t ch, ret;

	if (!gpio_is_ready_dt(&i2c_rst_dt)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&i2c_rst_dt, GPIO_OUTPUT_HIGH);
	if (ret < 0) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&enable_3v6_dt, GPIO_OUTPUT_HIGH);
	if (ret < 0) {
		return 0;
	}


	check_i2c_device(&tca9548a);

	channel_mask = 0x00;

	for (ch=0; ch<8; ch++){
		if(tca9548a_set_channel(&tca9548a, ch) == 0){
			LOG_INF("Channel %d is set\n", ch);
			
			if (vl6180_read8(&tca9548a, VL6180X_REG_IDENTIFICATION_MODEL_ID) != 0xB4){
				LOG_ERR("Vl6180 sensor is failed in Channel %d\n", ch);
				range[ch] = -1;
			}
			else{
				if(vl6180_read8(&tca9548a, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET) & 0x01){
					loadSettings(&tca9548a);
		 			vl6180_write8( &tca9548a, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
				}

        		channel_mask |= (1 << ch);
				LOG_INF("Vl6180 sensor in Channel %d was initiated successfully\n", ch);
			}

		}
		else{
			LOG_ERR("Channel %d is failed\n", ch);
			range[ch] = -1;
		}
	}
}

void check_sensors(void)
{
	uint8_t ch;
	uint8_t pre_channel_mask;

	check_i2c_device(&tca9548a);

	pre_channel_mask = channel_mask;

	channel_mask = 0x00;

	for (ch=0; ch<8; ch++){
		if(tca9548a_set_channel(&tca9548a, ch) == 0){
			LOG_INF("Channel %d is set\n", ch);
			
			if (vl6180_read8(&tca9548a, VL6180X_REG_IDENTIFICATION_MODEL_ID) != 0xB4){
				LOG_ERR("Vl6180 sensor is failed in Channel %d\n", ch);
				range[ch] = -1;
			}
			else{
				if(((pre_channel_mask >> ch) & 0x01) == 0x00){
					if(vl6180_read8(&tca9548a, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET) & 0x01){
						loadSettings(&tca9548a);
		 				vl6180_write8( &tca9548a, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
					}
				}
        		channel_mask |= (1 << ch);
				LOG_INF("Vl6180 sensor in Channel %d was initiated successfully\n", ch);
			}

		}
		else{
			LOG_ERR("Channel %d is failed\n", ch);
			range[ch] = -1;
		}
	}
}
static void sensor_measure_fn(void)
{
	uint8_t ch;

	if(channel_mask > 0){
		for (ch=0; ch<8; ch++){
			if(channel_mask & (1 << ch)){
    	    	tca9548a_set_channel(&tca9548a, ch);
				lux[ch] = vl6180_readLux(&tca9548a, VL6180X_ALS_GAIN_5);
				range[ch] = vl6180_readRange(&tca9548a);
				LOG_INF("Data in Channel %d: [lux:] %f, [range:] %d\n", ch, lux[ch], range[ch]);
			}
		}
	}
	else {
		LOG_ERR("Sensor channels are not finded");
	}
}

static int json_add_obj(cJSON *parent, const char *str, cJSON *item)
{
	cJSON_AddItemToObject(parent, str, item);

	return 0;
}

static int json_add_str(cJSON *parent, const char *str, const char *item)
{
	cJSON *json_str;

	json_str = cJSON_CreateString(item);
	if (json_str == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_str);
}

static int json_add_number(cJSON *parent, const char *str, double item)
{
	cJSON *json_num;

	json_num = cJSON_CreateNumber(item);
	if (json_num == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_num);
}
/*
	Send the data to the AWS IoT Backend
*/
static int publish_func(bool version_number_include)
{
	int err;
	char *message;
	int64_t message_ts = 0;
	int16_t bat_voltage = 0;
	char imei[16]; 
	int16_t rssi = 0;
	uint8_t ch = 0;

	err = date_time_now(&message_ts);
	if (err) {
		LOG_ERR("date_time_now, error: %d", err);
		return err;
	}

	check_sensors();

	sensor_measure_fn();

#if defined(CONFIG_NRF_MODEM_LIB)
	/* Request battery voltage data from the modem. */
	err = modem_info_short_get(MODEM_INFO_BATTERY, &bat_voltage);
	if (err != sizeof(bat_voltage)) {
		LOG_ERR("modem_info_short_get for battery voltage, error: %d", err);
		return err;
	}
	err = modem_info_string_get(MODEM_INFO_IMEI, imei, 16);
	if (err != (sizeof(imei) - 1)) {
		LOG_ERR("modem_info_string_get for IMEI, error: %d", err);
		return err;
	}
	err = modem_info_short_get(MODEM_INFO_RSRP, &rssi);
	if (err != sizeof(rssi)) {
		LOG_ERR("modem_info_short_get for RSSI, error: %d", err);
		return err;
	}

	rssi = (-140) + rssi; 
#endif

	cJSON *root_obj = cJSON_CreateObject();
	cJSON *data_obj = cJSON_CreateObject();
	

	if (root_obj == NULL || data_obj == NULL)
	{
		cJSON_Delete(data_obj);
		cJSON_Delete(root_obj);
		err = -ENOMEM;
		return err;
	}

	err = 0;

	err += json_add_str(root_obj, "imei", imei);
	err += json_add_str(root_obj, "firmware_version", CONFIG_FIRMWARE_VERSION);
	err += json_add_str(root_obj, "hardware_version", CONFIG_HARDWARE_VERSION);
	err += json_add_number(root_obj, "battery_voltage", bat_voltage);
	err += json_add_number(root_obj, "cellular_rssi", rssi);


	err += json_add_number(data_obj, "ch0", range[0]);	
	err += json_add_number(data_obj, "ch1", range[1]);	
	err += json_add_number(data_obj, "ch2", range[2]);	
	err += json_add_number(data_obj, "ch3", range[3]);	
	err += json_add_number(data_obj, "ch4", range[4]);	
	err += json_add_number(data_obj, "ch5", range[5]);	
	err += json_add_number(data_obj, "ch6", range[6]);	
	err += json_add_number(data_obj, "ch7", range[7]);	
	

	err += json_add_obj(root_obj, "data", data_obj);
	

	if (err) {
		LOG_ERR("json_add, error: %d", err);
		goto cleanup;
	}

	message = cJSON_Print(root_obj);
	if (message == NULL) {
		LOG_ERR("cJSON_Print, error: returned NULL");
		err = -ENOMEM;
		goto cleanup;
	}

	struct aws_iot_data tx_data = {
		.qos = MQTT_QOS_0_AT_MOST_ONCE,
		.topic.type = 0,
		.topic.str = SEND_TOPIC,
		.topic.len = strlen(SEND_TOPIC),
		.ptr = message,
		.len = strlen(message)
	};

	LOG_INF("Publishing: %s to AWS IoT broker", message);

	err = aws_iot_send(&tx_data);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
	}

	cJSON_FreeString(message);

cleanup:

	cJSON_Delete(root_obj);

	return err;
}

static void connect_work_fn(struct k_work *work)
{
	int err;

	if (cloud_connected) {
		return;
	}

	err = aws_iot_connect(NULL);
	if (err) {
		LOG_ERR("aws_iot_connect, error: %d", err);
	}

	LOG_INF("Next connection retry in %d seconds",
		CONFIG_AWS_IOT_SAMPLE_CONNECTION_RETRY_TIMEOUT_SECONDS);

	k_work_schedule(&connect_work,
			K_SECONDS(CONFIG_AWS_IOT_SAMPLE_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

static void publish_work_fn(struct k_work *work)
{
	int err;

	if (!cloud_connected) {
		return;
	}

	err = publish_func(false);
	if (err) {
		LOG_ERR("shadow_update, error: %d", err);
	}

	LOG_INF("Next data publication in %d seconds",
		CONFIG_AWS_IOT_SAMPLE_PUBLICATION_INTERVAL_SECONDS);

	k_work_schedule(&publish_work,
			K_SECONDS(CONFIG_AWS_IOT_SAMPLE_PUBLICATION_INTERVAL_SECONDS));
}

static void shadow_update_version_work_fn(struct k_work *work)
{
	int err;

	err = publish_func(true);
	if (err) {
		LOG_ERR("shadow_update, error: %d", err);
	}
}

static void print_received_data(const char *buf, const char *topic,
				size_t topic_len)
{
	char *str = NULL;
	cJSON *root_obj = NULL;

	root_obj = cJSON_Parse(buf);
	if (root_obj == NULL) {
		LOG_ERR("cJSON Parse failure");
		return;
	}

	str = cJSON_Print(root_obj);
	if (str == NULL) {
		LOG_ERR("Failed to print JSON object");
		goto clean_exit;
	}

	LOG_INF("Data received from AWS IoT console: Topic: %.*s Message: %s",
		topic_len, topic, str);

	cJSON_FreeString(str);

clean_exit:
	cJSON_Delete(root_obj);
}

void aws_iot_event_handler(const struct aws_iot_evt *const evt)
{
	switch (evt->type) {
	case AWS_IOT_EVT_CONNECTING:
		LOG_INF("AWS_IOT_EVT_CONNECTING");
		break;
	case AWS_IOT_EVT_CONNECTED:
		LOG_INF("AWS_IOT_EVT_CONNECTED");

		cloud_connected = true;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&connect_work);

		if (evt->data.persistent_session) {
			LOG_INF("Persistent session enabled");
		}

#if defined(CONFIG_NRF_MODEM_LIB)
		/** Successfully connected to AWS IoT broker, mark image as
		 *  working to avoid reverting to the former image upon reboot.
		 */
		boot_write_img_confirmed();
#endif

		/** Send version number to AWS IoT broker to verify that the
		 *  FOTA update worked.
		 */
		k_work_submit(&shadow_update_version_work);

		/** Start sequential shadow data updates.
		 */
		k_work_schedule(&publish_work,
				K_SECONDS(CONFIG_AWS_IOT_SAMPLE_PUBLICATION_INTERVAL_SECONDS));

#if defined(CONFIG_NRF_MODEM_LIB)
		int err = lte_lc_psm_req(true);
		if (err) {
			LOG_ERR("Requesting PSM failed, error: %d", err);
		}
#endif
		break;
	case AWS_IOT_EVT_READY:
		LOG_INF("AWS_IOT_EVT_READY");
		break;
	case AWS_IOT_EVT_DISCONNECTED:
		LOG_INF("AWS_IOT_EVT_DISCONNECTED");
		cloud_connected = false;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&publish_work);
		k_work_schedule(&connect_work, K_NO_WAIT);
		break;
	case AWS_IOT_EVT_DATA_RECEIVED:
		LOG_INF("AWS_IOT_EVT_DATA_RECEIVED");
		print_received_data(evt->data.msg.ptr, evt->data.msg.topic.str,
				    evt->data.msg.topic.len);
		break;
	case AWS_IOT_EVT_PUBACK:
		LOG_INF("AWS_IOT_EVT_PUBACK, message ID: %d", evt->data.message_id);
		break;
	case AWS_IOT_EVT_FOTA_START:
		LOG_INF("AWS_IOT_EVT_FOTA_START");
		break;
	case AWS_IOT_EVT_FOTA_ERASE_PENDING:
		LOG_INF("AWS_IOT_EVT_FOTA_ERASE_PENDING");
		LOG_INF("Disconnect LTE link or reboot");
#if defined(CONFIG_NRF_MODEM_LIB)
		err = lte_lc_offline();
		if (err) {
			LOG_ERR("Error disconnecting from LTE");
		}
#endif
		break;
	case AWS_IOT_EVT_FOTA_ERASE_DONE:
		LOG_INF("AWS_FOTA_EVT_ERASE_DONE");
		LOG_INF("Reconnecting the LTE link");
#if defined(CONFIG_NRF_MODEM_LIB)
		err = lte_lc_connect();
		if (err) {
			LOG_ERR("Error connecting to LTE");
		}
#endif
		break;
	case AWS_IOT_EVT_FOTA_DONE:
		LOG_INF("AWS_IOT_EVT_FOTA_DONE");
		LOG_INF("FOTA done, rebooting device");
		aws_iot_disconnect();
		sys_reboot(0);
		break;
	case AWS_IOT_EVT_FOTA_DL_PROGRESS:
		LOG_INF("AWS_IOT_EVT_FOTA_DL_PROGRESS, (%d%%)", evt->data.fota_progress);
	case AWS_IOT_EVT_ERROR:
		LOG_INF("AWS_IOT_EVT_ERROR, %d", evt->data.err);
		break;
	case AWS_IOT_EVT_FOTA_ERROR:
		LOG_INF("AWS_IOT_EVT_FOTA_ERROR");
		break;
	default:
		LOG_WRN("Unknown AWS IoT event type: %d", evt->type);
		break;
	}
}

static void work_init(void)
{
	k_work_init_delayable(&publish_work, publish_work_fn);
	k_work_init_delayable(&connect_work, connect_work_fn);
	k_work_init(&shadow_update_version_work, shadow_update_version_work_fn);
}

#if defined(CONFIG_NRF_MODEM_LIB)
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		     (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		LOG_INF("Network registration status: %s",
			evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
			"Connected - home network" : "Connected - roaming");

		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		LOG_INF("PSM parameter update: TAU: %d, Active time: %d",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			LOG_INF("%s", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s",
			evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
			"Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		LOG_INF("LTE cell changed: Cell ID: %d, Tracking area: %d",
			evt->cell.id, evt->cell.tac);
		break;
	default:
		break;
	}
}

static void nrf_modem_lib_dfu_handler(int err)
{
	switch (err) {
	case NRF_MODEM_DFU_RESULT_OK:
		LOG_INF("Modem update suceeded, reboot");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_UUID_ERROR:
	case NRF_MODEM_DFU_RESULT_AUTH_ERROR:
		LOG_INF("Modem update failed, error: %d", err);
		LOG_INF("Modem will use old firmware");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_HARDWARE_ERROR:
	case NRF_MODEM_DFU_RESULT_INTERNAL_ERROR:
		LOG_INF("Modem update malfunction, error: %d, reboot", err);
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_VOLTAGE_LOW:
		LOG_INF("Modem update cancelled due to low power, error: %d", err);
		LOG_INF("Please reboot once you have sufficient power for the DFU");
		break;
	default:
		break;
	}
}
#endif

static int app_topics_subscribe(void)
{
	int err;
	static char custom_topic[75] = "my-custom-topic/example";
	static char custom_topic_2[75] = "my-custom-topic/example_2";

	const struct aws_iot_topic_data topics_list[APP_TOPICS_COUNT] = {
		[0].str = custom_topic,
		[0].len = strlen(custom_topic),
		[1].str = custom_topic_2,
		[1].len = strlen(custom_topic_2)
	};

	err = aws_iot_subscription_topics_add(topics_list, ARRAY_SIZE(topics_list));
	if (err) {
		LOG_ERR("aws_iot_subscription_topics_add, error: %d", err);
	}

	return err;
}

static void date_time_event_handler(const struct date_time_evt *evt)
{
	switch (evt->type) {
	case DATE_TIME_OBTAINED_MODEM:
		/* Fall through */
	case DATE_TIME_OBTAINED_NTP:
		/* Fall through */
	case DATE_TIME_OBTAINED_EXT:
		LOG_INF("Date time obtained");
		k_sem_give(&date_time_obtained);

		/* De-register handler. At this point the sample will have
		 * date time to depend on indefinitely until a reboot occurs.
		 */
		date_time_register_handler(NULL);
		break;
	case DATE_TIME_NOT_OBTAINED:
		LOG_INF("DATE_TIME_NOT_OBTAINED");
		break;
	default:
		LOG_ERR("Unknown event: %d", evt->type);
		break;
	}
}

int main(void)
{
	int err;

	LOG_INF("The AWS IoT sample started, version: %s", CONFIG_AWS_IOT_SAMPLE_APP_VERSION);

	cJSON_Init();

#if defined(CONFIG_NRF_MODEM_LIB)
	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Modem library initialization failed, error: %d", err);
		return 0;
	}

	nrf_modem_lib_dfu_handler(err);
#endif

#if defined(CONFIG_AWS_IOT_SAMPLE_DEVICE_ID_USE_HW_ID)
	char device_id[HW_ID_LEN] = { 0 };

	err = hw_id_get(device_id, ARRAY_SIZE(device_id));
	if (err) {
		LOG_ERR("Failed to retrieve device ID, error: %d", err);
		return 0;
	}

	struct aws_iot_config config = {
	    .client_id = device_id,
	    .client_id_len = strlen(device_id)
	};

	LOG_INF("Device id: %s", device_id);

	err = aws_iot_init(&config, aws_iot_event_handler);
#else
	err = aws_iot_init(NULL, aws_iot_event_handler);
#endif
	if (err) {
		LOG_ERR("AWS IoT library could not be initialized, error: %d", err);
	}

	sensors_init();

	/** Subscribe to customizable non-shadow specific topics
	 *  to AWS IoT backend.
	 */
	err = app_topics_subscribe();
	if (err) {
		LOG_ERR("Adding application specific topics failed, error: %d", err);
	}

	work_init();
#if defined(CONFIG_NRF_MODEM_LIB)
	err = lte_lc_init_and_connect_async(lte_handler);
	if (err) {
		LOG_ERR("Modem could not be configured, error: %d", err);
		return 0;
	}

	err = modem_info_init();
	if (err) {
		LOG_ERR("Failed initializing modem info module, error: %d", err);
	}

	k_sem_take(&lte_connected, K_FOREVER);
#endif

	/* Trigger a date time update. The date_time API is used to timestamp data that is sent
	 * to AWS IoT.
	 */
	date_time_update_async(date_time_event_handler);

	/* Postpone connecting to AWS IoT until date time has been obtained. */
	k_sem_take(&date_time_obtained, K_FOREVER);
	k_work_schedule(&connect_work, K_NO_WAIT);
	return 0;
}
