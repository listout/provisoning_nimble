/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
/* BLE */
#include "ble_spp_server.h"
#include "console/console.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "wifi_provisioning/manager.h"
#include <wifi_provisioning/scheme_ble.h>


#define PROV_QR_VERSION         "v1"
#define PROV_TRANSPORT_SOFTAP   "softap"
#define PROV_TRANSPORT_BLE      "ble"
#define QRCODE_BASE_URL         "https://espressif.github.io/esp-jumpstart/qrcode.html"
#if CONFIG_EXAMPLE_PROV_SECURITY_VERSION_2
#if CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE
#define EXAMPLE_PROV_SEC2_USERNAME "wifiprov"
#define EXAMPLE_PROV_SEC2_PWD "abcd1234"

static const char *TAG = "test";

/* Signal Wi-Fi events on this event-group */
const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;

/* This salt,verifier has been generated for username = "wifiprov" and password = "abcd1234"
 * IMPORTANT NOTE: For production cases, this must be unique to every device
 * and should come from device manufacturing partition.*/
static const char sec2_salt[] = {
    0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4};

static const char sec2_verifier[] = {
    0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43, 0x78, 0xcf, 0xfd,
    0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24, 0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9,
    0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1, 0x32, 0x24, 0xda, 0x11, 0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a,
    0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89, 0xaa, 0x16, 0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6,
    0xe4, 0x3a, 0x71, 0x2e, 0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff, 0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87,
    0xdc, 0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35, 0x6a, 0xc4, 0x70, 0xad,
    0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e, 0xc3, 0xa5, 0xf9, 0x68, 0xdd,
    0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e, 0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18,
    0x34, 0x29, 0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9, 0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e,
    0x5f, 0x08, 0x33, 0x40, 0x76, 0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05,
    0x44, 0x17, 0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44, 0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23, 0x81,
    0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17, 0xbd, 0x08, 0xb8, 0x96, 0xbc, 0x26, 0xeb, 0x32,
    0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc, 0xa9, 0x38, 0xa6, 0x67, 0xf0, 0xad, 0x50,
    0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2, 0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b, 0xb1, 0xe2,
    0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64, 0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6,
    0x21, 0xe7, 0x2d, 0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84,
    0x53, 0x4a, 0xb3, 0x0c, 0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb, 0xec, 0xf0, 0x10,
    0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84, 0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0, 0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7,
    0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8, 0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71,
    0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1, 0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62,
    0xde, 0x36, 0xb2, 0xba};
#endif

static esp_err_t
example_get_sec2_salt(const char **salt, uint16_t *salt_len)
{
#if CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE
	ESP_LOGI(TAG, "Development mode: using hard coded salt");
	*salt = sec2_salt;
	*salt_len = sizeof(sec2_salt);
	return ESP_OK;
#elif CONFIG_EXAMPLE_PROV_SEC2_PROD_MODE
	ESP_LOGE(TAG, "Not implemented!");
	return ESP_FAIL;
#endif
}

static esp_err_t
example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len)
{
#if CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE
	ESP_LOGI(TAG, "Development mode: using hard coded verifier");
	*verifier = sec2_verifier;
	*verifier_len = sizeof(sec2_verifier);
	return ESP_OK;
#elif CONFIG_EXAMPLE_PROV_SEC2_PROD_MODE
	/* This code needs to be updated with appropriate implementation to provide verifier */
	ESP_LOGE(TAG, "Not implemented!");
	return ESP_FAIL;
#endif
}
#endif

/* Event handler for catching system events */
static void
event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
#ifdef CONFIG_EXAMPLE_RESET_PROV_MGR_ON_FAILURE
	static int retries;
#endif
	if (event_base == WIFI_PROV_EVENT) {
		switch (event_id) {
		case WIFI_PROV_START:
			ESP_LOGI(TAG, "Provisioning started");
			break;
		case WIFI_PROV_CRED_RECV: {
			wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
			ESP_LOGI(TAG,
			         "Received Wi-Fi credentials"
			         "\n\tSSID     : %s\n\tPassword : %s",
			         (const char *)wifi_sta_cfg->ssid,
			         (const char *)wifi_sta_cfg->password);
			break;
		}
		case WIFI_PROV_CRED_FAIL: {
			wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
			ESP_LOGE(TAG,
			         "Provisioning failed!\n\tReason : %s"
			         "\n\tPlease reset to factory and retry provisioning",
			         (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed"
			                                               : "Wi-Fi access-point not found");
#ifdef CONFIG_EXAMPLE_RESET_PROV_MGR_ON_FAILURE
			retries++;
			if (retries >= CONFIG_EXAMPLE_PROV_MGR_MAX_RETRY_CNT) {
				ESP_LOGI(TAG, "Failed to connect with provisioned AP, reseting provisioned credentials");
				wifi_prov_mgr_reset_sm_state_on_failure();
				retries = 0;
			}
#endif
			break;
		}
		case WIFI_PROV_CRED_SUCCESS:
			ESP_LOGI(TAG, "Provisioning successful");
#ifdef CONFIG_EXAMPLE_RESET_PROV_MGR_ON_FAILURE
			retries = 0;
#endif
			break;
		case WIFI_PROV_END:
			/* De-initialize manager once provisioning is finished */
			wifi_prov_mgr_deinit();
			break;
		default:
			break;
		}
	} else if (event_base == WIFI_EVENT) {
		switch (event_id) {
		case WIFI_EVENT_STA_START:
			esp_wifi_connect();
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
			ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
			esp_wifi_connect();
			break;
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
		case WIFI_EVENT_AP_STACONNECTED:
			ESP_LOGI(TAG, "SoftAP transport: Connected!");
			break;
		case WIFI_EVENT_AP_STADISCONNECTED:
			ESP_LOGI(TAG, "SoftAP transport: Disconnected!");
			break;
#endif
		default:
			break;
		}
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
		ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
		/* Signal main application to continue execution */
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
	} else if (event_base == PROTOCOMM_TRANSPORT_BLE_EVENT) {
		switch (event_id) {
		case PROTOCOMM_TRANSPORT_BLE_CONNECTED:
			ESP_LOGI(TAG, "BLE transport: Connected!");
			break;
		case PROTOCOMM_TRANSPORT_BLE_DISCONNECTED:
			ESP_LOGI(TAG, "BLE transport: Disconnected!");
			break;
		default:
			break;
		}
#endif
	} else if (event_base == PROTOCOMM_SECURITY_SESSION_EVENT) {
		switch (event_id) {
		case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
			ESP_LOGI(TAG, "Secured session established!");
			break;
		case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
			ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
			break;
		case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
			ESP_LOGE(TAG, "Received incorrect username and/or PoP for establishing secure session!");
			break;
		default:
			break;
		}
	}
}

static void
wifi_init_sta(void)
{
	/* Start Wi-Fi in station mode */
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());
}

static void
get_device_service_name(char *service_name, size_t max)
{
	uint8_t eth_mac[6];
	const char *ssid_prefix = "PROV_";
	esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
	snprintf(service_name, max, "%s%02X%02X%02X", ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

/* Handler for the optional provisioning endpoint registered by the application.
 * The data format can be chosen by applications. Here, we are using plain ascii text.
 * Applications can choose to use other formats like protobuf, JSON, XML, etc.
 */
esp_err_t
custom_prov_data_handler(
    uint32_t session_id, const uint8_t *inbuf, ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	if (inbuf) {
		ESP_LOGI(TAG, "Received data: %.*s", inlen, (char *)inbuf);
	}
	char response[] = "SUCCESS";
	*outbuf = (uint8_t *)strdup(response);
	if (*outbuf == NULL) {
		ESP_LOGE(TAG, "System out of memory");
		return ESP_ERR_NO_MEM;
	}
	*outlen = strlen(response) + 1; /* +1 for NULL terminating byte */

	return ESP_OK;
}

static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
static uint8_t own_addr_type;
int gatt_svr_register(void);
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
static uint16_t ble_spp_svc_gatt_read_val_handle;

void ble_store_config_init(void);

QueueHandle_t parser_queue; // Queue to receive incoming messages

/**
 * Logs information about a connection to the console.
 */
static void
ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc)
{
	MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=", desc->conn_handle, desc->our_ota_addr.type);
	print_addr(desc->our_ota_addr.val);
	MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=", desc->our_id_addr.type);
	print_addr(desc->our_id_addr.val);
	MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=", desc->peer_ota_addr.type);
	print_addr(desc->peer_ota_addr.val);
	MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=", desc->peer_id_addr.type);
	print_addr(desc->peer_id_addr.val);
	MODLOG_DFLT(INFO,
	            " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
	            "encrypted=%d authenticated=%d bonded=%d\n",
	            desc->conn_itvl,
	            desc->conn_latency,
	            desc->supervision_timeout,
	            desc->sec_state.encrypted,
	            desc->sec_state.authenticated,
	            desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ble_spp_server_advertise(void)
{
	struct ble_gap_adv_params adv_params;
	struct ble_hs_adv_fields fields;
	const char *name;
	int rc;

	/**
	 *  Set the advertisement data included in our advertisements:
	 *     o Flags (indicates advertisement type and other general info).
	 *     o Advertising tx power.
	 *     o Device name.
	 *     o 16-bit service UUIDs (alert notifications).
	 */

	memset(&fields, 0, sizeof fields);

	/* Advertise two flags:
	 *     o Discoverability in forthcoming advertisement (general)
	 *     o BLE-only (BR/EDR unsupported).
	 */
	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

	/* Indicate that the TX power level field should be included; have the
	 * stack fill this value automatically.  This is done by assigning the
	 * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
	 */
	fields.tx_pwr_lvl_is_present = 1;
	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	name = ble_svc_gap_device_name();
	fields.name = (uint8_t *)name;
	fields.name_len = strlen(name);
	fields.name_is_complete = 1;

	fields.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
	fields.num_uuids16 = 1;
	fields.uuids16_is_complete = 1;

	rc = ble_gap_adv_set_fields(&fields);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
		return;
	}

	/* Begin advertising. */
	memset(&adv_params, 0, sizeof adv_params);
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_spp_server_gap_event, NULL);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
		return;
	}
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * ble_spp_server uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  ble_spp_server.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
ble_spp_server_gap_event(struct ble_gap_event *event, void *arg)
{
	struct ble_gap_conn_desc desc;
	int rc;

	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		/* A new connection was established or a connection attempt failed. */
		MODLOG_DFLT(INFO,
		            "connection %s; status=%d ",
		            event->connect.status == 0 ? "established" : "failed",
		            event->connect.status);
		if (event->connect.status == 0) {
			rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
			assert(rc == 0);
			ble_spp_server_print_conn_desc(&desc);
		}
		MODLOG_DFLT(INFO, "\n");
		if (event->connect.status != 0 || CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1) {
			/* Connection failed or if multiple connection allowed; resume
			 * advertising. */
			ble_spp_server_advertise();
		}
		return 0;

	case BLE_GAP_EVENT_DISCONNECT:
		MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
		ble_spp_server_print_conn_desc(&event->disconnect.conn);
		MODLOG_DFLT(INFO, "\n");

		conn_handle_subs[event->disconnect.conn.conn_handle] = false;

		/* Connection terminated; resume advertising. */
		ble_spp_server_advertise();
		return 0;

	case BLE_GAP_EVENT_CONN_UPDATE:
		/* The central has updated the connection parameters. */
		MODLOG_DFLT(INFO, "connection updated; status=%d ", event->conn_update.status);
		rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
		assert(rc == 0);
		ble_spp_server_print_conn_desc(&desc);
		MODLOG_DFLT(INFO, "\n");
		return 0;

	case BLE_GAP_EVENT_ADV_COMPLETE:
		MODLOG_DFLT(INFO, "advertise complete; reason=%d", event->adv_complete.reason);
		ble_spp_server_advertise();
		return 0;

	case BLE_GAP_EVENT_MTU:
		MODLOG_DFLT(INFO,
		            "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
		            event->mtu.conn_handle,
		            event->mtu.channel_id,
		            event->mtu.value);
		return 0;

	case BLE_GAP_EVENT_SUBSCRIBE:
		MODLOG_DFLT(INFO,
		            "subscribe event; conn_handle=%d attr_handle=%d "
		            "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
		            event->subscribe.conn_handle,
		            event->subscribe.attr_handle,
		            event->subscribe.reason,
		            event->subscribe.prev_notify,
		            event->subscribe.cur_notify,
		            event->subscribe.prev_indicate,
		            event->subscribe.cur_indicate);
		conn_handle_subs[event->subscribe.conn_handle] = true;
		return 0;

	default:
		return 0;
	}
}

static void
ble_spp_server_on_reset(int reason)
{
	MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
ble_spp_server_on_sync(void)
{
	int rc;

	rc = ble_hs_util_ensure_addr(0);
	assert(rc == 0);

	/* Figure out address to use while advertising (no privacy for now) */
	rc = ble_hs_id_infer_auto(0, &own_addr_type);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
		return;
	}

	/* Printing ADDR */
	uint8_t addr_val[6] = {0};
	rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

	MODLOG_DFLT(INFO, "Device Address: ");
	print_addr(addr_val);
	MODLOG_DFLT(INFO, "\n");
	/* Begin advertising. */
	ble_spp_server_advertise();
}

void
ble_spp_server_host_task(void *param)
{
	MODLOG_DFLT(INFO, "BLE Host Task Started");
	/* This function will return only when nimble_port_stop() is executed */
	nimble_port_run();

	nimble_port_freertos_deinit();
}

/* Callback function for custom service */
static int
ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		MODLOG_DFLT(INFO, "Callback for read");
		break;

	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		MODLOG_DFLT(INFO, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
		uint8_t msg[50];
		int rc = ble_hs_mbuf_to_flat(ctxt->om, msg, 50, NULL);
		if (rc == 0)
			MODLOG_DFLT(INFO, "Read successfully");
		else
			MODLOG_DFLT(INFO, "Could not read");
		msg[ctxt->om->om_len - 2] = '\0';
		MODLOG_DFLT(INFO, "Got message %s", msg);
		xQueueSendToBack(parser_queue, msg, 1000 / portTICK_PERIOD_MS);
		break;

	default:
		MODLOG_DFLT(INFO, "\nDefault Callback");
		break;
	}
	return 0;
}

/* Define new custom service */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        /*** Service: SPP */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics =
            (struct ble_gatt_chr_def[]){{
                                            /* Support SPP service */
                                            .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16),
                                            .access_cb = ble_svc_gatt_handler,
                                            .val_handle = &ble_spp_svc_gatt_read_val_handle,
                                            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                                        },
                                        {
                                            0, /* No more characteristics */
                                        }},
    },
    {
        0, /* No more services. */
    },
};

static void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
	char buf[BLE_UUID_STR_LEN];

	switch (ctxt->op) {
	case BLE_GATT_REGISTER_OP_SVC:
		MODLOG_DFLT(DEBUG,
		            "registered service %s with handle=%d\n",
		            ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
		            ctxt->svc.handle);
		break;

	case BLE_GATT_REGISTER_OP_CHR:
		MODLOG_DFLT(DEBUG,
		            "registering characteristic %s with "
		            "def_handle=%d val_handle=%d\n",
		            ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
		            ctxt->chr.def_handle,
		            ctxt->chr.val_handle);
		break;

	case BLE_GATT_REGISTER_OP_DSC:
		MODLOG_DFLT(DEBUG,
		            "registering descriptor %s with handle=%d\n",
		            ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
		            ctxt->dsc.handle);
		break;

	default:
		assert(0);
		break;
	}
}

int
gatt_svr_init_t(void)
{
	int rc = 0;
	ble_svc_gap_init();
	ble_svc_gatt_init();

	rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

	if (rc != 0) {
		return rc;
	}

	rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
	if (rc != 0) {
		return rc;
	}

	return 0;
}

void
provision_start()
{
	nimble_port_stop();
	nimble_port_deinit();
	/* Initialize TCP/IP */
	ESP_ERROR_CHECK(esp_netif_init());

	/* Initialize the event loop */
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	wifi_event_group = xEventGroupCreate();

	/* Register our event handler for Wi-Fi, IP and Provisioning related events */
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
	ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_TRANSPORT_BLE_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
#endif
	ESP_ERROR_CHECK(
	    esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

	/* Initialize Wi-Fi including netif with default config */
	esp_netif_create_default_wifi_sta();
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
	esp_netif_create_default_wifi_ap();
#endif /* CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP */
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	/* Configuration for the provisioning manager */
	wifi_prov_mgr_config_t config = {
	/* What is the Provisioning Scheme that we want ?
	 * wifi_prov_scheme_softap or wifi_prov_scheme_ble */
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
	    .scheme = wifi_prov_scheme_ble,
#endif /* CONFIG_EXAMPLE_PROV_TRANSPORT_BLE */
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
	    .scheme = wifi_prov_scheme_softap,
#endif /* CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP */

	/* Any default scheme specific event handler that you would
	 * like to choose. Since our example application requires
	 * neither BT nor BLE, we can choose to release the associated
	 * memory once provisioning is complete, or not needed
	 * (in case when device is already provisioned). Choosing
	 * appropriate scheme specific event handler allows the manager
	 * to take care of this automatically. This can be set to
	 * WIFI_PROV_EVENT_HANDLER_NONE when using wifi_prov_scheme_softap*/
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
	    .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
#endif /* CONFIG_EXAMPLE_PROV_TRANSPORT_BLE */
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
	                                .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE
#endif /* CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP */
	};

	/* Initialize provisioning manager with the
	 * configuration parameters set above */
	ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

	bool provisioned = false;
#ifdef CONFIG_EXAMPLE_RESET_PROVISIONED
	wifi_prov_mgr_reset_provisioning();
#else
	/* Let's find out if the device is provisioned */
	ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

#endif
	/* If device is not yet provisioned start provisioning service */
	if (!provisioned) {
		ESP_LOGI(TAG, "Starting provisioning");

		/* What is the Device Service Name that we want
		 * This translates to :
		 *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
		 *     - device name when scheme is wifi_prov_scheme_ble
		 */
		char service_name[12];
		get_device_service_name(service_name, sizeof(service_name));

#ifdef CONFIG_EXAMPLE_PROV_SECURITY_VERSION_1
		/* What is the security level that we want (0, 1, 2):
		 *      - WIFI_PROV_SECURITY_0 is simply plain text communication.
		 *      - WIFI_PROV_SECURITY_1 is secure communication which consists of secure handshake
		 *          using X25519 key exchange and proof of possession (pop) and AES-CTR
		 *          for encryption/decryption of messages.
		 *      - WIFI_PROV_SECURITY_2 SRP6a based authentication and key exchange
		 *        + AES-GCM encryption/decryption of messages
		 */
		wifi_prov_security_t security = WIFI_PROV_SECURITY_1;

		/* Do we want a proof-of-possession (ignored if Security 0 is selected):
		 *      - this should be a string with length > 0
		 *      - NULL if not used
		 */
		const char *pop = "abcd1234";

		/* This is the structure for passing security parameters
		 * for the protocomm security 1.
		 */
		wifi_prov_security1_params_t *sec_params = pop;

		const char *username = NULL;

#elif CONFIG_EXAMPLE_PROV_SECURITY_VERSION_2
		wifi_prov_security_t security = WIFI_PROV_SECURITY_2;
		/* The username must be the same one, which has been used in the generation of salt and verifier */

#if CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE
		/* This pop field represents the password that will be used to generate salt and verifier.
		 * The field is present here in order to generate the QR code containing password.
		 * In production this password field shall not be stored on the device */
		/*
		 *const char *username = EXAMPLE_PROV_SEC2_USERNAME;
		 *const char *pop = EXAMPLE_PROV_SEC2_PWD;
		 */
#elif CONFIG_EXAMPLE_PROV_SEC2_PROD_MODE
		/* The username and password shall not be embedded in the firmware,
		 * they should be provided to the user by other means.
		 * e.g. QR code sticker */
		const char *username = NULL;
		const char *pop = NULL;
#endif
		/* This is the structure for passing security parameters
		 * for the protocomm security 2.
		 * If dynamically allocated, sec2_params pointer and its content
		 * must be valid till WIFI_PROV_END event is triggered.
		 */
		wifi_prov_security2_params_t sec2_params = {};

		ESP_ERROR_CHECK(example_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
		ESP_ERROR_CHECK(example_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

		wifi_prov_security2_params_t *sec_params = &sec2_params;
#endif
		/* What is the service key (could be NULL)
		 * This translates to :
		 *     - Wi-Fi password when scheme is wifi_prov_scheme_softap
		 *          (Minimum expected length: 8, maximum 64 for WPA2-PSK)
		 *     - simply ignored when scheme is wifi_prov_scheme_ble
		 */
		const char *service_key = NULL;

#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
		/* This step is only useful when scheme is wifi_prov_scheme_ble. This will
		 * set a custom 128 bit UUID which will be included in the BLE advertisement
		 * and will correspond to the primary GATT service that provides provisioning
		 * endpoints as GATT characteristics. Each GATT characteristic will be
		 * formed using the primary service UUID as base, with different auto assigned
		 * 12th and 13th bytes (assume counting starts from 0th byte). The client side
		 * applications must identify the endpoints by reading the User Characteristic
		 * Description descriptor (0x2901) for each characteristic, which contains the
		 * endpoint name of the characteristic */
		uint8_t custom_service_uuid[] = {
		    /* LSB <---------------------------------------
		     * ---------------------------------------> MSB */
		    0xb4,
		    0xdf,
		    0x5a,
		    0x1c,
		    0x3f,
		    0x6b,
		    0xf4,
		    0xbf,
		    0xea,
		    0x4a,
		    0x82,
		    0x03,
		    0x04,
		    0x90,
		    0x1a,
		    0x02,
		};

		/* If your build fails with linker errors at this point, then you may have
		 * forgotten to enable the BT stack or BTDM BLE settings in the SDK (e.g. see
		 * the sdkconfig.defaults in the example project) */
		wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);
#endif /* CONFIG_EXAMPLE_PROV_TRANSPORT_BLE */

		/* An optional endpoint that applications can create if they expect to
		 * get some additional custom data during provisioning workflow.
		 * The endpoint name can be anything of your choice.
		 * This call must be made before starting the provisioning.
		 */
		wifi_prov_mgr_endpoint_create("custom-data");

		/* Do not stop and de-init provisioning even after success,
		 * so that we can restart it later. */
#ifdef CONFIG_EXAMPLE_REPROVISIONING
		wifi_prov_mgr_disable_auto_stop(1000);
#endif
		/* Start provisioning service */
		ESP_ERROR_CHECK(
		    wifi_prov_mgr_start_provisioning(security, (const void *)sec_params, service_name, service_key));

		/* The handler for the optional endpoint created above.
		 * This call must be made after starting the provisioning, and only if the endpoint
		 * has already been created above.
		 */
		wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);

		/* Uncomment the following to wait for the provisioning to finish and then release
		 * the resources of the manager. Since in this case de-initialization is triggered
		 * by the default event loop handler, we don't need to call the following */
		// wifi_prov_mgr_wait();
		// wifi_prov_mgr_deinit();
		/* Print QR code for provisioning */
	} else {
		ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

		/* We don't need the manager as device is already provisioned,
		 * so let's release it's resources */
		wifi_prov_mgr_deinit();

		/* Start Wi-Fi station */
		wifi_init_sta();
	}

	/* Wait for Wi-Fi connection */
	xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);

	/* Start main application now */
#if CONFIG_EXAMPLE_REPROVISIONING
	while (1) {
		for (int i = 0; i < 10; i++) {
			ESP_LOGI(TAG, "Hello World!");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}

		/* Resetting provisioning state machine to enable re-provisioning */
		wifi_prov_mgr_reset_sm_state_for_reprovision();

		/* Wait for Wi-Fi connection */
		xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);
	}
#else
	while (1) {
		ESP_LOGI(TAG, "Hello World!");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
#endif
}

void
parser_task(void *args)
{
	uint8_t msg[50];
	for (;;) {
		if (xQueueReceive(parser_queue, msg, 6000 / portTICK_PERIOD_MS) != pdTRUE)
			continue;

		MODLOG_DFLT(INFO, "In parser task got: %s", msg);

		if ((strcmp((const char *)msg, "reboot") == 0)) {
			esp_restart();
		} else if ((strcmp((const char *)msg, "prov") == 0)) {
			provision_start();
		}
	}
}

void
app_main(void)
{
	int rc;

	/* Initialize NVS — it is used to store PHY calibration data */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ret = nimble_port_init();
	if (ret != ESP_OK) {
		MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
		return;
	}

	/* Initialize connection_handle array */
	for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
		conn_handle_subs[i] = false;
	}

	parser_queue = xQueueCreate(10, sizeof(uint8_t) * 50);
	xTaskCreatePinnedToCore(parser_task, "parser task", 1024 * 4, NULL, 4, NULL, 0);

	/* Initialize the NimBLE host configuration. */
	ble_hs_cfg.reset_cb = ble_spp_server_on_reset;
	ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
	ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

	ble_hs_cfg.sm_io_cap = 3;
#ifdef CONFIG_EXAMPLE_BONDING
	ble_hs_cfg.sm_bonding = 1;
#endif
#ifdef CONFIG_EXAMPLE_MITM
	ble_hs_cfg.sm_mitm = 1;
#endif
#ifdef CONFIG_EXAMPLE_USE_SC
	ble_hs_cfg.sm_sc = 1;
#else
	ble_hs_cfg.sm_sc = 0;
#endif
#ifdef CONFIG_EXAMPLE_BONDING
	ble_hs_cfg.sm_our_key_dist = 1;
	ble_hs_cfg.sm_their_key_dist = 1;
#endif

	/* Register custom service */
	rc = gatt_svr_init_t();
	assert(rc == 0);

	/* Set the default device name. */
	rc = ble_svc_gap_device_name_set("nimble-ble-spp-svr");
	assert(rc == 0);

	/* XXX Need to have template for store */
	ble_store_config_init();

	nimble_port_freertos_init(ble_spp_server_host_task);
}
