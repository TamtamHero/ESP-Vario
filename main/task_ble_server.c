#include "nimble/nimble_port_freertos.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "config.h"
#include "task_ble_server.h"

#define TAG "ble_server"

// first half of sha256 of "vario", "vario rx", "vario tx"
#define SVC_UUID 0x77, 0x07, 0x48, 0x05, 0x17, 0xb3, 0x5e, 0x6e, 0x46, 0x4c, 0x17, 0xa0, 0x76, 0x84, 0xe2, 0x23
#define CHR_RX_UUID 0x25, 0xe7, 0x77, 0x8e, 0xa6, 0x4e, 0xfe, 0x69, 0x50, 0x6a, 0xd5, 0xb7, 0x81, 0x9c, 0x49, 0xbc
#define CHR_TX_UUID 0x23, 0x5e, 0x87, 0xa2, 0x05, 0x32, 0xb4, 0xee, 0x24, 0x9e, 0xda, 0xbf, 0xc9, 0xbb, 0x67, 0xe1

static int ble_server_gap_event(struct ble_gap_event *event, void *arg);

static void ble_server_on_reset(int reason);
static void ble_server_on_sync(void);

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static uint8_t vario_addr_type;
static bool notify_state;
static uint16_t conn_handle;
static uint16_t chr_tx_handle;
static const char *device_name = "ESP-Vario";

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Variometer */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: Rx */
                .uuid = BLE_UUID128_DECLARE(CHR_RX_UUID),
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN,
            }, {
                /* Characteristic: Tx */
                .uuid = BLE_UUID128_DECLARE(CHR_TX_UUID),
                .access_cb = gatt_svr_chr_access,
                .val_handle = &chr_tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },
    {
        0, /* No more services */
    },
};

void ble_server_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

esp_err_t ble_server_init(void) {
    ESP_ERROR_CHECK(nimble_port_init());

    // Initialize the NimBLE host
    ble_hs_cfg.reset_cb = ble_server_on_reset;
    ble_hs_cfg.sync_cb = ble_server_on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = NULL;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }

    // Set device name and power level
    rc = ble_svc_gap_device_name_set(device_name);
    if (rc != 0) {
        return ESP_FAIL;
    }

    // Start the host task
    nimble_port_freertos_init(ble_server_host_task);

    return ESP_OK;
}

/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void ble_server_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
       ESP_LOGE(TAG, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(vario_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_server_gap_event, NULL);
    if (rc != 0) {
       ESP_LOGE(TAG, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void ble_server_on_reset(int reason) {
    ESP_LOGE(TAG, "Resetting state; reason=%d\n", reason);
}

static void ble_server_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &vario_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(vario_addr_type, addr_val, NULL);

    ESP_LOGI(TAG, "Device Address: ");
    ESP_LOG_BUFFER_HEX(TAG, addr_val, sizeof(addr_val));

    /* Begin advertising */
    ble_server_advertise();
}

static int ble_server_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            /* A new connection was established or a connection attempt failed */
            ESP_LOGI(TAG, "connection %s; status=%d\n",
                        event->connect.status == 0 ? "established" : "failed",
                        event->connect.status);

            if (event->connect.status != 0) {
                /* Connection failed; resume advertising */
                ble_server_advertise();
            }
            conn_handle = event->connect.conn_handle;
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnect; reason=%d\n", event->disconnect.reason);

            /* Connection terminated; resume advertising */
            ble_server_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "adv complete\n");
            ble_server_advertise();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "subscribe event; cur_notify=%d\n value handle; val_handle=%d\n", event->subscribe.cur_notify, chr_tx_handle);
            if (event->subscribe.attr_handle == chr_tx_handle) {
                notify_state = event->subscribe.cur_notify;
            } else if (event->subscribe.attr_handle != chr_tx_handle) {
                notify_state = event->subscribe.cur_notify;
            }
            ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "mtu update event; conn_handle=%d mtu=%d\n", event->mtu.conn_handle, event->mtu.value);
            break;
    }

    return 0;
}

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc = 0;
    char* msg = "hello";
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, msg, strlen(msg));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = 0;
            return rc;

        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/*
LK8000 EXTERNAL INSTRUMENT SERIES 1 - NMEA SENTENCE: LK8EX1
VERSION A, 110217

LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

Field 0, raw pressure in hPascal:
	hPA*100 (example for 1013.25 becomes  101325)
	no padding (987.25 becomes 98725, NOT 098725)
	If no pressure available, send 999999 (6 times 9)
	If pressure is available, field 1 altitude will be ignored

Field 1, altitude in meters, relative to QNH 1013.25
	If raw pressure is available, this value will be IGNORED (you can set it to 99999
	but not really needed)!
	(if you want to use this value, set raw pressure to 999999)
	This value is relative to sea level (QNE). We are assuming that
	currently at 0m altitude pressure is standard 1013.25.
	If you cannot send raw altitude, then send what you have but then
	you must NOT adjust it from Basic Setting in LK.
	Altitude can be negative
	If altitude not available, and Pressure not available, set Altitude
	to 99999  (5 times 9)
	LK will say "Baro altitude available" if one of fields 0 and 1 is available.

Field 2, vario in cm/s
	If vario not available, send 9999  (4 times 9)
	Value can also be negative

Field 3, temperature in C , can be also negative
	If not available, send 99

Field 4, battery voltage or charge percentage
	Cannot be negative
	If not available, send 999 (3 times 9)
	Voltage is sent as float value like: 0.1 1.4 2.3  11.2
	To send percentage, add 1000. Example 0% = 1000
	14% = 1014 .  Do not send float values for percentages.
	Percentage should be 0 to 100, with no decimals, added by 1000!
*/

static uint8_t ble_nmea_checksum(const char *sz_nmea) {
    const char* sz = &sz_nmea[1]; // skip leading '$'
    uint8_t cksum = 0;
    while ((*sz) != 0 && (*sz != '*')) {
        cksum ^= (uint8_t) *sz;
        sz++;
    }
    return cksum;
}

void ble_transmit_LK8EX1(float pressure_pa, int32_t cps, float bat_voltage) {
    if(notify_state == true){
        char szmsg[45];
        uint16_t bp = MIN(1100, 1000 + ((bat_voltage-3.5)*100.0/0.65)); // 100% being 4.15V, 0% being 3.5V
        uint8_t len = sprintf(szmsg, "$LK8EX1,%lu,-,%ld,99,%u*", (uint32_t)pressure_pa, cps, bp);
        uint8_t cksum = ble_nmea_checksum(szmsg);
        sprintf(szmsg+len, "%02X\r\n", cksum);

        struct os_mbuf *om = ble_hs_mbuf_from_flat(szmsg, strlen(szmsg));
        int rc = ble_gatts_notify_custom(conn_handle, chr_tx_handle, om);
        if(rc != 0){
            ESP_LOGE(TAG, " error transmitting LK8EX1: %d\n", rc);
        }
    }
}