#include "sgp30.h"

#include <inttypes.h>  // PRIu64
#include <stdio.h>     // printk
#include <unistd.h>    // sleep
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
// BLE includes
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
// Settings subsystem
#include <zephyr/settings/settings.h>
// ADC for battery monitoring
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(sgp30_demo, CONFIG_LOG_DEFAULT_LEVEL);

/* Define a custom service UUID for SGP30 sensor data */
#define SGP30_SERVICE_UUID BT_UUID_128_ENCODE(0x00001234, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)
#define TVOC_CHAR_UUID    BT_UUID_128_ENCODE(0x00001235, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)
#define CO2EQ_CHAR_UUID   BT_UUID_128_ENCODE(0x00001236, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)
#define BATTERY_CHAR_UUID BT_UUID_128_ENCODE(0x00001237, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)

/* ADC configuration */
//#define ADC_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 10
#define ADC_CHANNEL 0
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_GAIN ADC_GAIN_1_6
#define BATTERY_VOLTAGE_MAX 4200 /* Maximum battery voltage in mV */
#define BATTERY_VOLTAGE_MIN 3000 /* Minimum battery voltage in mV */



static struct bt_uuid_128 sgp30_service_uuid = BT_UUID_INIT_128(SGP30_SERVICE_UUID);
static struct bt_uuid_128 tvoc_char_uuid = BT_UUID_INIT_128(TVOC_CHAR_UUID);
static struct bt_uuid_128 co2eq_char_uuid = BT_UUID_INIT_128(CO2EQ_CHAR_UUID);
static struct bt_uuid_128 battery_char_uuid = BT_UUID_INIT_128(BATTERY_CHAR_UUID);

// ADC device and values
//static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static const struct adc_dt_spec adc_battery = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static int16_t adc_raw_value;

struct adc_sequence batt_sequence = {
	.buffer = &adc_raw_value,
	.buffer_size = sizeof(adc_raw_value),
};

// Declare global variables for characteristic values
static uint16_t tvoc_value;
static uint16_t co2eq_value;
static uint8_t battery_level = 100; // Default battery level value (0-100%)
static struct bt_conn *current_conn;

// Add subscription tracking variables
static bool tvoc_notif_enabled;
static bool co2eq_notif_enabled;
static bool battery_notif_enabled;

// Callback for BT connection events
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

// BT Connection callbacks
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

// Read callbacks for the characteristics
static ssize_t read_tvoc(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const uint16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(tvoc_value));
}

static ssize_t read_co2eq(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const uint16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(co2eq_value));
}

static ssize_t read_battery(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(battery_level));
}

// CCC change callbacks to track subscriptions
static void tvoc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    tvoc_notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("TVOC notifications %s", tvoc_notif_enabled ? "enabled" : "disabled");
}

static void co2eq_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    co2eq_notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("CO2eq notifications %s", co2eq_notif_enabled ? "enabled" : "disabled");
}

static void battery_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    battery_notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Battery notifications %s", battery_notif_enabled ? "enabled" : "disabled");
}

// Define the GATT service
BT_GATT_SERVICE_DEFINE(sgp30_svc,
    BT_GATT_PRIMARY_SERVICE(&sgp30_service_uuid),
    BT_GATT_CHARACTERISTIC(&tvoc_char_uuid.uuid, 
                        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                        BT_GATT_PERM_READ, 
                        read_tvoc, NULL, &tvoc_value),
    BT_GATT_CCC(tvoc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&co2eq_char_uuid.uuid, 
                        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                        BT_GATT_PERM_READ, 
                        read_co2eq, NULL, &co2eq_value),
    BT_GATT_CCC(co2eq_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&battery_char_uuid.uuid, 
                        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                        BT_GATT_PERM_READ, 
                        read_battery, NULL, &battery_level),
    BT_GATT_CCC(battery_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Advertisement data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SGP30_SERVICE_UUID),
};

// Scan response data (additional info)
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

// Initialize BLE
/*static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("BLE init failed (err %d)", err);
        return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        LOG_INF("Loading settings");
        settings_load();
    }

    LOG_INF("BLE initialized");

    // Start advertising with scan response
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising started");
}*/

// Function to update BLE characteristics and notify if connected
static void update_ble_values(uint16_t tvoc, uint16_t co2eq, uint8_t batt_level)
{
    tvoc_value = tvoc;
    co2eq_value = co2eq;
    battery_level = batt_level;

    if (current_conn) {
        // Only notify if the client has subscribed to the characteristic
        if (tvoc_notif_enabled) {
            bt_gatt_notify(current_conn, &sgp30_svc.attrs[2], &tvoc_value, sizeof(tvoc_value));
        }
        
        if (co2eq_notif_enabled) {
            bt_gatt_notify(current_conn, &sgp30_svc.attrs[5], &co2eq_value, sizeof(co2eq_value));
        }
        
        if (battery_notif_enabled) {
            bt_gatt_notify(current_conn, &sgp30_svc.attrs[8], &battery_level, sizeof(battery_level));
        }
    }
}

// Function to read battery voltage and convert to percentage
static uint8_t read_battery_level(void)
{
    int val_mv;
    adc_raw_value = 0;
	int err = adc_read(adc_battery.dev, &batt_sequence);
	if (err < 0) {
		return -1;
	}
    printk("ADC raw value: %d\n", adc_raw_value);

	val_mv = (int)adc_raw_value;
	adc_raw_to_millivolts_dt(&adc_battery, &val_mv);

    uint8_t battery_percentage;

    // Apply any scaling factor needed (depends on your voltage divider if used)
    val_mv *= 5; // Example scaling factor - adjust based on your hardware

    printk("Battery voltage: %d mV\n", val_mv);

    // Convert voltage to percentage based on min/max values
    if (val_mv >= BATTERY_VOLTAGE_MAX) {
        battery_percentage = 100;
    } else if (val_mv <= BATTERY_VOLTAGE_MIN) {
        battery_percentage = 0;
    } else {
        battery_percentage = (uint8_t)(((val_mv - BATTERY_VOLTAGE_MIN) * 100) / 
                             (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN));
    }

    return battery_percentage;
}

int main(void) {
    uint16_t i = 0;
    int16_t err;
    uint16_t tvoc_ppb, co2_eq_ppm;
    uint32_t iaq_baseline;
    uint16_t ethanol_raw_signal, h2_raw_signal;

    // Delay briefly to allow system to stabilize
    k_sleep(K_MSEC(100));
    printk("Starting SGP30 BLE demo\n");

    // Initialize I2C first
    sensirion_i2c_init();
    printk("I2C initialized\n");

    // Initialize settings subsystem
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        err = settings_subsys_init();
        if (err) {
            printk("Settings initialization failed (err %d)\n", err);
            // Continue anyway
        }
    }

    // Initialize BLE with delay to ensure system is ready
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        // Continue with just sensor functionality
    } else {
        printk("BLE initialized\n");
        
        // Register connection callbacks
        bt_conn_cb_register(&conn_callbacks);
        
        // Load settings after BT is initialized
        if (IS_ENABLED(CONFIG_SETTINGS)) {
            settings_load();
        }
        
        // Start advertising with scan response
        err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (err) {
            printk("Advertising failed to start (err %d)\n", err);
            // Continue with just sensor functionality
        } else {
            printk("Advertising started\n");
        }
    }
    
    // SGP30 sensor initialization
    const char* driver_version = sgp30_get_driver_version();
    if (driver_version) {
        printk("SGP30 driver version %s\n", driver_version);
    } else {
        printk("Getting driver version failed, continuing anyway\n");
    }

    // Probe for sensor with timeout
    int16_t probe;
    int retry_count = 0;
    while (retry_count < 5) {
        probe = sgp30_probe();
        printk("probe: %d\n", probe);

        if (probe == STATUS_OK)
            break;

        if (probe == SGP30_ERR_UNSUPPORTED_FEATURE_SET)
            printk("Your sensor needs at least feature set version 1.0 (0x20)\n");

        printk("SGP sensor probing failed, retrying...\n");
        retry_count++;
        k_sleep(K_MSEC(1000));
    }
    
    if (probe != STATUS_OK) {
        printk("Failed to find SGP30 sensor, continuing with BLE only\n");
        // If sensor not found, just keep BLE running
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }
    
    printk("SGP sensor probing successful\n");

    // Rest of the SGP30 initialization
    uint16_t feature_set_version;
    uint8_t product_type;
    err = sgp30_get_feature_set_version(&feature_set_version, &product_type);
    if (err == STATUS_OK) {
        printk("Feature set version: %u\n", feature_set_version);
        printk("Product type: %u\n", product_type);
    }

    uint64_t serial_id;
    err = sgp30_get_serial_id(&serial_id);
    if (err == STATUS_OK) {
        printk("SerialID: %" PRIu64 "\n", serial_id);
    }

    /* Read gas raw signals */
    err = sgp30_measure_raw_blocking_read(&ethanol_raw_signal, &h2_raw_signal);
    if (err == STATUS_OK) {
        printk("Ethanol raw signal: %u\n", ethanol_raw_signal);
        printk("H2 raw signal: %u\n", h2_raw_signal);
    }

    /* Initialize IAQ algorithm */
    err = sgp30_iaq_init();
    if (err == STATUS_OK) {
        printk("sgp30_iaq_init done\n");
    } else {
        printk("sgp30_iaq_init failed!\n");
    }
    if (!adc_is_ready_dt(&adc_battery)) {
		return 0;
	}

	err = adc_channel_setup_dt(&adc_battery);
	if (err < 0) {
		return 0;
	}

	err = adc_sequence_init_dt(&adc_battery, &batt_sequence);
	if (err < 0) {
		return 0;
	}
    printk("ADC initialized\n");

    /* Run periodic IAQ measurements at defined intervals */
    while (1) {
        err = sgp30_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
        if (err == STATUS_OK) {
            printk("tVOC  Concentration: %dppb\n", tvoc_ppb);
            printk("CO2eq Concentration: %dppm\n", co2_eq_ppm);
            /* Read battery level from ADC every minute */
            battery_level = read_battery_level();
            printk("Current battery level: %d%%\n", battery_level);            
            // Update BLE values if BLE is initialized
            if (bt_is_ready()) {
                update_ble_values(tvoc_ppb, co2_eq_ppm, battery_level);
            }
        } else {
            printk("error reading IAQ values\n");
        }

        /* Persist the current baseline every hour */
        if (++i % 3600 == 3599) {
            err = sgp30_get_iaq_baseline(&iaq_baseline);
            if (err == STATUS_OK) {
                /* IMPLEMENT: store baseline to presistent storage */
            }
        }



        k_sleep(K_SECONDS(1));  // SGP30 measurement interval
    }
    return 0;
}
