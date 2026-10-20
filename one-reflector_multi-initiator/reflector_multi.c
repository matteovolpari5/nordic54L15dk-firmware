/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Reflector with Ranging Responder sample
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#define CON_STATUS_LED DK_LED1
#define MAX_CONN       CONFIG_BT_MAX_CONN

static struct bt_conn *connections[MAX_CONN];

/* One semaphore per slot: signals that a new connection needs CS setup */
static struct k_sem sem_config[MAX_CONN];

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int find_slot(struct bt_conn *conn)
{
	for (int i = 0; i < MAX_CONN; i++) {
		if (connections[i] == conn) {
			return i;
		}
	}
	return -1;
}

static int alloc_slot(struct bt_conn *conn)
{
	for (int i = 0; i < MAX_CONN; i++) {
		if (connections[i] == NULL) {
			connections[i] = bt_conn_ref(conn);
			return i;
		}
	}
	return -1;
}

static void restart_advertising(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err && err != -EALREADY) {
		LOG_WRN("Failed to restart advertising (err %d)", err);
	}
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Connection failed to %s (err 0x%02X)", addr, err);
		return;
	}

	int slot = alloc_slot(conn);

	if (slot < 0) {
		LOG_WRN("No free slots, rejecting %s", addr);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	LOG_INF("Connected %s (slot %d)", addr, slot);
	dk_set_led_on(CON_STATUS_LED);

	/* Keep advertising so other initiators can connect */
	restart_advertising();
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	int slot = find_slot(conn);

	if (slot >= 0) {
		bt_conn_unref(connections[slot]);
		connections[slot] = NULL;
		LOG_INF("Disconnected slot %d (reason 0x%02X)", slot, reason);
	}

	int active = 0;

	for (int i = 0; i < MAX_CONN; i++) {
		if (connections[i] != NULL) {
			active++;
		}
	}

	if (active == 0) {
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void remote_capabilities_cb(struct bt_conn *conn,
				    uint8_t status,
				    struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(params);

	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("CS capability exchange failed on slot %d (HCI status 0x%02x)",
			find_slot(conn), status);
	}
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			      struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(config);

	int slot = find_slot(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		if (slot >= 0) {
			k_sem_give(&sem_config[slot]);
		}
	} else {
		LOG_WRN("CS config creation failed on slot %d (HCI status 0x%02x)",
			slot, status);
	}
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	ARG_UNUSED(conn);

	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("CS security enable failed on slot %d (HCI status 0x%02x)",
			find_slot(conn), status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn,
				 uint8_t status,
				 struct bt_conn_le_cs_procedure_enable_complete *params)
{
	if (status == BT_HCI_ERR_SUCCESS) {
		if (params->state == 1) {
			LOG_INF("CS procedures enabled on slot %d — proc_interval %u, "
				"subevent_len %u us",
				find_slot(conn), params->procedure_interval,
				params->subevent_len);
		}
	} else {
		LOG_WRN("CS procedures enable failed on slot %d (HCI status 0x%02x)",
			find_slot(conn), status);
	}
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
};

static int setup_cs_for_slot(int slot)
{
	struct bt_conn *conn = connections[slot];
	int err;

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = false,
		.enable_reflector_role = true,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(conn, &default_settings);
	if (err) {
		LOG_ERR("Failed to configure CS settings on slot %d (err %d)", slot, err);
		return err;
	}

	if (k_sem_take(&sem_config[slot], K_SECONDS(10)) != 0) {
		LOG_ERR("Timeout waiting CS config on slot %d", slot);
		return -ETIMEDOUT;
	}

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = 0,
		.max_procedure_len = 1000,
		.min_procedure_interval = 1,
		.max_procedure_interval = 100,
		.max_procedure_count = 0,
		.min_subevent_len = 10000,
		.max_subevent_len = 75000,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
		.phy = BT_LE_CS_PROCEDURE_PHY_2M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
	};

	err = bt_le_cs_set_procedure_parameters(conn, &procedure_params);
	if (err) {
		LOG_ERR("Failed to set procedure parameters on slot %d (err %d)", slot, err);
		return err;
	}

	LOG_INF("CS setup complete for slot %d", slot);
	return 0;
}

/*
 * Per-slot worker thread: attende che la connessione sia pronta per il setup CS
 * e lo esegue in modo indipendente per ogni initiator.
 */
#define SLOT_STACK_SIZE 2048
#define SLOT_PRIORITY   7

static void slot_thread_fn(void *p1, void *p2, void *p3)
{
	int slot = (int)(intptr_t)p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		while (connections[slot] == NULL) {
			k_sleep(K_MSEC(100));
		}

		setup_cs_for_slot(slot);

		while (connections[slot] != NULL) {
			k_sleep(K_MSEC(200));
		}
	}
}

#define DEFINE_SLOT_THREAD(n)                                                              \
	K_THREAD_STACK_DEFINE(slot_stack_##n, SLOT_STACK_SIZE);                            \
	static struct k_thread slot_thread_data_##n;

DEFINE_SLOT_THREAD(0)
DEFINE_SLOT_THREAD(1)
DEFINE_SLOT_THREAD(2)
DEFINE_SLOT_THREAD(3)

static k_thread_stack_t *slot_stacks[MAX_CONN] = {
	slot_stack_0,
	slot_stack_1,
	slot_stack_2,
	slot_stack_3,
};

static struct k_thread *slot_threads[MAX_CONN] = {
	&slot_thread_data_0,
	&slot_thread_data_1,
	&slot_thread_data_2,
	&slot_thread_data_3,
};

int main(void)
{
	int err;

	LOG_INF("Starting Channel Sounding Reflector Sample");

	dk_leds_init();

	for (int i = 0; i < MAX_CONN; i++) {
		k_sem_init(&sem_config[i], 0, 1);
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	/* Avvia un thread per ogni slot di connessione */
	for (int i = 0; i < MAX_CONN; i++) {
		k_thread_create(slot_threads[i], slot_stacks[i], SLOT_STACK_SIZE,
				slot_thread_fn, (void *)(intptr_t)i, NULL, NULL,
				SLOT_PRIORITY, 0, K_NO_WAIT);
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	LOG_INF("Advertising started, waiting for up to %d initiators", MAX_CONN);

	return 0;
}