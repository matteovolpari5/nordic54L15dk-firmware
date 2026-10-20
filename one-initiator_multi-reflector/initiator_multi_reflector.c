#include <math.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/cs_de.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>



LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#define CON_STATUS_LED DK_LED1
#define CS_CONFIG_ID 0

#if defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_2_SUB_MODE_1)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_2)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_1)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_1_NO_SUB_MODE
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_3)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_3_NO_SUB_MODE
#else
BUILD_ASSERT(false, "Invalid ranging mode");
#endif

#define NUM_MODE_0_STEPS       3
#define PROCEDURE_COUNTER_NONE (-1)
#define DE_SLIDING_WINDOW_SIZE (9)
#define MAX_AP		       (CONFIG_BT_RAS_MAX_ANTENNA_PATHS)
#define MAX_REFLECTORS 3

#define LOCAL_PROCEDURE_MEM                                                                        \
	((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) +                \
	 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

#define CHANNEL_INDEX_OFFSET (2)
#define TONE_QI_OK_TONE_COUNT_THRESHOLD (15)

#define MEASUREMENTS_PER_REFLECTOR 1

static K_SEM_DEFINE(sem_connected, 0, MAX_REFLECTORS);

struct cs_measurement_sample {
    cs_de_dist_estimates_t dist;
    float local_rssi;
    float peer_rssi;
};

struct distance_estimate_buffer {
    struct cs_measurement_sample estimates[DE_SLIDING_WINDOW_SIZE];
    uint8_t num_valid;
    uint8_t index;
};

struct reflector_ctx {
    struct bt_conn *conn;
    bt_addr_le_t addr;
    bool in_use;
    bool connected;
    bool configured;
    bool procedure_enabled;
    float latest_local_rssi;
    float latest_peer_rssi;
    uint16_t latest_local_rssi_count;
    uint16_t latest_peer_rssi_count;

    struct k_sem sem_security;
    struct k_sem sem_ras_features;
    struct k_sem sem_mtu_exchange_done;
    struct k_sem sem_discovery_done;
    struct k_sem sem_remote_capabilities_obtained;
    struct k_sem sem_config_created;
    struct k_sem sem_cs_security_enabled;
    struct k_sem sem_local_steps;
    struct k_sem sem_distance_estimate_updated;
    struct k_sem sem_procedure_enable_complete;

    struct bt_gatt_exchange_params mtu_exchange_params;

    struct bt_conn_le_cs_config cs_config;
    uint32_t ras_feature_bits;

    int32_t most_recent_local_ranging_counter;
    int32_t dropped_ranging_counter;

    struct distance_estimate_buffer distance_estimate_buffers[MAX_AP];
    uint16_t n_iqs[CONFIG_BT_RAS_MAX_ANTENNA_PATHS][CS_DE_NUM_CHANNELS];
    cs_de_report_t cs_de_report;

    struct net_buf_simple latest_local_steps;
    struct net_buf_simple latest_peer_steps;
    uint8_t latest_local_steps_data[LOCAL_PROCEDURE_MEM];
    uint8_t latest_peer_steps_data[BT_RAS_PROCEDURE_MEM];
};

static struct reflector_ctx reflectors[MAX_REFLECTORS];
static uint8_t active_reflector_count;
static uint8_t setup_reflector_count;

static bool setup_in_progress;

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params);

static struct reflector_ctx *ctx_from_conn(struct bt_conn *conn)
{
    for (int i = 0; i < MAX_REFLECTORS; i++) {
        if (reflectors[i].in_use && reflectors[i].conn == conn) {
            return &reflectors[i];
        }
    }
    return NULL;
}

static struct reflector_ctx *alloc_reflector_ctx(struct bt_conn *conn)
{
    for (int i = 0; i < MAX_REFLECTORS; i++) {
        if (!reflectors[i].in_use) {
            struct reflector_ctx *ctx = &reflectors[i];
            memset(ctx, 0, sizeof(*ctx));

            ctx->in_use = true;
            ctx->connected = true;
            ctx->conn = bt_conn_ref(conn);
            bt_addr_le_copy(&ctx->addr, bt_conn_get_dst(conn));

            ctx->mtu_exchange_params.func = mtu_exchange_cb;

            k_sem_init(&ctx->sem_security, 0, 1);
            k_sem_init(&ctx->sem_ras_features, 0, 1);
            k_sem_init(&ctx->sem_mtu_exchange_done, 0, 1);
            k_sem_init(&ctx->sem_discovery_done, 0, 1);
            k_sem_init(&ctx->sem_remote_capabilities_obtained, 0, 1);
            k_sem_init(&ctx->sem_config_created, 0, 1);
            k_sem_init(&ctx->sem_cs_security_enabled, 0, 1);
            k_sem_init(&ctx->sem_local_steps, 1, 1);
            k_sem_init(&ctx->sem_distance_estimate_updated, 0, 1);
            k_sem_init(&ctx->sem_procedure_enable_complete, 0, 1);

            ctx->most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
            ctx->dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

            net_buf_simple_init_with_data(&ctx->latest_local_steps,
                              ctx->latest_local_steps_data,
                              sizeof(ctx->latest_local_steps_data));
            net_buf_simple_init_with_data(&ctx->latest_peer_steps,
                              ctx->latest_peer_steps_data,
                              sizeof(ctx->latest_peer_steps_data));
            net_buf_simple_reset(&ctx->latest_local_steps);
            net_buf_simple_reset(&ctx->latest_peer_steps);

            active_reflector_count++;
            return ctx;
        }
    }

    return NULL;
}


static void free_reflector_ctx(struct reflector_ctx *ctx)
{
    if (!ctx || !ctx->in_use) {
        return;
    }

    if (ctx->conn) {
        bt_conn_unref(ctx->conn);
        ctx->conn = NULL;
    }

    memset(ctx, 0, sizeof(*ctx));

    if (active_reflector_count > 0) {
        active_reflector_count--;
    }
}

// DEBUG only function
static void log_ctx_state(const char *tag, struct reflector_ctx *ctx)
{
    if (!ctx) {
        LOG_INF("%s: ctx=NULL", tag);
        return;
    }

    LOG_INF("%s: conn=%p in_use=%d connected=%d configured=%d procedure_enabled=%d "
            "ras_features=0x%08x local_counter=%d dropped_counter=%d "
            "local_steps_len=%u peer_steps_len=%u",
            tag,
            ctx->conn,
            ctx->in_use,
            ctx->connected,
            ctx->configured,
            ctx->procedure_enabled,
            ctx->ras_feature_bits,
            ctx->most_recent_local_ranging_counter,
            ctx->dropped_ranging_counter,
            ctx->latest_local_steps.len,
            ctx->latest_peer_steps.len);
}

#define CONN_INTERVAL_RANGING   0x0010  /* 20ms - tight, for active CS */
#define CONN_INTERVAL_IDLE      0x0140  /* 400ms - sparse, for idle connections */

static int set_conn_interval(struct reflector_ctx *ctx, uint16_t interval)
{
    struct bt_le_conn_param params = {
        .interval_min = interval,
        .interval_max = interval,
        .latency = 0,
        .timeout = BT_GAP_MS_TO_CONN_TIMEOUT(4000),
    };
    int err = bt_conn_le_param_update(ctx->conn, &params);
    if (err) {
        LOG_WRN("conn param update failed for conn=%p err=%d", ctx->conn, err);
    }
    return err;
}

static void store_measurement_in_buffer(struct reflector_ctx *ctx,
                                        uint8_t ap,
                                        cs_de_dist_estimates_t *p_estimates)
{
    struct distance_estimate_buffer *buffer = &ctx->distance_estimate_buffers[ap];
    int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

    __ASSERT_NO_MSG(lock_state == 0);

    buffer->estimates[buffer->index].dist = *p_estimates;

    buffer->estimates[buffer->index].local_rssi =
        (ctx->latest_local_rssi_count > 0) ?
        (ctx->latest_local_rssi / ctx->latest_local_rssi_count) : NAN;

    buffer->estimates[buffer->index].peer_rssi =
        (ctx->latest_peer_rssi_count > 0) ?
        (ctx->latest_peer_rssi / ctx->latest_peer_rssi_count) : NAN;

    buffer->index = (buffer->index + 1) % DE_SLIDING_WINDOW_SIZE;

    if (buffer->num_valid < DE_SLIDING_WINDOW_SIZE) {
        buffer->num_valid++;
    }

    k_mutex_unlock(&distance_estimate_buffer_mutex);
}

static int float_cmp(const void *a, const void *b)
{
	float fa = *(const float *)a;
	float fb = *(const float *)b;

	return (fa > fb) - (fa < fb);
}

static float median_inplace(int count, float *values)
{
	if (count == 0) {
		return NAN;
	}

	qsort(values, count, sizeof(float), float_cmp);

	if (count % 2 == 0) {
		return (values[count/2] + values[count/2 - 1]) / 2;
	} else {
		return values[count/2];
	}
}

static cs_de_dist_estimates_t get_distance_ctx(struct reflector_ctx *ctx, uint8_t ap)
{
	cs_de_dist_estimates_t averaged_result = {};
	uint8_t num_ifft = 0;
	uint8_t num_phase_slope = 0;
	uint8_t num_rtt = 0;

	static float temp_ifft[DE_SLIDING_WINDOW_SIZE];
	static float temp_phase_slope[DE_SLIDING_WINDOW_SIZE];
	static float temp_rtt[DE_SLIDING_WINDOW_SIZE];

    struct distance_estimate_buffer *buffer = &ctx->distance_estimate_buffers[ap];

	int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

	__ASSERT_NO_MSG(lock_state == 0);

	for (uint8_t i = 0; i < buffer->num_valid; i++) {
		if (isfinite(buffer->estimates[i].dist.ifft)) {
			temp_ifft[num_ifft] = buffer->estimates[i].dist.ifft;
			num_ifft++;
		}
		if (isfinite(buffer->estimates[i].dist.phase_slope)) {
			temp_phase_slope[num_phase_slope] = buffer->estimates[i].dist.phase_slope;
			num_phase_slope++;
		}
		if (isfinite(buffer->estimates[i].dist.rtt)) {
			temp_rtt[num_rtt] = buffer->estimates[i].dist.rtt;
			num_rtt++;
		}
	}

	k_mutex_unlock(&distance_estimate_buffer_mutex);

	averaged_result.ifft = median_inplace(num_ifft, temp_ifft);
	averaged_result.phase_slope = median_inplace(num_phase_slope, temp_phase_slope);
	averaged_result.rtt = median_inplace(num_rtt, temp_rtt);

	return averaged_result;
}

static bool m_is_tone_quality_ok(uint16_t num_iqs[CS_DE_NUM_CHANNELS], uint8_t channel_map[10])
{
	uint8_t ok_tones_count = 0;

	for (uint8_t i = 0; i < CS_DE_NUM_CHANNELS; ++i) {
		if (BT_LE_CS_CHANNEL_BIT_GET(channel_map, i + CHANNEL_INDEX_OFFSET) &&
		    num_iqs[i] >= 1) {
			ok_tones_count += 1;
		}
	}
	return (ok_tones_count >= TONE_QI_OK_TONE_COUNT_THRESHOLD);
}

static void cumulate_mean(float *avg, float new_value, uint16_t *N)
{
	float a = 1.0f / (*N);
	float b = 1.0f - a;
	*avg = a * new_value + b * (*avg);
}

static void extract_rtt_timings(cs_de_report_t *p_report,
                                struct reflector_ctx *ctx,
                                struct bt_hci_le_cs_step_data_mode_1 *local_rtt_data,
                                struct bt_hci_le_cs_step_data_mode_1 *peer_rtt_data)
{
    if (local_rtt_data->packet_quality_aa_check !=
            BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
        local_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
        local_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE ||
        peer_rtt_data->packet_quality_aa_check !=
            BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
        peer_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
        peer_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE) {
        return;
    }

    if (p_report->role == BT_CONN_LE_CS_ROLE_INITIATOR) {
        p_report->rtt_accumulated_half_ns +=
            local_rtt_data->toa_tod_initiator - peer_rtt_data->tod_toa_reflector;
    } else {
        p_report->rtt_accumulated_half_ns +=
            peer_rtt_data->toa_tod_initiator - local_rtt_data->tod_toa_reflector;
    }

    p_report->rtt_count++;

    ctx->latest_local_rssi += (float)(int8_t)local_rtt_data->packet_rssi;
    ctx->latest_local_rssi_count++;

    ctx->latest_peer_rssi += (float)(int8_t)peer_rtt_data->packet_rssi;
    ctx->latest_peer_rssi_count++;
}

struct parse_user_data {
    struct reflector_ctx *ctx;
    cs_de_report_t *report;
};

static bool process_ranging_header(struct ras_ranging_header *ranging_header, void *user_data)
{
    struct parse_user_data *ud = user_data;
    cs_de_report_t *p_report = ud->report;

    p_report->n_ap = MAX(1, ((ranging_header->antenna_paths_mask & BIT(0)) +
                 ((ranging_header->antenna_paths_mask & BIT(1)) >> 1) +
                 ((ranging_header->antenna_paths_mask & BIT(2)) >> 2) +
                 ((ranging_header->antenna_paths_mask & BIT(3)) >> 3)));
    return true;
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
                  struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
    struct parse_user_data *ud = user_data;
    struct reflector_ctx *ctx = ud->ctx;
    cs_de_report_t *p_report = ud->report;

    if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
        struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
            (struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
        struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
            (struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

        for (uint8_t tone_index = 0; tone_index < p_report->n_ap; tone_index++) {
            int antenna_path = bt_le_cs_get_antenna_path(
                p_report->n_ap,
                local_step_data->antenna_permutation_index,
                tone_index);

            if (antenna_path < 0) {
                LOG_WRN("Invalid antenna path.");
                return true;
            }

            if (local_step_data->tone_info[tone_index].quality_indicator !=
                    BT_HCI_LE_CS_TONE_QUALITY_HIGH ||
                peer_step_data->tone_info[tone_index].quality_indicator !=
                    BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
                return true;
            }

            struct bt_le_cs_iq_sample local_iq =
                bt_le_cs_parse_pct(local_step_data->tone_info[tone_index].phase_correction_term);
            struct bt_le_cs_iq_sample remote_iq =
                bt_le_cs_parse_pct(peer_step_data->tone_info[tone_index].phase_correction_term);

            uint8_t ch = local_step->channel - CHANNEL_INDEX_OFFSET;
            ctx->n_iqs[antenna_path][ch]++;

            if (ctx->n_iqs[antenna_path][ch] == 1) {
                p_report->iq_tones[antenna_path].i_local[ch] = local_iq.i;
                p_report->iq_tones[antenna_path].q_local[ch] = local_iq.q;
                p_report->iq_tones[antenna_path].i_remote[ch] = remote_iq.i;
                p_report->iq_tones[antenna_path].q_remote[ch] = remote_iq.q;
            } else {
                cumulate_mean(&p_report->iq_tones[antenna_path].i_local[ch],
                          local_iq.i, &ctx->n_iqs[antenna_path][ch]);
                cumulate_mean(&p_report->iq_tones[antenna_path].q_local[ch],
                          local_iq.q, &ctx->n_iqs[antenna_path][ch]);
                cumulate_mean(&p_report->iq_tones[antenna_path].i_remote[ch],
                          remote_iq.i, &ctx->n_iqs[antenna_path][ch]);
                cumulate_mean(&p_report->iq_tones[antenna_path].q_remote[ch],
                          remote_iq.q, &ctx->n_iqs[antenna_path][ch]);
            }
        }
    }

    if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_1 ||
        local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
        struct bt_hci_le_cs_step_data_mode_1 *local_rtt =
            (struct bt_hci_le_cs_step_data_mode_1 *)local_step->data;
        struct bt_hci_le_cs_step_data_mode_1 *peer_rtt =
            (struct bt_hci_le_cs_step_data_mode_1 *)peer_step->data;

        extract_rtt_timings(p_report, ctx, local_rtt, peer_rtt);
    }

    return true;
}

static void ranging_data_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
    struct reflector_ctx *ctx = ctx_from_conn(conn);

    if (!ctx) {
        return;
    }
    /* LOG_INF("ranging_data_cb: conn=%p ctx=%p counter=%u err=%d local_recent=%d "
        "local_steps_len=%u peer_steps_len=%u",
        conn,
        ctx,
        ranging_counter,
        err,
        ctx->most_recent_local_ranging_counter,
        ctx->latest_local_steps.len,
        ctx->latest_peer_steps.len); */

    //("ranging_data_cb_entry", ctx);

    if (err) {
        LOG_ERR("Error when receiving ranging data with ranging counter %d (err %d)",
            ranging_counter, err);
        return;
    }

    if (ranging_counter != ctx->most_recent_local_ranging_counter) {
       /*  LOG_INF("Ranging data dropped as peer ranging counter doesn't match local ranging "
            "data counter. (peer: %u, local: %u)",
            ranging_counter, ctx->most_recent_local_ranging_counter); */
        net_buf_simple_reset(&ctx->latest_local_steps);
        k_sem_give(&ctx->sem_local_steps);
        return;
    }

    if (ctx->latest_local_steps.len == 0) {
        LOG_WRN("No local step data available for counter=%u; peer data callback arrived but local len is 0",
        ctx->most_recent_local_ranging_counter);
        LOG_WRN("All subevents in ranging counter %u were aborted",
            ctx->most_recent_local_ranging_counter);
        net_buf_simple_reset(&ctx->latest_local_steps);
        k_sem_give(&ctx->sem_local_steps);

        if (!(ctx->ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
            net_buf_simple_reset(&ctx->latest_peer_steps);
        }
        return;
    }

    memset(&ctx->cs_de_report, 0x0, sizeof(cs_de_report_t));
    memset(ctx->n_iqs, 0, sizeof(ctx->n_iqs));


	struct parse_user_data ud = {
		.ctx = ctx,
		.report = &ctx->cs_de_report,
	};

    ctx->latest_local_rssi = 0.0f;
    ctx->latest_peer_rssi = 0.0f;
    ctx->latest_local_rssi_count = 0;
    ctx->latest_peer_rssi_count = 0;

    bt_ras_rreq_rd_subevent_data_parse(&ctx->latest_peer_steps,
                   &ctx->latest_local_steps,
                   ctx->cs_config.role,
                   process_ranging_header, NULL,
                   process_step_data,
                   &ud);

    for (uint8_t ap = 0; ap < ctx->cs_de_report.n_ap; ap++) {
        ctx->cs_de_report.distance_estimates[ap].ifft = NAN;
        ctx->cs_de_report.distance_estimates[ap].phase_slope = NAN;
        ctx->cs_de_report.distance_estimates[ap].rtt = NAN;
        ctx->cs_de_report.distance_estimates[ap].best = NAN;

        if (m_is_tone_quality_ok(ctx->n_iqs[ap], ctx->cs_config.channel_map)) {
            ctx->cs_de_report.tone_quality[ap] = CS_DE_TONE_QUALITY_OK;
        } else {
            ctx->cs_de_report.tone_quality[ap] = CS_DE_TONE_QUALITY_BAD;
        }
    }

    net_buf_simple_reset(&ctx->latest_local_steps);

    if (!(ctx->ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
        net_buf_simple_reset(&ctx->latest_peer_steps);
    }

    k_sem_give(&ctx->sem_local_steps);

    /* LOG_INF("Parsed ranging data: n_ap=%u rtt_count=%u",
        ctx->cs_de_report.n_ap,
        ctx->cs_de_report.rtt_count); */
    cs_de_quality_t quality = cs_de_calc(&ctx->cs_de_report);

    if (quality == CS_DE_QUALITY_OK) {
        for (uint8_t ap = 0; ap < ctx->cs_de_report.n_ap; ap++) {
            if (ctx->cs_de_report.tone_quality[ap] == CS_DE_TONE_QUALITY_OK ||
                isfinite(ctx->cs_de_report.distance_estimates[ap].rtt)) {
                store_measurement_in_buffer(ctx, ap, &ctx->cs_de_report.distance_estimates[ap]);
            }
            /* LOG_INF("Stored distance estimate: ap=%u ifft=%f phase_slope=%f rtt=%f tone_quality=%u",
            ap,
            (double)ctx->cs_de_report.distance_estimates[ap].ifft,
            (double)ctx->cs_de_report.distance_estimates[ap].phase_slope,
            (double)ctx->cs_de_report.distance_estimates[ap].rtt,
            ctx->cs_de_report.tone_quality[ap]); */
        }
        k_sem_give(&ctx->sem_distance_estimate_updated);
    }
}

// DEBUG only function
static const char *cs_abort_reason_to_str(uint8_t reason, bool procedure)
{
    switch (reason) {
    case 0x0:
        return "no abort";
    case 0x1:
        return "local host or remote request";
    case 0x2:
        return procedure ? "filtered channel map < 15 channels"
                         : "no CS_SYNC received";
    case 0x3:
        return procedure ? "channel map update instant passed"
                         : "scheduling conflicts or limited resources";
    case 0xF:
        return "unspecified reasons";
    default:
        return "reserved/unknown";
    }
}

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
    struct reflector_ctx *ctx = ctx_from_conn(conn);

    if (!ctx) {
        return;
    }
    uint8_t subevent_status = result->header.subevent_done_status;
    uint8_t procedure_status = result->header.procedure_done_status;
    uint8_t subevent_abort_reason = (subevent_status >> 4) & 0x0F;
    uint8_t procedure_abort_reason = procedure_status & 0x0F;

    /* LOG_INF("subevent_result_cb: proc_cnt=%u subevent_status=0x%02x procedure_status=0x%02x "
            "subevent_abort_reason=0x%x (%s) procedure_abort_reason=0x%x (%s)",
            result->header.procedure_counter,
            subevent_status,
            procedure_status,
            subevent_abort_reason,
            cs_abort_reason_to_str(subevent_abort_reason, false),
            procedure_abort_reason,
            cs_abort_reason_to_str(procedure_abort_reason, true)); */

    //log_ctx_state("subevent_result_cb_entry", ctx);

    if (ctx->dropped_ranging_counter == result->header.procedure_counter) {
        return;
    }

    if (ctx->most_recent_local_ranging_counter !=
        bt_ras_rreq_get_ranging_counter(result->header.procedure_counter)) {
        int sem_state = k_sem_take(&ctx->sem_local_steps, K_NO_WAIT);

        if (sem_state < 0) {
            ctx->dropped_ranging_counter = result->header.procedure_counter;
            LOG_INF("Dropped subevent results. Waiting for ranging data from peer.");
            LOG_WRN("Dropped subevent results: proc_cnt=%u because sem_local_steps unavailable",
                result->header.procedure_counter);
            return;
        }

        ctx->most_recent_local_ranging_counter =
            bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
    }

    if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
    } else if (result->step_data_buf) {
        if (result->step_data_buf->len <= net_buf_simple_tailroom(&ctx->latest_local_steps)) {
            uint16_t len = result->step_data_buf->len;
            uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

            net_buf_simple_add_mem(&ctx->latest_local_steps, step_data, len);
        } else {
            LOG_ERR("Not enough memory to store step data. (%d > %d)",
                ctx->latest_local_steps.len + result->step_data_buf->len,
                ctx->latest_local_steps.size);
            net_buf_simple_reset(&ctx->latest_local_steps);
            ctx->dropped_ranging_counter = result->header.procedure_counter;
            return;
        }
    }

    ctx->dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

    if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
        ctx->most_recent_local_ranging_counter =
            bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
    } else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
        //LOG_WRN("Procedure %u aborted", result->header.procedure_counter);
        LOG_WRN("Procedure aborted: proc_cnt=%u subevent_done_status=%u procedure_done_status=%u "
        "local_steps_len=%u peer_steps_len=%u",
        result->header.procedure_counter,
        result->header.subevent_done_status,
        result->header.procedure_done_status,
        ctx->latest_local_steps.len,
        ctx->latest_peer_steps.len);
        //log_ctx_state("procedure_aborted", ctx);
        net_buf_simple_reset(&ctx->latest_local_steps);
        k_sem_give(&ctx->sem_local_steps);
    }
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
    struct reflector_ctx *ctx = ctx_from_conn(conn);

    if (!ctx) {
        return;
    }
    LOG_INF("ranging_data_ready_cb: conn=%p ctx=%p ranging_counter=%u local_recent=%d",
        conn, ctx, ranging_counter, ctx->most_recent_local_ranging_counter);
    //log_ctx_state("ranging_data_ready_cb", ctx);
    LOG_DBG("Ranging data ready %i", ranging_counter);

    if (ranging_counter == ctx->most_recent_local_ranging_counter) {
        LOG_INF("Requesting peer ranging data for counter=%u", ranging_counter);
        int err = bt_ras_rreq_cp_get_ranging_data(ctx->conn,
                              &ctx->latest_peer_steps,
                              ranging_counter,
                              ranging_data_cb);
        if (err) {
            LOG_ERR("Get ranging data failed (err %d)", err);
            LOG_WRN("ranging_data_ready ignored: peer counter=%u local_recent=%d",
            ranging_counter, ctx->most_recent_local_ranging_counter);
            net_buf_simple_reset(&ctx->latest_local_steps);
            net_buf_simple_reset(&ctx->latest_peer_steps);
            k_sem_give(&ctx->sem_local_steps);
        }
    }
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_INF("Ranging data overwritten %i", ranging_counter);
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_ERR("MTU exchange failed (err %d)", err);
		return;
	}
	struct reflector_ctx *ctx = ctx_from_conn(conn);
	LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
	if (ctx) {
		k_sem_give(&ctx->sem_mtu_exchange_done);
	}
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	LOG_INF("The discovery procedure succeeded");

	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);
	struct reflector_ctx *ctx = ctx_from_conn(conn);

	bt_gatt_dm_data_print(dm);

	err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
	if (err) {
		LOG_ERR("RAS RREQ alloc init failed (err %d)", err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Could not release the discovery data (err %d)", err);
	}

	if (ctx) {
		k_sem_give(&ctx->sem_discovery_done);
	}
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_INF("The service could not be found during the discovery, disconnecting");
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_INF("The discovery procedure failed (err %d)", err);
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
		return;
	}

	LOG_INF("Security changed: %s level %u", addr, level);
	struct reflector_ctx *ctx = ctx_from_conn(conn);
	if (ctx) {
		k_sem_give(&ctx->sem_security);
	}
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	/* Ignore peer parameter preferences. */
	return false;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("CONNECTED_CB addr=%s err=0x%02X", addr, err);

    if (err) {
        LOG_ERR("Connection failed for %s", addr);
        bt_conn_unref(conn);
        return;
    }

    struct reflector_ctx *ctx = alloc_reflector_ctx(conn);
    if (!ctx) {
        LOG_WRN("No free reflector slots, disconnecting %s", addr);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    LOG_INF("Stored %s in slot, active=%u/%u",
            addr, active_reflector_count, MAX_REFLECTORS);

    if (!setup_in_progress && active_reflector_count < MAX_REFLECTORS) {
        int scan_err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
        if (scan_err) {
            LOG_ERR("Failed to restart scan (err %d)", scan_err);
        } else {
            LOG_INF("Restarted scan for next reflector");
        }
    }

    dk_set_led_on(CON_STATUS_LED);
    k_sem_give(&sem_connected);
}


static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    struct reflector_ctx *ctx = ctx_from_conn(conn);

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected %s (reason 0x%02X)", addr, reason);

    if (ctx) {
        free_reflector_ctx(ctx);
    }

    if (active_reflector_count == 0) {
        dk_set_led_off(CON_STATUS_LED);
    }
}

static void remote_capabilities_cb(struct bt_conn *conn,
				   uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	struct reflector_ctx *ctx = ctx_from_conn(conn);
	if (!ctx) {
		return;
	}

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS capability exchange completed.");
		k_sem_give(&ctx->sem_remote_capabilities_obtained);
	} else {
		LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
	}
}

static const char *cs_mode_to_str(uint8_t mode)
{
    switch (mode) {
    case BT_CONN_LE_CS_MAIN_MODE_1_NO_SUB_MODE:
        return "Main mode 1";
    case BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE:
        return "Main mode 2";
    case BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1:
        return "Main mode 2 + submode 1";
    case BT_CONN_LE_CS_MAIN_MODE_3_NO_SUB_MODE:
        return "Main mode 3";
    default:
        return "Unknown";
    }
}

static const char *cs_role_to_str(uint8_t role)
{
    switch (role) {
    case BT_CONN_LE_CS_ROLE_INITIATOR:
        return "Initiator";
    case BT_CONN_LE_CS_ROLE_REFLECTOR:
        return "Reflector";
    default:
        return "Unknown";
    }
}

static const char *cs_rtt_type_to_str(uint8_t rtt_type)
{
    switch (rtt_type) {
    case BT_CONN_LE_CS_RTT_TYPE_AA_ONLY:
        return "AA only";
    case BT_CONN_LE_CS_RTT_TYPE_32_BIT_SOUNDING:
        return "32-bit sounding";
    case BT_CONN_LE_CS_RTT_TYPE_96_BIT_SOUNDING:
        return "96-bit sounding";
    case BT_CONN_LE_CS_RTT_TYPE_32_BIT_RANDOM:
        return "32-bit random";
    case BT_CONN_LE_CS_RTT_TYPE_64_BIT_RANDOM:
        return "64-bit random";
    case BT_CONN_LE_CS_RTT_TYPE_96_BIT_RANDOM:
        return "96-bit random";
    case BT_CONN_LE_CS_RTT_TYPE_128_BIT_RANDOM:
        return "128-bit random";
    default:
        return "Unknown";
    }
}

static const char *cs_phy_to_str(uint8_t phy)
{
    switch (phy) {
    case BT_CONN_LE_CS_SYNC_1M_PHY:
        return "LE 1M PHY";
    case BT_CONN_LE_CS_SYNC_2M_PHY:
        return "LE 2M PHY";
    case BT_CONN_LE_CS_SYNC_2M_2BT_PHY:
        return "LE 2M 2BT PHY";
    default:
        return "Unknown";
    }
}

static const char *cs_chsel_to_str(uint8_t type)
{
    switch (type) {
    case BT_CONN_LE_CS_CHSEL_TYPE_3B:
        return "Algorithm #3b";
    case BT_CONN_LE_CS_CHSEL_TYPE_3C:
        return "Algorithm #3c";
    default:
        return "Unknown";
    }
}

static const char *cs_ch3c_shape_to_str(uint8_t shape)
{
    switch (shape) {
    case BT_CONN_LE_CS_CH3C_SHAPE_HAT:
        return "Hat shape";
    case BT_CONN_LE_CS_CH3C_SHAPE_X:
        return "X shape";
    default:
        return "Unknown";
    }
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
                 struct bt_conn_le_cs_config *config)
{
    struct reflector_ctx *ctx = ctx_from_conn(conn);

    if (!ctx) {
        return;
    }

    if (status == BT_HCI_ERR_SUCCESS) {
        ctx->cs_config = *config;

        LOG_INF("CS config creation complete.");
        LOG_INF(" - id: %u", ctx->cs_config.id);
        LOG_INF(" - mode: %s (raw=%u)",
            cs_mode_to_str(ctx->cs_config.mode), ctx->cs_config.mode);
        LOG_INF(" - min_main_mode_steps: %u", ctx->cs_config.min_main_mode_steps);
        LOG_INF(" - max_main_mode_steps: %u", ctx->cs_config.max_main_mode_steps);
        LOG_INF(" - main_mode_repetition: %u", ctx->cs_config.main_mode_repetition);
        LOG_INF(" - mode_0_steps: %u", ctx->cs_config.mode_0_steps);
        LOG_INF(" - role: %s (raw=%u)",
            cs_role_to_str(ctx->cs_config.role), ctx->cs_config.role);
        LOG_INF(" - rtt_type: %s (raw=%u)",
            cs_rtt_type_to_str(ctx->cs_config.rtt_type), ctx->cs_config.rtt_type);
        LOG_INF(" - cs_sync_phy: %s (raw=%u)",
            cs_phy_to_str(ctx->cs_config.cs_sync_phy), ctx->cs_config.cs_sync_phy);
        LOG_INF(" - channel_map_repetition: %u", ctx->cs_config.channel_map_repetition);
        LOG_INF(" - channel_selection_type: %s (raw=%u)",
            cs_chsel_to_str(ctx->cs_config.channel_selection_type),
            ctx->cs_config.channel_selection_type);
        LOG_INF(" - ch3c_shape: %s (raw=%u)",
            cs_ch3c_shape_to_str(ctx->cs_config.ch3c_shape),
            ctx->cs_config.ch3c_shape);
        LOG_INF(" - ch3c_jump: %u", ctx->cs_config.ch3c_jump);
        LOG_INF(" - t_ip1_time_us: %u", ctx->cs_config.t_ip1_time_us);
        LOG_INF(" - t_ip2_time_us: %u", ctx->cs_config.t_ip2_time_us);
        LOG_INF(" - t_fcs_time_us: %u", ctx->cs_config.t_fcs_time_us);
        LOG_INF(" - t_pm_time_us: %u", ctx->cs_config.t_pm_time_us);
        LOG_INF(" - channel_map: 0x%08X%08X%04X",
            sys_get_le32(&ctx->cs_config.channel_map[6]),
            sys_get_le32(&ctx->cs_config.channel_map[2]),
            sys_get_le16(&ctx->cs_config.channel_map[0]));

        k_sem_give(&ctx->sem_config_created);
    } else {
        LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
    }
}

static int set_security_with_retry(struct reflector_ctx *ctx)
{
    int err;

    for (int attempt = 0; attempt < 5; attempt++) {
        while (k_sem_take(&ctx->sem_security, K_NO_WAIT) == 0) {
        }

        err = bt_conn_set_security(ctx->conn, BT_SECURITY_L2);
        if (!err) {
            LOG_INF("Security request started for %p on attempt %d", ctx->conn, attempt + 1);

            if (k_sem_take(&ctx->sem_security, K_SECONDS(15)) == 0) {
                k_sleep(K_SECONDS(1));
                return 0;
            }

            LOG_WRN("Timeout waiting security_changed on attempt %d", attempt + 1);
        } else if (err == -12) {
            LOG_WRN("bt_conn_set_security busy on %p, retry attempt %d", ctx->conn, attempt + 1);
        } else {
            LOG_ERR("bt_conn_set_security failed on %p (err %d)", ctx->conn, err);
            return err;
        }

        k_sleep(K_SECONDS(2 + attempt));
    }

    return -EBUSY;
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS security enabled.");
		struct reflector_ctx *ctx = ctx_from_conn(conn);
		if (ctx && status == BT_HCI_ERR_SUCCESS) {
			k_sem_give(&ctx->sem_cs_security_enabled);
		}
	} else {
		LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn,
				uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	struct reflector_ctx *ctx = ctx_from_conn(conn);

    /* LOG_INF("procedure_enable_cb: conn=%p status=0x%02x state=%u config_id=%u "
        "tone_ant_cfg=%u tx_power=%d subevent_len=%u subevents_per_event=%u "
        "subevent_interval=%u event_interval=%u procedure_interval=%u "
        "procedure_count=%u max_procedure_len=%u",
        conn,
        status,
        params ? params->state : 0,
        params ? params->config_id : 0,
        params ? params->tone_antenna_config_selection : 0,
        params ? params->selected_tx_power : 0,
        params ? params->subevent_len : 0,
        params ? params->subevents_per_event : 0,
        params ? params->subevent_interval : 0,
        params ? params->event_interval : 0,
        params ? params->procedure_interval : 0,
        params ? params->procedure_count : 0,
        params ? params->max_procedure_len : 0); */

    //log_ctx_state("procedure_enable_cb", ctx);

	if (!ctx) {
		return;
	}

	if (status == BT_HCI_ERR_SUCCESS) {
		ctx->procedure_enabled = (params->state == 1);

		if (params->state == 1) {
			LOG_INF("CS procedures enabled:\n"
				" - config ID: %u\n"
				" - antenna configuration index: %u\n"
				" - TX power: %d dbm\n"
				" - subevent length: %u us\n"
				" - subevents per event: %u\n"
				" - subevent interval: %u\n"
				" - event interval: %u\n"
				" - procedure interval: %u\n"
				" - procedure count: %u\n"
				" - maximum procedure length: %u",
				params->config_id, params->tone_antenna_config_selection,
				params->selected_tx_power, params->subevent_len,
				params->subevents_per_event, params->subevent_interval,
				params->event_interval, params->procedure_interval,
				params->procedure_count, params->max_procedure_len);
		} else {
			LOG_INF("CS procedures disabled.");
		}
	} else {
		LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
	}
    k_sem_give(&ctx->sem_procedure_enable_complete);
}

void ras_features_read_cb(struct bt_conn *conn, uint32_t feature_bits, int err)
{
    struct reflector_ctx *ctx = ctx_from_conn(conn);

    if (!ctx) {
        return;
    }

    if (err) {
        LOG_WRN("Error while reading RAS feature bits (err %d)", err);
    } else {
        LOG_INF("Read RAS feature bits: 0x%x", feature_bits);
        ctx->ras_feature_bits = feature_bits;
    }

    k_sem_give(&ctx->sem_ras_features);
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match,
                              bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("SCAN MATCH addr=%s connectable=%d rssi=%d active=%u",
            addr, connectable, device_info->recv_info->rssi, active_reflector_count);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    int err;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
    LOG_ERR("CONNECTING ERROR to %s, restarting scan", addr);


    err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
    if (err) {
        LOG_ERR("Failed to restart scanning (err %i)", err);
    }
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
    LOG_INF("CONNECTING to %s", addr);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int scan_init(struct bt_scan_init_param *p_param)
{
	int err;

	bt_scan_init(p_param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	return 0;
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_param_req = le_param_req,
	.security_changed = security_changed,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

static void cs_config_get(struct bt_le_cs_create_config_params *config_params)
{
	config_params->id = CS_CONFIG_ID;
	config_params->mode = CS_CONFIG_MODE;
	config_params->min_main_mode_steps = 2;
	config_params->max_main_mode_steps = 5;
	config_params->main_mode_repetition = 0;
	config_params->mode_0_steps = NUM_MODE_0_STEPS;
	config_params->role = BT_CONN_LE_CS_ROLE_INITIATOR;
	config_params->rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY;
	config_params->cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY;
	config_params->channel_map_repetition = 1;
	config_params->channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B;
	config_params->ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT;
	config_params->ch3c_jump = 2;
}

static void distance_estimates_print(struct reflector_ctx *ctx, uint8_t reflector_idx, uint8_t ap)
{
    struct distance_estimate_buffer *buffer = &ctx->distance_estimate_buffers[ap];
    cs_de_dist_estimates_t distance_on_ap = get_distance_ctx(ctx, ap);

    float local_rssi = NAN;
    float peer_rssi = NAN;

    int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);
    __ASSERT_NO_MSG(lock_state == 0);

    if (buffer->num_valid > 0) {
        uint8_t last = (buffer->index + DE_SLIDING_WINDOW_SIZE - 1) % DE_SLIDING_WINDOW_SIZE;
        local_rssi = buffer->estimates[last].local_rssi;
        peer_rssi = buffer->estimates[last].peer_rssi;
    }

    k_mutex_unlock(&distance_estimate_buffer_mutex);

    LOG_INF("Reflector %u AP %u ifft: %.2f, phase_slope: %.2f, rtt: %.2f meters, local_rssi: %.1f dBm, peer_rssi: %.1f dBm",
            reflector_idx, ap,
            (double)distance_on_ap.ifft,
            (double)distance_on_ap.phase_slope,
            (double)distance_on_ap.rtt,
            (double)local_rssi,
            (double)peer_rssi);
}

// setup_reflector performs the necessary steps to set up a connection as a reflector ready for ranging.
static int setup_reflector(struct reflector_ctx *ctx,
               const struct bt_scan_init_param *scan_params)
{
    int err;

    LOG_INF("setup_reflector start conn=%p", ctx->conn);

    k_sleep(K_MSEC(500));

    err = set_security_with_retry(ctx);
    if (err) {
        LOG_ERR("Failed to secure connection %p (err %d)", ctx->conn, err);
        return err;
    }

    err = bt_gatt_exchange_mtu(ctx->conn, &ctx->mtu_exchange_params);
    if (err) {
        LOG_ERR("MTU exchange start failed (err %d)", err);
        return err;
    }

    if (k_sem_take(&ctx->sem_mtu_exchange_done, K_SECONDS(10)) != 0) {
        LOG_ERR("Timeout waiting MTU exchange");
        return -ETIMEDOUT;
    }

    k_sleep(K_MSEC(300));

    err = bt_gatt_dm_start(ctx->conn, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
    if (err) {
        LOG_ERR("Discovery failed (err %d)", err);
        return err;
    }

    if (k_sem_take(&ctx->sem_discovery_done, K_SECONDS(10)) != 0) {
        LOG_ERR("Timeout waiting discovery");
        return -ETIMEDOUT;
    }

    k_sleep(K_MSEC(300));

    const struct bt_le_cs_set_default_settings_param default_settings = {
        .enable_initiator_role = true,
        .enable_reflector_role = false,
        .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
        .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
    };

    err = bt_le_cs_set_default_settings(ctx->conn, &default_settings);
    if (err) {
        LOG_ERR("Failed to configure default CS settings (err %d)", err);
        return err;
    }

    err = bt_ras_rreq_read_features(ctx->conn, ras_features_read_cb);
    if (err) {
        LOG_ERR("Could not get RAS features from peer (err %d)", err);
        return err;
    }

    if (k_sem_take(&ctx->sem_ras_features, K_SECONDS(10)) != 0) {
        LOG_ERR("Timeout waiting RAS features");
        return -ETIMEDOUT;
    }

    bool realtime_rd = ctx->ras_feature_bits & RAS_FEAT_REALTIME_RD;

    if (realtime_rd) {
        err = bt_ras_rreq_realtime_rd_subscribe(ctx->conn,
                            &ctx->latest_peer_steps,
                            ranging_data_cb);
        if (err) {
            LOG_ERR("RAS realtime ranging subscribe failed (err %d)", err);
            return err;
        }
    } else {
        err = bt_ras_rreq_rd_overwritten_subscribe(ctx->conn, ranging_data_overwritten_cb);
        if (err) {
            LOG_ERR("RD overwritten subscribe failed (err %d)", err);
            return err;
        }

        err = bt_ras_rreq_rd_ready_subscribe(ctx->conn, ranging_data_ready_cb);
        if (err) {
            LOG_ERR("RD ready subscribe failed (err %d)", err);
            return err;
        }

        err = bt_ras_rreq_on_demand_rd_subscribe(ctx->conn);
        if (err) {
            LOG_ERR("On-demand RD subscribe failed (err %d)", err);
            return err;
        }

        err = bt_ras_rreq_cp_subscribe(ctx->conn);
        if (err) {
            LOG_ERR("CP subscribe failed (err %d)", err);
            return err;
        }
    }

    err = bt_le_cs_read_remote_supported_capabilities(ctx->conn);
    if (err) {
        LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
        return err;
    }

    if (k_sem_take(&ctx->sem_remote_capabilities_obtained, K_SECONDS(10)) != 0) {
        LOG_ERR("Timeout waiting remote capabilities");
        return -ETIMEDOUT;
    }

    struct bt_le_cs_create_config_params config_params = {0};
    cs_config_get(&config_params);
    bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

    err = bt_le_cs_create_config(ctx->conn, &config_params,
                     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
    if (err) {
        LOG_ERR("Failed to create CS config (err %d)", err);
        return err;
    }

    if (k_sem_take(&ctx->sem_config_created, K_SECONDS(10)) != 0) {
        LOG_ERR("Timeout waiting config created");
        return -ETIMEDOUT;
    }

    err = bt_le_cs_security_enable(ctx->conn);
    if (err) {
        LOG_ERR("Failed to start CS Security (err %d)", err);
        return err;
    }

    if (k_sem_take(&ctx->sem_cs_security_enabled, K_SECONDS(10)) != 0) {
        LOG_ERR("Timeout waiting CS security enabled");
        return -ETIMEDOUT;
    }

    //todo check real value meaning
    const uint16_t acl_interval_in_proc_interval_units =
        scan_params->conn_param->interval_max * 2;
    uint16_t desired_procedure_interval = realtime_rd ? 5 : 10;
    uint16_t desired_max_procedure_length =
        acl_interval_in_proc_interval_units * (desired_procedure_interval - 1);

    const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
        .config_id = 0,
        .max_procedure_len = 1000,
        .min_procedure_interval = 10,
        .max_procedure_interval = 100,
        .max_procedure_count = 0,
        .min_subevent_len = 20000,
        .max_subevent_len = 75000,
        .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
        .phy = BT_LE_CS_PROCEDURE_PHY_2M,
        .tx_power_delta = 0x80,
        .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
        .snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
        .snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
    };

    LOG_INF("Applying procedure params: conn=%p config_id=%u max_procedure_len=%u "
        "min_procedure_interval=%u max_procedure_interval=%u "
        "max_procedure_count=%u min_subevent_len=%u max_subevent_len=%u "
        "tone_cfg=%u phy=%u tx_power_delta=%d preferred_peer_antenna=%u "
        "snr_init=%u snr_refl=%u realtime_rd=%d",
        ctx->conn,
        procedure_params.config_id,
        procedure_params.max_procedure_len,
        procedure_params.min_procedure_interval,
        procedure_params.max_procedure_interval,
        procedure_params.max_procedure_count,
        procedure_params.min_subevent_len,
        procedure_params.max_subevent_len,
        procedure_params.tone_antenna_config_selection,
        procedure_params.phy,
        procedure_params.tx_power_delta,
        procedure_params.preferred_peer_antenna,
        procedure_params.snr_control_initiator,
        procedure_params.snr_control_reflector,
        realtime_rd);

    //log_ctx_state("before_set_procedure_params", ctx);


    err = bt_le_cs_set_procedure_parameters(ctx->conn, &procedure_params);
    if (err) {
        LOG_ERR("Failed to set procedure parameters (err %d)", err);
        return err;
    }
    LOG_INF("bt_le_cs_set_procedure_parameters success for conn=%p", ctx->conn);

    ctx->configured = true;
    k_sleep(K_SECONDS(1));
    return 0;
}


static int cs_enable_for_ctx(struct reflector_ctx *ctx, bool enable)
{
    struct bt_le_cs_procedure_enable_param params = {
        .config_id = CS_CONFIG_ID,
        .enable = enable ? 1 : 0,
    };

    if (!ctx || !ctx->conn) {
        return -EINVAL;
    }

    /* LOG_INF("cs_enable_for_ctx: conn=%p enable=%d configured=%d procedure_enabled=%d",
            ctx->conn, enable, ctx->configured, ctx->procedure_enabled); */
    //log_ctx_state("before_cs_enable", ctx);

    /* Drain stale completion events */
    while (k_sem_take(&ctx->sem_procedure_enable_complete, K_NO_WAIT) == 0) {
    }

    int err = bt_le_cs_procedure_enable(ctx->conn, &params);
    if (err) {
        LOG_ERR("bt_le_cs_procedure_enable failed (err %d)", err);
        return err;
    }

    //LOG_INF("bt_le_cs_procedure_enable submitted successfully enable=%d", enable);

    if (k_sem_take(&ctx->sem_procedure_enable_complete, K_SECONDS(5)) != 0) {
        LOG_ERR("Timeout waiting for procedure_enable_cb (enable=%d)", enable);
        return -ETIMEDOUT;
    }

    if (ctx->procedure_enabled != enable) {
        LOG_ERR("Procedure state mismatch: requested=%d actual=%d",
                enable, ctx->procedure_enabled);
        return -EIO;
    }

    //log_ctx_state("after_cs_enable_complete", ctx);
    return 0;
}

static const struct bt_le_conn_param default_scan_conn_param = {
    .interval_min = 0x10,
    .interval_max = 0x10,
    .latency = 0,
    .timeout = BT_GAP_MS_TO_CONN_TIMEOUT(4000),
};

static int init_bluetooth_and_scan(struct bt_scan_init_param *scan_params)
{
    int err;

    if (!scan_params) {
        return -EINVAL;
    }

    dk_leds_init();

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    *scan_params = (struct bt_scan_init_param) {
        .scan_param = NULL,
        .conn_param = &default_scan_conn_param,
        .connect_if_match = 1,
    };

    err = scan_init(scan_params);
    if (err) {
        LOG_ERR("Scan init failed (err %d)", err);
        return err;
    }

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }

    return 0;
}

static int wait_for_reflectors(void)
{
    int err;

    LOG_INF("Waiting for up to %d reflectors...", MAX_REFLECTORS);

    while (active_reflector_count < MAX_REFLECTORS) {
        if (k_sem_take(&sem_connected, K_MSEC(100)) == 0) {
            LOG_INF("Reflector connected (%u/%u)",
                active_reflector_count, MAX_REFLECTORS);
        }
    }

    err = bt_scan_stop();
    if (err) {
        LOG_WRN("bt_scan_stop failed (err %d)", err);
    }

    if (active_reflector_count == 0) {
        LOG_ERR("No reflectors connected");
        return -ENODEV;
    }

    return 0;
}

static int setup_connected_reflectors(const struct bt_scan_init_param *scan_params)
{
    int err;

    if (!scan_params) {
        return -EINVAL;
    }

    LOG_INF("Starting setup for %u reflectors", active_reflector_count);

    setup_in_progress = true;
    setup_reflector_count = 0;

    for (int i = 0; i < MAX_REFLECTORS; i++) {
        struct reflector_ctx *ctx = &reflectors[i];

        if (!ctx->in_use || !ctx->conn) {
            continue;
        }

        LOG_INF("Setting up reflector slot %d", i);

        k_sleep(K_MSEC(100));

        err = setup_reflector(ctx, scan_params);
        if (err) {
            LOG_ERR("Setup failed for reflector slot %d (err %d)", i, err);
            continue;
        }

        setup_reflector_count++;
        LOG_INF("Reflector slot %d configured", i);

        k_sleep(K_MSEC(100));
    }

    setup_in_progress = false;

    if (setup_reflector_count == 0) {
        LOG_ERR("No reflector configured successfully");
        return -ENODEV;
    }

    return 0;
}

static int build_round_robin_slots(int rr_slots[MAX_REFLECTORS], int *rr_count)
{
    if (!rr_slots || !rr_count) {
        return -EINVAL;
    }

    *rr_count = 0;

    for (int i = 0; i < MAX_REFLECTORS; i++) {
        if (reflectors[i].in_use &&
            reflectors[i].configured &&
            reflectors[i].conn) {
            rr_slots[*rr_count] = i;
            (*rr_count)++;
        }
    }

    if (*rr_count == 0) {
        LOG_ERR("No configured reflectors available for ranging");
        return -ENODEV;
    }

    LOG_INF("Starting round-robin channel sounding on %d reflector(s)", *rr_count);
    return 0;
}

static bool reflector_ready_for_ranging(const struct reflector_ctx *ctx)
{
    return ctx &&
           ctx->in_use &&
           ctx->configured &&
           ctx->conn;
}

static void clear_measurement_state(struct reflector_ctx *ctx)
{
    if (!ctx) {
        return;
    }

    memset(ctx->distance_estimate_buffers, 0, sizeof(ctx->distance_estimate_buffers));
    memset(ctx->n_iqs, 0, sizeof(ctx->n_iqs));
    memset(&ctx->cs_de_report, 0, sizeof(ctx->cs_de_report));
}

static int prepare_active_reflector(int slot, const int rr_slots[], int rr_count)
{
    struct reflector_ctx *ctx;
    int err;

    if (!rr_slots || rr_count <= 0 || slot < 0 || slot >= MAX_REFLECTORS) {
        return -EINVAL;
    }

    ctx = &reflectors[slot];

    if (!reflector_ready_for_ranging(ctx)) {
        return -ENODEV;
    }

    LOG_INF("Round-robin: preparing reflector slot %d", slot);

    reflector_round_reset(ctx);
    clear_measurement_state(ctx);

    for (int i = 0; i < rr_count; i++) {
        int other_slot = rr_slots[i];

        if (other_slot == slot) {
            continue;
        }

        if (reflectors[other_slot].in_use && reflectors[other_slot].conn) {
            set_conn_interval(&reflectors[other_slot], CONN_INTERVAL_IDLE);
        }
    }

    err = set_conn_interval(ctx, CONN_INTERVAL_RANGING);
    if (err) {
        LOG_ERR("Failed to set ranging interval on reflector slot %d (err %d)", slot, err);
        return err;
    }

    k_sleep(K_MSEC(150));
    return 0;
}

static int collect_measurements_for_reflector(struct reflector_ctx *ctx, int slot)
{
    int measurement_count = 0;
    int timeout_count = 0;

    if (!ctx) {
        return -EINVAL;
    }

    LOG_INF("CS enabled on reflector slot %d, collecting %d measurements",
            slot, MEASUREMENTS_PER_REFLECTOR);

    while (measurement_count < MEASUREMENTS_PER_REFLECTOR) {
        if (!ctx->in_use || !ctx->configured || !ctx->conn || !ctx->procedure_enabled) {
            LOG_WRN("Reflector slot %d no longer ready during measurement", slot);
            return -ENODEV;
        }

        if (k_sem_take(&ctx->sem_distance_estimate_updated, K_SECONDS(3)) == 0) {
            bool printed = false;

            for (uint8_t ap = 0; ap < MAX_AP; ap++) {
                if (ctx->distance_estimate_buffers[ap].num_valid != 0) {
                    distance_estimates_print(ctx, slot, ap);
                    printed = true;
                }
            }

            if (printed) {
                measurement_count++;
                timeout_count = 0;

                LOG_INF("Reflector slot %d measurement %d/%d",
                        slot, measurement_count, MEASUREMENTS_PER_REFLECTOR);
            }

            continue;
        }

        timeout_count++;
        LOG_WRN("Timeout waiting for distance estimate on reflector slot %d", slot);

        if (timeout_count >= 2) {
            LOG_WRN("Too many consecutive timeouts on reflector slot %d", slot);
            return -ETIMEDOUT;
        }
    }

    return 0;
}

static void disable_active_reflector(struct reflector_ctx *ctx, int slot)
{
    int err;

    if (!ctx || !ctx->conn || !ctx->procedure_enabled) {
        return;
    }

    LOG_INF("Disabling CS on reflector slot %d", slot);

    err = cs_enable_for_ctx(ctx, false);
    if (err) {
        LOG_ERR("Failed to disable CS on reflector slot %d (err %d)", slot, err);
    }
}

static void run_round_robin(const int rr_slots[], int rr_count)
{
    int rr_idx = 0;

    while (true) {
        int slot;
        struct reflector_ctx *ctx;
        int err;

        slot = rr_slots[rr_idx];
        ctx = &reflectors[slot];

        if (!reflector_ready_for_ranging(ctx)) {
            rr_idx = (rr_idx + 1) % rr_count;
            continue;
        }

        err = prepare_active_reflector(slot, rr_slots, rr_count);
        if (err) {
            rr_idx = (rr_idx + 1) % rr_count;
            k_sleep(K_MSEC(500));
            continue;
        }

        LOG_INF("Round-robin: enabling CS on reflector slot %d", slot);

        err = cs_enable_for_ctx(ctx, true);
        if (err) {
            LOG_ERR("Failed to enable CS on reflector slot %d (err %d)", slot, err);
            rr_idx = (rr_idx + 1) % rr_count;
            k_sleep(K_MSEC(500));
            continue;
        }

        (void)collect_measurements_for_reflector(ctx, slot);
        disable_active_reflector(ctx, slot);

        k_sleep(K_MSEC(50));
        rr_idx = (rr_idx + 1) % rr_count;
    }
}

int main(void)
{
    struct bt_scan_init_param scan_params;
    int rr_slots[MAX_REFLECTORS];
    int rr_count;
    int err;

    err = init_bluetooth_and_scan(&scan_params);
    if (err) {
        return 0;
    }

    err = wait_for_reflectors();
    if (err) {
        return 0;
    }

    err = setup_connected_reflectors(&scan_params);
    if (err) {
        return 0;
    }

    err = build_round_robin_slots(rr_slots, &rr_count);
    if (err) {
        return 0;
    }

    run_round_robin(rr_slots, rr_count);
    return 0;
}