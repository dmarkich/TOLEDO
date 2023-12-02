/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"
#include "lc_pwm_led.h"
#include <lamp.h>

#define PWM_SIZE_STEP 512

struct lightness_ctx {
	struct bt_mesh_lightness_srv lightness_srv;
	struct k_work_delayable per_work;
	uint16_t target_lvl;
	uint16_t current_lvl;
	uint32_t time_per;
	uint32_t rem_time;
};

/* Set up a repeating delayed work to blink the DK's LEDs when attention is
 * requested.
 */
static struct k_work_delayable attention_blink_work;
static bool attention;

static void attention_blink(struct k_work *work)
{
	static int idx;
	const uint8_t pattern[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led_out1))
		BIT(0),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led_out2))
		BIT(1),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led_out3))
		BIT(2),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led_out4))
		BIT(3),
#endif
	};

	if (attention) {
		dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]);
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
		dk_set_leds(DK_NO_LEDS_MSK);
	}
}

static void attention_on(struct bt_mesh_model *mod)
{
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(struct bt_mesh_model *mod)
{
	/* Will stop rescheduling blink timer */
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static void start_new_light_trans(const struct bt_mesh_lightness_set *set,
				  struct lightness_ctx *ctx)
{
	uint32_t step_cnt = abs(set->lvl - ctx->current_lvl) / PWM_SIZE_STEP;
	uint32_t time = set->transition ? set->transition->time : 0;
	uint32_t delay = set->transition ? set->transition->delay : 0;

	ctx->target_lvl = set->lvl;
	ctx->time_per = (step_cnt ? time / step_cnt : 0);
	ctx->rem_time = time;
	k_work_reschedule(&ctx->per_work, K_MSEC(delay));

	printk("New light transition-> Lvl: %d, Time: %d, Delay: %d\n",
	       set->lvl, time, delay);
}

static void periodic_led_work(struct k_work *work)
{
	struct lightness_ctx *l_ctx =
		CONTAINER_OF(work, struct lightness_ctx, per_work);
	l_ctx->rem_time -= l_ctx->time_per;

	if ((l_ctx->rem_time <= l_ctx->time_per) ||
	    (abs(l_ctx->target_lvl - l_ctx->current_lvl) <= PWM_SIZE_STEP)) {
		struct bt_mesh_lightness_status status = {
			.current = l_ctx->target_lvl,
			.target = l_ctx->target_lvl,
		};

		l_ctx->current_lvl = l_ctx->target_lvl;
		l_ctx->rem_time = 0;

		bt_mesh_lightness_srv_pub(&l_ctx->lightness_srv, NULL, &status);

		goto apply_and_print;
	} else if (l_ctx->target_lvl > l_ctx->current_lvl) {
		l_ctx->current_lvl += PWM_SIZE_STEP;
	} else {
		l_ctx->current_lvl -= PWM_SIZE_STEP;
	}

	k_work_reschedule(&l_ctx->per_work, K_MSEC(l_ctx->time_per));
apply_and_print:
	lc_pwm_led_set(l_ctx->current_lvl);
	// set_lamp(l_ctx->current_lvl);
	fade_lamp();
	printk("Current light lvl: %u/65535\n", l_ctx->current_lvl);
}

static void light_set(struct bt_mesh_lightness_srv *srv,
		      struct bt_mesh_msg_ctx *ctx,
		      const struct bt_mesh_lightness_set *set,
		      struct bt_mesh_lightness_status *rsp)
{
	struct lightness_ctx *l_ctx =
		CONTAINER_OF(srv, struct lightness_ctx, lightness_srv);

	start_new_light_trans(set, l_ctx);
	rsp->current = l_ctx->current_lvl;
	rsp->target = l_ctx->target_lvl;
	rsp->remaining_time = set->transition ? set->transition->time : 0;
}

static void light_get(struct bt_mesh_lightness_srv *srv,
		      struct bt_mesh_msg_ctx *ctx,
		      struct bt_mesh_lightness_status *rsp)
{
	struct lightness_ctx *l_ctx =
		CONTAINER_OF(srv, struct lightness_ctx, lightness_srv);

	rsp->current = l_ctx->current_lvl;
	rsp->target = l_ctx->target_lvl;
	rsp->remaining_time = l_ctx->rem_time;
}

static const struct bt_mesh_lightness_srv_handlers lightness_srv_handlers = {
	.light_set = light_set,
	.light_get = light_get,
};

static struct lightness_ctx my_ctx = {
	.lightness_srv = BT_MESH_LIGHTNESS_SRV_INIT(&lightness_srv_handlers),

};

static struct bt_mesh_light_ctrl_srv light_ctrl_srv =
	BT_MESH_LIGHT_CTRL_SRV_INIT(&my_ctx.lightness_srv);

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(1,
		     BT_MESH_MODEL_LIST(
			     BT_MESH_MODEL_CFG_SRV,
			     BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
			     BT_MESH_MODEL_LIGHTNESS_SRV(
					 &my_ctx.lightness_srv)),
		     BT_MESH_MODEL_NONE),
	BT_MESH_ELEM(2,
		     BT_MESH_MODEL_LIST(
			     BT_MESH_MODEL_LIGHT_CTRL_SRV(&light_ctrl_srv)),
		     BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

const struct bt_mesh_comp *model_handler_init(void)
{
	k_work_init_delayable(&attention_blink_work, attention_blink);
	k_work_init_delayable(&my_ctx.per_work, periodic_led_work);

	return &comp;
}

void model_handler_start(void)
{
	int err;

	if (bt_mesh_is_provisioned()) {
		return;
	}

	err = bt_mesh_light_ctrl_srv_enable(&light_ctrl_srv);
	if (!err) {
		printk("Successfully enabled LC server\n");
	}
}
