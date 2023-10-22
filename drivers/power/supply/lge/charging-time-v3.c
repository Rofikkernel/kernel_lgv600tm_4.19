/*
 *   V2 : auto calculating algorithm
 *   V3 : V3 is based on V2. It supports ACTM.
 */
#define pr_fmt(fmt) "CHGTIME: %s: " fmt, __func__
#define pr_chgtime(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_info(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

#include <linux/of.h>
#include <linux/slab.h>
#include "veneer-primitives.h"

#define LGTTF_SLOT_COUNT         256
#define SAMPLING_PERIOD_MS       1500

#define CURR_PROFILE_COL         6
#define TARGET_DATA_COL          5
#define CURR_PROFILE_RAW_SIZE    20
#define CURR_PROFILE_SIZE        (CURR_PROFILE_COL * CURR_PROFILE_RAW_SIZE)
#define TARGET_DATA_RAW_SIZE     24
#define TARGET_DATA_SIZE         (TARGET_DATA_COL * TARGET_DATA_RAW_SIZE)

#define BCC_VOTER                   "BCC"

static int pr_debugmask = ERROR | UPDATE | EVALUATE;

enum lgttf_actm {
	LGTTF_ACTM_THERMAL = 0,
	LGTTF_ACTM_BALANCED,
	LGTTF_ACTM_CHARGING,
	LGTTF_ACTM_MAX,
};

enum lgttf_id {
	LGTTF_ID_CP_PPS = 0,
	LGTTF_ID_CP_QC3,
	LGTTF_ID_QC2,
	LGTTF_ID_DCP,
	LGTTF_ID_SDP,
	LGTTF_ID_WLC_EPP,
	LGTTF_ID_WLC_BPP,
	LGTTF_ID_BCC,
	LGTTF_ID_MAX,
};

enum lgttf_type {
	LGTTF_TYPE_CP_PPS = 0,
	LGTTF_TYPE_CP_QC3,
	LGTTF_TYPE_CP_NONE,
	LGTTF_TYPE_WIRELESS,
	LGTTF_TYPE_BCC,
};

struct lgttf_datum {
	int msoc;          /*    255 */
	int raw_soc;       /*    x10 */
	int ui_soc;        /*    x10 */
	int chg_curr;      /*     uA */
	int slot_time;     /*    sec */
	int ttf;           /* second */
};

struct lgttf_curr_profile {
	int rsoc;
	int curr_ma;
};

struct lgttf_table {
	int id;
	int actm_mode;
	int type;
	int power;         /*    mW */
	int max_current;   /*    mA */
	int down_ratio;    /*    uA */
	int target;        /*   sec */
	struct lgttf_datum data[LGTTF_SLOT_COUNT];
};

struct lgttf_base_param {
	int current_size;  /* total current table size            */
	u32 dt_current[CURR_PROFILE_SIZE];
	u32 dt_target[TARGET_DATA_SIZE];
	int full_soc;		   /* rescale soc                         */
	int designed;      /* full capacity designed              */
	int fix_ui_soc;    /* slot time is fixed from this ui soc */
};

struct lgttf_rt_param {
	int chg_type;
	int power;
	int bcc_ttf;
	int selected;
	int actm_mode;
};

static struct lgttf {
	struct lgttf_base_param base;
	struct lgttf_rt_param rt;
	struct lgttf_table table[LGTTF_ACTM_MAX][LGTTF_ID_MAX+1]; /* index 8 is for new DCP */
	struct lgttf_curr_profile profile[LGTTF_ACTM_MAX][CURR_PROFILE_RAW_SIZE];

	struct voter_entry ibat_voter;
	struct delayed_work	lgttf_work;

	int (*get_actm_param)(int id, int *val);
    int (*set_actm_param)(int id, int val);
} lgttf = {0, };

void lgttf_init_rt(void)
{
	lgttf.rt.chg_type = LGTTF_TYPE_CP_NONE;
	lgttf.rt.power = -1;
	lgttf.rt.bcc_ttf = 0;
	lgttf.rt.selected = -1;
	lgttf.rt.actm_mode = -9999;
}

const char str_actm[4][9] = {
	"THERMAL", "BALANCED", "CHARGING", "AUTO"};
const char str_chg_type[5][9] = {
	"CP_PPS", "CP_QC3", "CP_NONE", "WIRELESS", "BCC"};
const char str_id[8][8] = {
	"CP_PPS", "CP_QC3", "QC2", "DCP", "SDP", "WLC_EPP", "WLC_BPP", "BCC"};

#define MAX_ERROR_RATIO    5     /* decipct */
#define MAX_TRY_COMP       10
static int lgttf_set_each_data(struct lgttf_table *table, struct lgttf_curr_profile *curr)
{
	int rc = 0, i = 0;
	int next = 0, t_soc = 0, t_curr = 0;
	int temp = 0;
	int total = 0, total_temp = 0;
	int exit_count = MAX_TRY_COMP, error = 0;

	/* make base table */
	for (i = 0; i < LGTTF_SLOT_COUNT; i++) {
		t_soc = curr[next].rsoc;
		t_curr = curr[next].curr_ma;

		if (i > t_soc && i <= lgttf.base.full_soc) {
			next++;
			if (next >= lgttf.base.current_size)
				next = lgttf.base.current_size - 1;
			t_soc = curr[next].rsoc;
			t_curr = curr[next].curr_ma;
		}

		table->data[i].msoc = i;
		table->data[i].raw_soc = (i * 100) / 255;
		table->data[i].ui_soc = (((i * 1000) / lgttf.base.full_soc) + 5) / 10;

		if (table->down_ratio > 0 && i > 0 &&
			table->data[i].raw_soc > 0 &&
			(t_curr * 1000) >= table->data[i-1].chg_curr)
			table->data[i].chg_curr =
				table->data[i-1].chg_curr - table->down_ratio;
		else
			table->data[i].chg_curr = min(table->max_current, t_curr) * 1000;

		temp = lgttf.base.designed * 60 * 60 / 255;
		table->data[i].slot_time = temp * 1000 / (table->data[i].chg_curr/1000);
		table->data[i].slot_time = (table->data[i].slot_time + 500 ) / 1000;

		/* fix cv slot_time */
		if (table->data[i].ui_soc > lgttf.base.fix_ui_soc) {
			table->data[i].slot_time =
				max(table->data[i].slot_time,
					lgttf.table[LGTTF_ACTM_THERMAL][LGTTF_ID_CP_PPS].data[i].slot_time);
		}

		if ((i < (lgttf.base.full_soc - 1)) && (table->data[i].ui_soc > 1))
			total += table->data[i].slot_time;

		pr_chgtime(MONITOR,
			"proto data: %d, %d, %d, %d, %d, %d, %d\n",
			table->data[i].msoc, table->data[i].raw_soc, table->data[i].ui_soc,
			table->data[i].chg_curr / 1000, table->data[i].slot_time, total, total / 60);
	}

	error = max(table->target, total) - min(table->target, total);
	pr_chgtime(UPDATE,
		"Before Compensation: target=%d(%d), total=%d(%d), diff=%d(%d)\n",
		table->target, table->target / 60,
		total, total / 60, error, error / 60);

	/* compensate ttf accuracy: allowance error is max 0.5%  */
	while ((--exit_count > 0) && (error > (table->target * MAX_ERROR_RATIO / 1000))) {
		total_temp = 0;
		for (i = 0; i < LGTTF_SLOT_COUNT; i++ ) {
			if (total > 0)
				table->data[i].slot_time =
					(((table->data[i].slot_time * 10) * table->target / total) + 5) / 10;
			else
				table->data[i].slot_time = 0;

			/* fix cv slot_time */
			if (table->data[i].ui_soc > lgttf.base.fix_ui_soc) {
				table->data[i].slot_time =
					max(table->data[i].slot_time,
						lgttf.table[LGTTF_ACTM_THERMAL][LGTTF_ID_CP_PPS].data[i].slot_time);
			}

			if ((i < (lgttf.base.full_soc - 1)) && (table->data[i].ui_soc > 1))
				total_temp += table->data[i].slot_time;
		}

		total = total_temp;
		error = max(table->target, total) - min(table->target, total);
	}

	/* make ttf data */
	for (i = (LGTTF_SLOT_COUNT-1); i >= 0 ; i--) {
		if (i > lgttf.base.full_soc - 2)
			table->data[i].ttf = 0;
		else
			table->data[i].ttf =
				table->data[i+1].ttf + table->data[i].slot_time;
	}

	pr_chgtime(UPDATE,
		"After Compensation(%d): "
		"target=%d(%d), total=%d(%d), diff=%d(%d), ttf[4]=%d(%d)\n",
			MAX_TRY_COMP - exit_count,
			table->target, table->target / 60, total, total / 60,
			error, error / 60, table->data[4].ttf, table->data[4].ttf / 60);

	/* print result for debug */
	for (i = 0; i < LGTTF_SLOT_COUNT; i++)
		pr_chgtime(MONITOR,
			"last data[%d]: %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			MAX_TRY_COMP - exit_count,
			table->data[i].msoc, table->data[i].raw_soc, table->data[i].ui_soc,
			table->data[i].chg_curr / 1000, table->data[i].slot_time, table->data[i].ttf,
			table->data[i].ttf / 60, table->target, table->target / 60);

	return rc;
}

static int lgttf_set_each_table(
	struct lgttf_table *table, int id, u32 *target, struct lgttf_curr_profile *curr)
{
	int rc = 0;
	int temp = 0;

	memset(table, 0, sizeof(struct lgttf_table));

	table->id = id;
	table->actm_mode = target[0] / 100;
	table->type = target[0] % 100;
	table->power = target[1];
	table->max_current = target[2];
	table->down_ratio = target[3];
	table->target = target[4] * 60;

	if (table->down_ratio > 0 &&
		(table->down_ratio * 245 / 1000) > (table->max_current - 300)) {
		temp = (table->max_current - 300) * 1000 / 245;
		pr_chgtime(ERROR,
			"[ERROR id: %d] down_ratio(%d) is too big. "
			"I will rescale it to %d, but you have to check down_raio.\n",
			id, table->down_ratio, temp);
		table->down_ratio = temp;
	}

	pr_chgtime(UPDATE,
		"header: id=%s, actm=%s, type=%s, power=%d, max_current=%d, "
		"down_ratio=%d, target=%d(%d)\n",
		str_id[table->id],
		str_actm[table->actm_mode],
		str_chg_type[table->type],
		table->power, table->max_current,
		table->down_ratio, table->target, table->target / 60);

	lgttf_set_each_data(table, curr);

	return rc;
}

int charging_time_remains(int rawsoc)
{
	int result_new = 0;
	static int result_old = 0;
	struct lgttf_table *table = NULL;
	int selected = lgttf.rt.selected;
	int actm = lgttf.rt.actm_mode;
	int type = lgttf.rt.chg_type;

	if (rawsoc < 0 || rawsoc >= LGTTF_SLOT_COUNT ||
		selected < LGTTF_ID_CP_PPS || selected > LGTTF_ID_MAX ||
		actm < LGTTF_ACTM_THERMAL || actm > LGTTF_ACTM_CHARGING) {
		return -1;
	}

	table = &lgttf.table[actm][selected];
	if (!table)
		return -1;

	result_new = table->data[rawsoc].ttf;
	if (result_new != result_old) {
		pr_chgtime(UPDATE,
			"report...soc=%d(%d, %d), ttf=%dsec(%dmin) "
			"(id=%s, actm=%s, type=%s, power=%d)\n",
			rawsoc, table->data[rawsoc].ui_soc, table->data[rawsoc].raw_soc,
			result_new, result_new / 60,
			(selected < LGTTF_ID_CP_PPS || selected > LGTTF_ID_BCC) ?
				"ERROR" : str_id[selected],
			(actm < LGTTF_ACTM_THERMAL || actm > LGTTF_ACTM_MAX) ?
				"ERROR" : str_actm[actm],
			(type < LGTTF_TYPE_CP_PPS || type > LGTTF_TYPE_BCC) ?
				"ERROR" : str_chg_type[type],
			lgttf.rt.power);
	}

	result_old = result_new;
	return result_old;
}

static int lgttf_find_table(int power)
{
	int i = 0, actm = lgttf.rt.actm_mode;
	int upper = -1, lower = -1;
	u32 target[TARGET_DATA_COL] = {0, };
	int upper_time = 0, lower_time = 0, time_error = 0;

	for (i = LGTTF_ID_QC2; i < LGTTF_ID_WLC_EPP; i++) {
		if (lgttf.table[actm][i].power >= power) {
			if (upper > 0) {
				if (lgttf.table[actm][upper].power > lgttf.table[actm][i].power)
					upper = i;
			}
			else
				upper = i;
		}
		else {
			if (lower > 0) {
				if (lgttf.table[actm][lower].power < lgttf.table[actm][i].power)
					lower = i;
			}
			else
				lower = i;
		}
	}

	if (upper < 0 && lower >= 0) {
		lgttf.rt.selected = lower;
	}
	else if (upper >= 0 && lower < 0) {
		lgttf.rt.selected = upper;
	}
	else if (upper >= 0 && lower >= 0) {
		lgttf.rt.selected = upper;
		if (lgttf.table[actm][upper].power != power && upper != lower) {
			target[0] = LGTTF_TYPE_CP_NONE + (actm * 100);
			target[1] = power;
			target[2] = power / 5;
			target[3] = 0;

			upper_time = 60 * 60 * 1000 /
				(lgttf.table[actm][upper].power * 1000 / 5 / lgttf.base.designed);
			lower_time = 60 * 60 * 1000 /
				(lgttf.table[actm][lower].power * 1000 / 5 / lgttf.base.designed);
			time_error = (lgttf.table[actm][upper].target - upper_time +
				lgttf.table[actm][lower].target - lower_time) / 2;
			target[4] = 60 * 1000 / (power * 1000 / 5 / lgttf.base.designed)
				+ (time_error / 60);

			pr_chgtime(MONITOR,
				"upper=%d, %d, %d, lower=%d, %d, %d, "
				"error=%d, target=%d\n",
				upper, lgttf.table[actm][upper].power, upper_time,
				lower, lgttf.table[actm][lower].power, lower_time,
				time_error, target[4]);

			lgttf.rt.selected = LGTTF_ID_MAX;
				lgttf_set_each_table(&lgttf.table[actm][lgttf.rt.selected],
					lgttf.rt.selected, target, lgttf.profile[actm]);
		}
	}
	else {
		lgttf.rt.selected = -1;
	}

	pr_chgtime(UPDATE, "found...power=%d, selected=%s, type=%d, current=%d\n",
		power,
		(lgttf.rt.selected < LGTTF_ID_CP_PPS ||
			lgttf.rt.selected > LGTTF_ID_BCC) ?
				"ERROR" : str_id[lgttf.rt.selected],
		lgttf.table[actm][lgttf.rt.selected].type,
		lgttf.table[actm][lgttf.rt.selected].max_current);

	return lgttf.rt.selected;
}

static void lgttf_work_func(struct work_struct *work)
{
	static int retry = 0;
	int cp_type = LGTTF_TYPE_CP_NONE;
	int actm_mode = LGTTF_ACTM_THERMAL;
	int power = 0, rawsoc = 0, bcc_ttf = 0, charger = 0;

	if (lgttf.get_actm_param(VENEER_FEED_CHARGER_TYPE, &charger)) {
		schedule_delayed_work(
			&lgttf.lgttf_work, msecs_to_jiffies(SAMPLING_PERIOD_MS));
		return;
	}

	if (charger == CHARGING_SUPPLY_WIRELESS_5W ) {
		lgttf.rt.selected = LGTTF_ID_WLC_BPP;
		lgttf.rt.chg_type = LGTTF_TYPE_WIRELESS;
	} else if (charger == CHARGING_SUPPLY_WIRELESS_9W) {
		lgttf.rt.selected = LGTTF_ID_WLC_EPP;
		lgttf.rt.chg_type = LGTTF_TYPE_WIRELESS;
	}

	lgttf.get_actm_param(VENEER_FEED_ACTM_MODE_NOW, &actm_mode);
	if (actm_mode < 0 && retry < 15 ) {
		retry++;
		schedule_delayed_work(
			&lgttf.lgttf_work, msecs_to_jiffies(SAMPLING_PERIOD_MS));
		return;
	}

	if (actm_mode == -1) {
		actm_mode = LGTTF_ACTM_CHARGING + LGTTF_ACTM_CHARGING * 10;
	}
	else if (actm_mode == -2) {
		actm_mode = LGTTF_ACTM_BALANCED + LGTTF_ACTM_BALANCED * 10;
	}

	if (lgttf.rt.chg_type == LGTTF_TYPE_WIRELESS)
		lgttf.rt.actm_mode = actm_mode / 10;
	else
		lgttf.rt.actm_mode = actm_mode % 10;

	lgttf.get_actm_param(VENEER_FEED_ACTM_CURRENT_CP_PPS, &cp_type);
	if (cp_type > 0) {
		lgttf.rt.chg_type = LGTTF_TYPE_CP_PPS;
		lgttf.rt.selected = LGTTF_ID_CP_PPS;
	}
	else {
		lgttf.get_actm_param(VENEER_FEED_ACTM_CURRENT_CP_QC30, &cp_type);
		if (cp_type > 0) {
			lgttf.rt.chg_type = LGTTF_TYPE_CP_QC3;
			lgttf.rt.selected = LGTTF_ID_CP_QC3;
		}
	}

	lgttf.get_actm_param(VENEER_FEED_POWER_NOW, &power);
	lgttf.get_actm_param(VENEER_FEED_CAPACITY_RAW, &rawsoc);
	lgttf.get_actm_param(VENEER_FEED_BSM_TTF, &bcc_ttf);

	if (power > 0 && rawsoc >= 0) {
		if (bcc_ttf > 0) {
			lgttf.rt.chg_type = LGTTF_TYPE_BCC;
			lgttf.rt.selected = LGTTF_ID_BCC;
			power = lgttf.table[LGTTF_ACTM_BALANCED][LGTTF_ID_BCC].power;
		}
	}
	else {
		pr_chgtime(ERROR, "Error get_resource\n");
		return;
	}

	if (lgttf.rt.chg_type == LGTTF_TYPE_CP_NONE &&
		(lgttf.rt.power != power || lgttf.rt.bcc_ttf != bcc_ttf)) {
		pr_chgtime(UPDATE,
			"One more sampling (actm: %s, type: %s, power: %dmW, bcc: %d)\n",
				(lgttf.rt.actm_mode < LGTTF_ACTM_THERMAL ||
					lgttf.rt.actm_mode > LGTTF_ACTM_MAX) ?
						"ERROR" : str_actm[lgttf.rt.actm_mode],
				(lgttf.rt.chg_type < LGTTF_TYPE_CP_PPS ||
					lgttf.rt.chg_type > LGTTF_TYPE_BCC) ?
						"ERROR" : str_chg_type[lgttf.rt.chg_type],
				power, bcc_ttf);

		lgttf.rt.bcc_ttf = bcc_ttf;
		if (power >= 2500)
			lgttf.rt.power = power;

		schedule_delayed_work(
			&lgttf.lgttf_work, msecs_to_jiffies(SAMPLING_PERIOD_MS));
	}
	else {
		if (lgttf.rt.chg_type == LGTTF_TYPE_CP_NONE) {
			if (lgttf_find_table(power) < 0) {
				pr_chgtime(ERROR, "Can't find the proper table.\n");
				return;
			}
		}

		if (lgttf.rt.selected == LGTTF_ID_BCC)
			veneer_voter_set(&lgttf.ibat_voter,
				lgttf.table[LGTTF_ACTM_BALANCED][LGTTF_ID_BCC].max_current);
		else
			veneer_voter_release(&lgttf.ibat_voter);

		lgttf.set_actm_param(VENEER_FEED_POWER_SUPPLY_CHANGED, 0);
		pr_chgtime(UPDATE,
			"charging time table is selected...actm=%s, type=%s, id=%s, power=%dmW.\n",
				(lgttf.rt.actm_mode < LGTTF_ACTM_THERMAL ||
					lgttf.rt.actm_mode > LGTTF_ACTM_MAX) ?
						"ERROR" : str_actm[lgttf.rt.actm_mode],
				(lgttf.rt.chg_type < LGTTF_TYPE_CP_PPS ||
					lgttf.rt.chg_type > LGTTF_TYPE_BCC) ?
						"ERROR" : str_chg_type[lgttf.rt.chg_type],
				(lgttf.rt.selected < LGTTF_ID_CP_PPS ||
					lgttf.rt.selected > LGTTF_ID_BCC) ?
						"ERROR" : str_id[lgttf.rt.selected], power);
	}
}

/* ttf main function */
bool charging_time_update(enum charging_supplier charger, bool reloading)
{
	static enum charging_supplier type = CHARGING_SUPPLY_TYPE_NONE;
	bool charging = false, ret = false;

	if (reloading)
		charger = type;

	if (type != charger || reloading) {
		charging = ( (charger != CHARGING_SUPPLY_TYPE_UNKNOWN) &&
                     (charger != CHARGING_SUPPLY_TYPE_NONE)        );

		if (charging) {
			cancel_delayed_work_sync(&lgttf.lgttf_work);
			schedule_delayed_work(
				&lgttf.lgttf_work, msecs_to_jiffies(SAMPLING_PERIOD_MS));

			pr_chgtime(UPDATE, "Charging started, Start sampling\n");
		}
		else {
			pr_chgtime(UPDATE, "Charging stopped\n");
			charging_time_clear();
		}

		type = charger;
		ret = true;
	}

	return ret;
}

static int lgttf_parse_dt(struct device_node* dnode, int fullraw)
{
	int rc = 0, i = 0, j = 0;
	int id = 0;

	lgttf.base.full_soc = fullraw;
	rc |= of_property_read_u32(dnode,
			"lge,full-capacity-design",
			&lgttf.base.designed);
	rc |= of_property_read_u32(dnode,
			"lge,fix-slot-time-ui-soc",
			&lgttf.base.fix_ui_soc);

	rc |= of_property_read_u32(dnode,
			"lge,charging-current-profile-raws",
			&lgttf.base.current_size);
	rc |= of_property_read_u32_array(dnode,
			"lge,charging-current-profile",
			lgttf.base.dt_current,
			lgttf.base.current_size * CURR_PROFILE_COL);

	id = 0;
	memset(lgttf.profile, -1, sizeof(struct lgttf_curr_profile));
	for (j = 0; j < lgttf.base.current_size; j++) {
		for (i = 0; i < LGTTF_ACTM_MAX; i++) {
			lgttf.profile[i][j].rsoc = lgttf.base.dt_current[id];
			id++;
			lgttf.profile[i][j].curr_ma = lgttf.base.dt_current[id];
			id++;
		}
	}

	rc |= of_property_read_u32_array(dnode,
			"lge,charging-target-data", lgttf.base.dt_target,
			LGTTF_ACTM_MAX * LGTTF_ID_MAX * TARGET_DATA_COL);

	for (j = 0; j < LGTTF_ID_MAX; j++) {
		for (i = 0; i < LGTTF_ACTM_MAX; i++) {
			id = i + (j * LGTTF_ACTM_MAX);
			lgttf_set_each_table(&lgttf.table[i][j], j,
				&lgttf.base.dt_target[TARGET_DATA_COL * id], lgttf.profile[i]);
		}
	}

	return rc;
}

bool charging_time_create(
	struct device_node* dnode, int fullraw,
	int (*get_veneer_param)(int id, int *val),
    int (*set_veneer_param)(int id, int val))
{
	if (!dnode || !get_veneer_param || !set_veneer_param) {
		pr_chgtime(ERROR, "null param error.\n");
		goto fail;
	}

	if (0 > fullraw || fullraw >= LGTTF_SLOT_COUNT) {
		pr_chgtime(ERROR, "fullraw(%d) is out of range\n", fullraw);
		goto fail;
	}

	if (lgttf_parse_dt(dnode, fullraw)) {
		pr_chgtime(ERROR, "'Error of parsing dt file.\n");
		goto fail;
	}

	lgttf.get_actm_param = get_veneer_param;
	lgttf.set_actm_param = set_veneer_param;

	INIT_DELAYED_WORK(&lgttf.lgttf_work, lgttf_work_func);

	veneer_voter_register(&lgttf.ibat_voter, BCC_VOTER, VOTER_TYPE_IBAT, false);

	lgttf_init_rt();

	return true;

fail:
	pr_chgtime(ERROR, "Failed to create charging time\n");
	return false;
}

void lgttf_clear(void)
{
	cancel_delayed_work_sync(&lgttf.lgttf_work);
	lgttf_init_rt();
	veneer_voter_release(&lgttf.ibat_voter);
}

void charging_time_clear(void)
{
	lgttf_clear();
}

void lgttf_destroy(void)
{
	veneer_voter_unregister(&lgttf.ibat_voter);
}

void charging_time_destroy(void)
{
	lgttf_destroy();
}