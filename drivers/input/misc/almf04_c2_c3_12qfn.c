#define MODULE_TAG	"<almf04>"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/pm_wakeup.h>
#include <linux/unistd.h>
#include <linux/async.h>
#include <linux/in.h>

#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <linux/kernel.h>

#include <linux/firmware.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

#include <soc/qcom/lge/board_lge.h>

//#include "bs_log.h"

#include "almf04_c2_c3_12qfn.h"


#define CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE
//#define CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY

#define ALMF04_DRV_NAME     "lge_sar_rf"

#define FAR_STATUS      1
#define NEAR_STATUS     0

#ifdef ALMF04_3CH_SAR
	#define CNT_MAX_CH		4
#else
	#define CNT_MAX_CH		3
#endif

#define CH1_FAR         0x2
#define CH1_NEAR        0x1
#define CH2_FAR         (0x2 << 2)
#define CH2_NEAR        (0x1 << 2)

#ifdef ALMF04_3CH_SAR
	#define CH3_FAR         (0x2 << 4)
	#define CH3_NEAR        (0x1 << 4)

	#define SNR_FAR			0x2
	#define SNR_NEAR		0x1
#endif

/* I2C Suspend Check */
#define ALMF04_STATUS_RESUME        0
#define ALMF04_STATUS_SUSPEND       1
#define ALMF04_STATUS_QUEUE_WORK    2

#define SZ_CALDATA_UNIT 					24
static int CalData[CNT_MAX_CH*2][SZ_CALDATA_UNIT];


#define BIT_PERCENT_UNIT            8.192
#define MK_INT(X, Y)                (((int)X << 8)+(int)Y)

#define ENABLE_SENSOR_PINS          0
#define DISABLE_SENSOR_PINS         1

#define ON_SENSOR                   1
#define OFF_SENSOR                  2
#define PATH_CAPSENSOR_CAL  "/mnt/vendor/sns/sar_controller_cal.dat"

static struct i2c_driver almf04_driver;
static struct workqueue_struct *almf04_workqueue;


#ifdef CONFIG_OF
enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
	DT_STRING,
};

struct sensor_dt_to_pdata_map {
	const char			*dt_name;
	void				*ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type	type;
	int				default_val;
};
#endif

static struct i2c_client *almf04_i2c_client; /* global i2c_client to support ioctl */

struct almf04_platform_data {
	int (*init)(struct i2c_client *client);
	void (*exit)(struct i2c_client *client);
	unsigned int irq_gpio;
	unsigned long chip_enable;

	int (*power_on)(struct i2c_client *client, bool on);
	u32 irq_gpio_flags;

	bool i2c_pull_up;

	struct regulator *vcc_ana;
	struct regulator *vcc_dig;
	struct regulator *vcc_i2c;

	u32 vdd_ana_supply_min;
	u32 vdd_ana_supply_max;
	u32 vdd_ana_load_ua;

	u32 input_pins_num; // not include ref sensor pin
	const char *fw_name;
};

struct almf04_data {
	int (*get_nirq_low)(void);
	struct i2c_client *client;
	struct mutex update_lock;
	struct mutex enable_lock;
	struct delayed_work	dwork;		/* for PS interrupt */
	struct input_dev *input_dev_cap;
#ifdef CONFIG_OF
	struct almf04_platform_data *platform_data;
	int irq;
#endif
	unsigned int enable;
	unsigned int sw_mode;
	atomic_t i2c_status;

	unsigned int cap_detection;
	int touch_out;
};

static bool on_sensor = false;
static bool check_allnear = false;
static bool cal_result = false; // debugging calibration paused
static atomic_t pm_suspend_flag;

#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
static bool probe_end_flag = false;
#endif

static int get_bit(unsigned short x, int n);
static void check_firmware_ready(struct i2c_client *client);
static void check_init_touch_ready(struct i2c_client *client);


static void chg_mode(unsigned char flag, struct i2c_client *client)
{
	if(flag == ON) {
		i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x80);
		PINFO("ALMF04 change_mode : %d\n",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));

		#if defined(USE_ALMF04)
			i2c_smbus_write_byte_data(client, ADDR_ROM_SAFE, VAL_ROM_MASK1);
			i2c_smbus_write_byte_data(client, ADDR_ROM_SAFE, VAL_ROM_MASK2);
		#endif
	}
	else {
		i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x00);
		PINFO("ALMF04 change_mode : %d\n",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));
	}
	mdelay(1);
}

static int Backup_CalData(struct i2c_client *client)
{
	int loop, dloop;
	int ret;
	//============================================================//
	//[200512] ADS Change
	//[START]=====================================================//
	////============================================================//
	////[200506] ADS ADD
	////[START]=====================================================//
	//int chk_val;
	////[END]=======================================================//
	short chk_val;
	//[END]=======================================================//

	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
		if (ret) {
			PINFO("almf04: i2c_write_fail \n");
			return ret;
		}
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_R_CD_DUTY);
		if (ret) {
			PINFO("almf04: i2c_write_fail \n");
			return ret;
		}

		mdelay(1); 		//1 ms Delay

		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
			CalData[loop][dloop] = i2c_smbus_read_byte_data(client,I2C_ADDR_REQ_DATA + dloop);
	}

	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
		if (ret) {
			PINFO("almf04: i2c_write_fail \n");
			return ret;
		}
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_R_CD_REF);
		if (ret) {
			PINFO("almf04: i2c_write_fail \n");
			return ret;
		}

		mdelay(1); 		//1 ms Delay

		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
			CalData[CNT_MAX_CH+loop][dloop] = i2c_smbus_read_byte_data(client,I2C_ADDR_REQ_DATA + dloop);
	}

	//============================================================//
	//[200506] ADS Change
	//[START]=====================================================//
	/* if(CalData[0][0] == 0xFF || (CalData[0][0] == 0x00 && CalData[0][1] == 0x00))
	{
		PINFO("almf04: Invalid cal data, Not back up this value.");
		return -1;
	}*/
	//valid Data Check
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++) 	{
		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop+=2) {
			chk_val = MK_INT(CalData[loop][dloop], CalData[loop][dloop + 1]);
			if(chk_val <= 0) {
				PINFO("almf04: Invalid cal data, Not back up this value.([D:%d,%d]%d)", loop, dloop, chk_val);
				return -1;
			}
		}
	}

	for(loop = 0 ; loop < CNT_MAX_CH ; loop++) {
		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop+=2) {
			chk_val = MK_INT(CalData[CNT_MAX_CH + loop][dloop], CalData[CNT_MAX_CH + loop][dloop + 1]);
			if((chk_val <= 8000) || (chk_val >= 12000)) {
				PINFO("almf04: Invalid cal data, Not back up this value.([R:%d,%d]%d)", loop, dloop, chk_val);
				return -1;
			}
		}
	}
	//[END]=======================================================//

	for(loop =0;loop<(CNT_MAX_CH*2);loop++)
	{
		for(dloop=0;dloop < SZ_CALDATA_UNIT ; dloop++)
			PINFO("almf04: backup_caldata data[%d][%d] : %d \n",loop,dloop,CalData[loop][dloop]);
	}

	PINFO("almf04 backup_cal success");
	return 0;
}

static int Write_CalData(struct i2c_client *client)
{
	int loop, dloop;
	int ret;

	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
		if (ret) {
			PINFO("almf04: i2c_write_fail \n");
			return ret;
		}
		ret =i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_W_CD_DUTY);
		if (ret) {
			PINFO("almf04: i2c_write_fail \n");
			return ret;
		}

		mdelay(1); 		//1 ms Delay

		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
		{
			ret = i2c_smbus_write_byte_data(client,I2C_ADDR_REQ_DATA + dloop, CalData[loop][dloop]);
			if (ret) {
				PINFO("almf04: i2c_write_fail \n");
				return ret;
			}
		}
	}

	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
		if (ret) {
			PINFO("almf04: i2c_write_fail \n");
			return ret;
		}
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_W_CD_REF);
		if (ret) {
			PINFO("almf04: i2c_write_fail \n");
			return ret;
		}

		mdelay(1); 		//1 ms Delay

		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
		{
			ret = i2c_smbus_write_byte_data(client,I2C_ADDR_REQ_DATA + dloop, CalData[CNT_MAX_CH+loop][dloop]);
			if (ret) {
				PINFO("almf04: i2c_write_fail \n");
				return ret;
			}

		}
	}
	return 0;
}

static int RestoreProc_CalData(struct almf04_data *data, struct i2c_client *client)
{
	int loop;
	int ret;
	//Power On
	gpio_set_value(data->platform_data->chip_enable, 0);
	mdelay(450);

	//Calibration data write
	ret = Write_CalData(client);
	if(ret)
		return ret;

	//Initial code write
	for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
		ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
		if (ret) {
			PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
		}
		PINFO("Restore##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
	}

	check_firmware_ready(client);
	i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x80);
	check_firmware_ready(client);

	//Software Reset2
	i2c_smbus_write_byte_data(client,I2C_ADDR_SYS_CTRL, 0x02);
	PINFO("almf04 restore_cal success");
	return 0;
}

#if defined(CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY)
static int Update_Sensitivity(struct almf04_data *data, struct i2c_client *client)
{
	int ret = 0;
	unsigned char loop, ridx, iidx, sstvt[2];
	bool update_flag = false;

	for(loop = 0 ; loop < (CNT_MAX_CH -1) ; loop++)
	{
		ridx = I2C_ADDR_IT_SSTVT_CH1_H+loop*2;
		iidx = IDX_ICODE_ITSSTVT+loop*2;

		sstvt[0] = i2c_smbus_read_byte_data(client, ridx);
		sstvt[1] = i2c_smbus_read_byte_data(client, ridx + 1);

		PINFO(" ALMF04 <CH%d sstvt> read value:[0x%02x,0x%02x], Initvalue:[0x%02x,0x%02x]\n", sstvt[0], sstvt[1], InitCodeVal[iidx], InitCodeVal[iidx+1]);

		if((sstvt[0] != InitCodeVal[iidx]) || (sstvt[1] != InitCodeVal[iidx+1]))
		{
			update_flag = true;
			break;
		}
	}

	if(update_flag == true) {

		mdelay(10);

		PINFO("Update_Sensitivity Start");
		for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
			ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
			if (ret) {
				PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
				return ret;
			}
			PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		}
		check_firmware_ready(client);

		//E-flash Data Save Command
		ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x04);

		if (ret) {
			PINFO("i2c_write_fail[0x%x]",I2C_ADDR_SYS_CTRL);
			goto i2c_fail;
		}
		//============================================================//
		//[191125] ADS Change
		//[START]=====================================================//
		//mdelay(50);		//50ms delay
		check_firmware_ready(client);
		//[END]=======================================================//
		PINFO("Update_Sensitivity Finished");
	}
	return 0;

i2c_fail:
	return ret;
}
#endif

static unsigned char chk_done(unsigned int wait_cnt, struct i2c_client *client)
{
	unsigned int trycnt = 0;
	unsigned char rtn;

	do
	{
		if(++trycnt > wait_cnt) {
			PINFO("ALMF04  RTN_TIMEOUT");
			return RTN_TIMEOUT;
		}
		mdelay(1);
		rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
	}while((rtn & FLAG_DONE) != FLAG_DONE);

	return RTN_SUCC;
}

static unsigned char chk_done_erase(unsigned int wait_cnt, struct i2c_client *client)
{
	unsigned int trycnt = 0;
	unsigned char rtn;

	do
	{
		if(++trycnt > wait_cnt) return RTN_TIMEOUT;

		mdelay(5);

		rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
	}while((rtn & FLAG_DONE_ERASE) != FLAG_DONE_ERASE);

	return RTN_SUCC;
}

static unsigned char erase_eflash(struct i2c_client *client)
{
	#if defined(USE_ALMF04)
		i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
		i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_ALL_ERASE);
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
	#else
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_ERASE_ALL);
	#endif

	if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
		return RTN_TIMEOUT; //timeout

	return RTN_SUCC;
}
/*
static unsigned char write_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * wdata, struct i2c_client *client)
{
        unsigned int i;
	for(i = 0 ; i < SZ_PAGE_DATA ; i++) i2c_smbus_write_byte_data(client, i, wdata[i]);

	#if defined(USE_ALMF04)
		if(flag != FLAG_FUSE)
		{
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_WRITE));
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
		}
		else
		{
			//Erase
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_PG_ERASE);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_START);
			if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
			//write
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_PG_WRITE);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_START);
		}
	#else
		if(flag != FLAG_FUSE)
		{
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, page_addr[1]);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_L_WR);
		}
		else
		{
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, 0x00);
			//Erase
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_ERASE);
			if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
			//write
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_WR);
		}
	#endif

	if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;

	return RTN_SUCC;
}
static unsigned char read_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * rdata, struct i2c_client *client)
{
        unsigned int i;
	#if defined(USE_ALMF04)
		if(flag != FLAG_FUSE)
		{
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_READ));
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
		}
		else
		{
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_PG_READ);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_START);
		}
	#else
		if(flag != FLAG_FUSE)
		{
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, page_addr[1]);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_RD);
		}
		else
		{
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, 0x00);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_RD);
		}
	#endif

	if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;

	for(i = 0 ; i < SZ_PAGE_DATA ; i++) rdata[i] = i2c_smbus_read_byte_data(client, i);

	return RTN_SUCC;
}
*/
static void onoff_sensor(struct almf04_data *data, int onoff_mode)
{
	int nparse_mode;

	nparse_mode = onoff_mode;
	PINFO("ALMF04 onoff_sensor: nparse_mode [%d]",nparse_mode);

	if (nparse_mode == ENABLE_SENSOR_PINS) {
#if 1 // fixed bug to support cap sensor HAL
		input_report_abs(data->input_dev_cap, ABS_DISTANCE, 3);/* Initializing input event */
		input_sync(data->input_dev_cap);
#endif
		gpio_set_value(data->platform_data->chip_enable, 0);    /*chip_en pin - low : on, high : off*/
		mdelay(500);
		if (!on_sensor)
			enable_irq_wake(data->irq);
		on_sensor = true;
#if 0 // fixed bug to support cap sensor HAL
		if (gpio_get_value(data->platform_data->irq_gpio)) {
			input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* force FAR detection */
			input_sync(data->input_dev_cap);
		}
#endif
	}
	if (nparse_mode == DISABLE_SENSOR_PINS) {
		gpio_set_value(data->platform_data->chip_enable, 1);    /*chip_en pin - low : on, high : off*/
		if (on_sensor)
			disable_irq_wake(data->irq);
		on_sensor = false;
	}
}

//=========================================================================================================================================//
//[200506] ADS Change
//[START]==================================================================================================================================//
//static unsigned char load_firmware(struct almf04_data *data, struct i2c_client *client, const char *name)
//{
//	const struct firmware *fw = NULL;
//	unsigned char rtn;
//	int ret, i, count = 0;
//	int max_page;
//	unsigned short main_version, sub_version;
//	unsigned char page_addr[2];
//	unsigned char page_num;
//	unsigned char fw_version, ic_fw_version;
//	int version_addr;
//	int chip_id = -1;
//	int restore = 0;
//	unsigned char sys_status = 1;
//
//	PINFO("ALMF04 Load Firmware Entered");
//
//	gpio_set_value(data->platform_data->chip_enable, 1);
//	mdelay(50);
//	gpio_set_value(data->platform_data->chip_enable, 0);
//
//	ret = request_firmware(&fw, name, &data->client->dev);
//	if (ret)
//	{
//		PINFO("ALMF04 Unable to open bin [%s]  ret %d", name, ret);
//		if (fw) release_firmware(fw);
//		return 1;
//	}
//	else
//		PINFO("ALMF04 Open bin [%s] ret : %d ", name, ret);
//
//
//	max_page = (fw->size)/SZ_PAGE_DATA;
//	version_addr = (fw->size)-SZ_PAGE_DATA;
//	fw_version = MK_INT(fw->data[version_addr], fw->data[version_addr+1]);
//	page_num = fw->data[version_addr+3];
//	PINFO("###########fw version : %d.%d, fw_version : %d, page_num : %d###########", fw->data[version_addr], fw->data[version_addr+1], fw_version, page_num);
//
//	mdelay(50);
//
//	chip_id= i2c_smbus_read_byte_data(client, 0x00);
//	PINFO("ALMF04 SAR controller ALMF04 chip_id = 0x%x\n", chip_id);
//
//	main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
//	sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
//	ic_fw_version = MK_INT(main_version, sub_version);
//	PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);
//
//	if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version))
//	{
//		PINFO("ALMF04 update firmware");
//
//		mdelay(500);
//		check_firmware_ready(client);
//		sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
//
//		if (sub_version == 0) {
//			restore = 0;
//			PINFO("skip cal backup");
//		} else {
//			if (Backup_CalData(client) < 0) restore = 0;
//			else							restore = 1;
//		}
//
//		/* IC Download Mode Change */
//		chg_mode(ON, client);
//
//		/* firmware write process */
//		rtn = erase_eflash(client);
//		if(rtn != RTN_SUCC) {
//			PINFO("earse fail\n");
//			if (fw) release_firmware(fw);
//			return rtn;		//earse fail
//		}
//
//		//============================================================//
//		//[191014] ADS Change
//		//[START]=====================================================//
//		/*
//		   while(count < page_num) {
//		   page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
//		   page_addr[0] = (unsigned char)(count & 0x00FF);
//		   rtn = write_eflash_page(FLAG_FW, page_addr, fw->data + (count * SZ_PAGE_DATA), client);
//		   if(rtn == RTN_TIMEOUT) {
//		   if (fw) release_firmware(fw);
//		   return RTN_TIMEOUT;
//		   }
//		   count++;
//		   }
//		 */
//		while(count < page_num)
//		{
//			for(i=0; i < SZ_PAGE_DATA; i++) i2c_smbus_write_byte_data(client, i, fw->data[i + (count*SZ_PAGE_DATA)]);
//
//			page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
//			page_addr[0] = (unsigned char)(count & 0x00FF);
//
//			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
//			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_WRITE));
//			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
//
//			if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
//			{
//				if (fw) release_firmware(fw);
//				return RTN_TIMEOUT;
//			}
//			count++;
//		}
//		//[END]=====================================================//
//
//		chg_mode(OFF, client);
//
//		gpio_set_value(data->platform_data->chip_enable, 1);
//		mdelay(50);
//		gpio_set_value(data->platform_data->chip_enable, 0);
//		mdelay(50);
//
//		main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
//		sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
//		ic_fw_version = MK_INT(main_version, sub_version);
//		PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);
//
//		if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version))
//		{
//			PINFO("ATMF04 Firmware update failed.(ic version : %d.%d)", main_version, sub_version);
//			if (fw) release_firmware(fw);
//			return 3;
//		}
//		else
//			PINFO("ATMF04 Firmware update is done");
//	}
//	else {
//		PINFO("Not update firmware. Firmware version is lower than ic.(Or same version)");
//	}
//
//#if defined(CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY)
//	Update_Sensitivity(data, client);
//#endif
//
//	gpio_set_value(data->platform_data->chip_enable, 1);
//	mdelay(50);
//
//	if(restore)
//	{
//		ret = RestoreProc_CalData(data, client);
//		if(ret) PINFO("restore fail ret: %d",ret);
//	}
//
//	PINFO("disable ok");
//	release_firmware(fw);
//
//	return 0;
//}

static unsigned char load_firmware(struct almf04_data *data, struct i2c_client *client, const char *name)
{
	const struct firmware *fw = NULL;
	unsigned char rtn;
	int ret, i, count = 0;
	int max_page;
	unsigned short main_version, sub_version;
	unsigned char page_addr[2];
	unsigned char page_num;
	int version_addr;
	int restore = 0;
	unsigned char sys_status = 1;
	unsigned char rdata, vfy_fail;
	int retry_cnt = 0;

	PINFO("ALMF04 Load Firmware Entered");

	gpio_set_value(data->platform_data->chip_enable, 1);
	mdelay(50);
	gpio_set_value(data->platform_data->chip_enable, 0);

	ret = request_firmware(&fw, name, &data->client->dev);
	if (ret)
	{
		PINFO("ALMF04 Unable to open bin [%s]  ret %d", name, ret);
		if (fw) release_firmware(fw);
		return 1;
	}
	else
		PINFO("ALMF04 Open bin [%s] ret : %d ", name, ret);

	max_page = (fw->size)/SZ_PAGE_DATA;
	version_addr = (fw->size)-SZ_PAGE_DATA;
	page_num = fw->data[version_addr+3];
	PINFO("###########fw version : %d.%d, page_num : %d###########", fw->data[version_addr], fw->data[version_addr+1], page_num);

	mdelay(50);

	main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
	sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);

	//============================================================//
	//[200526] ADS Add
	//[START]=====================================================//
	if((sub_version == 0x00) || (sub_version == 0xB4))
	{
		for(retry_cnt = 0 ; retry_cnt < 5 ; retry_cnt++)
		{
			main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
			sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);

			if((sub_version != 0x00) && (sub_version != 0xB4)) break;
		}
	}
	//[END]=====================================================//
	PINFO("###########ic version : %d.%d, ###########", main_version, sub_version);

	if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version))
	{
		PINFO("ALMF04 update firmware");

		mdelay(500);
		check_firmware_ready(client);
		sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

		//if ((sub_version == 0) || (get_bit(sys_status, 3) == 1) || ((sys_status & 0x06) == 0x06) || ((sys_status & 0x06) == 0x00) ) {
		if ((sub_version == 0) || (sub_version == 0xB4) || (get_bit(sys_status, 3) == 1) || ((sys_status & 0x06) == 0x06) || ((sys_status & 0x06) == 0x00) ) {
			restore = 0;
			PINFO("skip cal backup");
		} else {
			if (Backup_CalData(client) < 0) restore = 0;
			else							restore = 1;
		}

		/* IC Download Mode Change */
		chg_mode(ON, client);

		//FW Download & Verify
		for(retry_cnt = 0 ; retry_cnt < 3 ; retry_cnt++)
		{
			//rom erase
			rtn = erase_eflash(client);
			if(rtn != RTN_SUCC) {
				PINFO("earse fail\n");
				if (fw) release_firmware(fw);
				return rtn;		//earse fail
			}

			//fw download
			while(count < page_num) {
				for(i=0; i < SZ_PAGE_DATA; i++) {
					i2c_smbus_write_byte_data(client, i, fw->data[i + (count*SZ_PAGE_DATA)]);
				}

				page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
				page_addr[0] = (unsigned char)(count & 0x00FF);

				i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
				i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_WRITE));
				i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);

				if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) {
					if (fw) release_firmware(fw);
					return RTN_TIMEOUT;
				}
				count++;
			}

			//fw verify
			count = 0;
			vfy_fail = 0;
			while(count < page_num) {
				page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
				page_addr[0] = (unsigned char)(count & 0x00FF);

				i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
				i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_READ));
				i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);

				if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) {
					if (fw) release_firmware(fw);
					return RTN_TIMEOUT;
				}

				for(i = 0; i < SZ_PAGE_DATA; i++) {
					rdata = i2c_smbus_read_byte_data(client, i);
					//Verify fail
					if(rdata != fw->data[i + (count*SZ_PAGE_DATA)]) {
						PINFO("verify failed!! [retry:%d][%d:(%d,%d)]......", retry_cnt, i + (count*SZ_PAGE_DATA), rdata, fw->data[i + (count*SZ_PAGE_DATA)]);
						vfy_fail = 1;
					}
				}

				if(vfy_fail == 1) break;
				count++;
			}

			//verify success
			if(vfy_fail == 0) {
				PINFO("verify success [retry:%d]", retry_cnt);
				break;
			}
		}

		chg_mode(OFF, client);

		gpio_set_value(data->platform_data->chip_enable, 1);
		mdelay(50);
		gpio_set_value(data->platform_data->chip_enable, 0);
		mdelay(50);

		main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
		sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);

		if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version)) {
			PINFO("ATMF04 Firmware update failed.(ic version : %d.%d)", main_version, sub_version);
			if (fw) release_firmware(fw);
			return 3;
		}
		else
			PINFO("ATMF04 Firmware update is done");
	}
	else {
		PINFO("Not update firmware. Firmware version is the same as IC.");
	}

#if defined(CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY)
	Update_Sensitivity(data, client);
#endif

	gpio_set_value(data->platform_data->chip_enable, 1);
	mdelay(50);

	if(restore)
	{
		ret = RestoreProc_CalData(data, client);
		if(ret) PINFO("restore fail ret: %d",ret);
	}

	PINFO("disable ok");
	release_firmware(fw);

	return 0;
}
//[END]====================================================================================================================================//

static int write_initcode(struct i2c_client *client)
{
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop;
	int en_state;
	int ret = 0;
	unsigned char sys_status = 0;

	PINFO("write_initcode entered[%d]",data->platform_data->irq_gpio);

	en_state = gpio_get_value(data->platform_data->chip_enable);

	if (en_state)
		gpio_set_value(data->platform_data->chip_enable, 0);

	mdelay(50);
	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
	if((get_bit(sys_status, 0) == 0) && (get_bit(sys_status, 1) == 1))
		i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);			//SW Reset1

	mdelay(500);

	check_firmware_ready(client);

	for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
		ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
		if (ret) {
			PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
			return ret;
		}
		PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
	}

	return 0;
}

static bool valid_multiple_input_pins(struct almf04_data *data)
{
	if (data->platform_data->input_pins_num > 1)
		return true;

	return false;

}

static int write_calibration_data(struct almf04_data *data, char *filename)
{
	int fd = 0;

	char file_result[5] = {0,}; // debugging calibration paused
	mm_segment_t old_fs = get_fs();

	PINFO("write_calibration_data Entered[%d]",data->platform_data->irq_gpio);
	set_fs(KERNEL_DS);
	fd = ksys_open(filename, O_WRONLY|O_CREAT, 0664);

	if (fd >= 0) {
#if 1 // debugging calibration paused
		if(cal_result) {
			strncpy(file_result, CAP_CAL_RESULT_PASS, strlen(CAP_CAL_RESULT_PASS));
			ksys_write(fd, file_result, sizeof(file_result));
		} else {
			strncpy(file_result, CAP_CAL_RESULT_FAIL, strlen(CAP_CAL_RESULT_FAIL));
			ksys_write(fd, file_result, sizeof(file_result));
		}
		PINFO("%s: write [%s] to %s, cal_result %d", __FUNCTION__, file_result, filename, cal_result);
		ksys_close(fd);
		set_fs(old_fs);
#else
		ksys_write(fd,0,sizeof(int));
		ksys_close(fd);
#endif
	} else {
		PINFO("%s: %s open failed [%d]......", __FUNCTION__, filename, fd);
	}

	//PINFO("sys open to save cal.dat");

	return 0;
}

static void check_firmware_ready(struct i2c_client *client)
{
	unsigned char sys_status = 1;
	int i;

	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
	for(i = 0 ; i < 100 ; i++) {
		if (get_bit(sys_status, 0) == 1) {
			PINFO("%s: Firmware is busy now.....[%d] sys_status = [0x%x]", __FUNCTION__, i, sys_status);
			mdelay(10);
			sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
		} else {
			break;
		}
	}
	PINFO("%s: sys_status = [0x%x]", __FUNCTION__, sys_status);
	return;
}

static void check_init_touch_ready(struct i2c_client *client)
{
	unsigned char init_touch_md_check = 0;
	int i;

	init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
	for(i = 0 ; i < 50 ; i++) {
		if ((get_bit(init_touch_md_check, 2) == 0) && (get_bit(init_touch_md_check, 1) == 0)) {
			PINFO("%s: Firmware init touch is not yet ready.....[%d] sys_statue = [0x%x]", __FUNCTION__, i, init_touch_md_check);
			mdelay(10);
			init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
		} else {
			break;
		}
	}
	PINFO("%s: sys_status = [0x%x]", __FUNCTION__, init_touch_md_check);

	return;

}

static ssize_t almf04_show_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	int loop;
	char buf_line[256] = "";
	int nlength = 0;
	char buf_regproxdata[512] = "";
	client = data->client;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		memset(buf_line, 0, sizeof(buf_line));
		sprintf(buf_line, "[0x%x:0x%x]", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		PINFO("###### [0x%x][0x%x]###", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		nlength = strlen(buf_regproxdata);
		strcpy(&buf_regproxdata[nlength], buf_line);
	}

	return sprintf(buf,"%s\n", buf_regproxdata);
}

static ssize_t almf04_store_read_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	int loop;
	int ret = 0;
	unsigned long val;
	bool flag_match = false;

	client = data->client;
	PINFO("input :%s", buf);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		PINFO("###### loop:%d [0x%x][0x%x]###", loop,InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		if( val ==  InitCodeAddr[loop])
		{
			PINFO("match Addr :%s", buf);
			flag_match = true;
			break;
		}
	}

	if(loop > MAX_CNT_INITCODE)
	{
		PINFO("Array index out of bounds");
		return -EINVAL;
	}

	if(flag_match)
	{
		ret = i2c_smbus_read_byte_data(client, InitCodeAddr[loop]);
		PINFO("read register [0x%x][0x%x]",InitCodeAddr[loop],i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
	}
	else
	{
		PINFO("match Addr fail");
	}

	return count;
}

static ssize_t almf04_store_write_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop;
	bool flag_match = false;
	unsigned int addr, val;
	client = data->client;

	PINFO("input :%s", buf);

	if (sscanf(buf, "%x %x", &addr, &val) <= 0)
		return count;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		PINFO("###### [0x%x][0x%x]###", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		if( addr ==  InitCodeAddr[loop]) {
			PINFO("match Addr :%s", buf);
			flag_match = true;
			break;
		}
	}

	if(loop > MAX_CNT_INITCODE)
	{
		PINFO("Array index out of bounds");
		return -EINVAL;
	}


	if(flag_match) {
		i2c_smbus_write_byte_data(client, InitCodeAddr[loop], val);
		PINFO("write register ##[0x%x][0x%x]##", InitCodeAddr[loop], val);
		i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x04);
		//============================================================//
		//[191125] ADS Change
		//[START]=====================================================//
		//mdelay(50);		//50ms delay
		check_firmware_ready(client);
		//[END]=======================================================//
		i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
		// return sprintf(buf,"0x%02x\n",i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		return count;
	}
	else
		return count;
}

static ssize_t almf04_show_regproxctrl0(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	PINFO("almf04_show_regproxctrl0: %d\n",on_sensor);
	if(on_sensor==true)
		return sprintf(buf,"0x%02x\n",0x0C);
	return sprintf(buf,"0x%02x\n",0x00);
}

static ssize_t almf04_store_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop;
	client = data->client;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
		PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], InitCodeVal[loop]);
	}
	i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
	return count;
}

static ssize_t almf04_show_proxstatus(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret ;
	struct almf04_data *data = dev_get_drvdata(dev);
	ret = gpio_get_value(data->platform_data->irq_gpio);
	return sprintf(buf, "%d\n", ret);
}


static ssize_t almf04_store_onoffsensor(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct almf04_data *data = dev_get_drvdata(dev);
	unsigned long val;
	struct i2c_client *client = to_i2c_client(dev);
	client = data->client;
	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if(val == ON_SENSOR) {
		onoff_sensor(data,ENABLE_SENSOR_PINS);
		PINFO("Store ON_SENSOR[%d]",data->platform_data->irq_gpio);
	}
	else if (val == OFF_SENSOR) {
		PINFO("Store OFF_SENSOR[%d]",data->platform_data->irq_gpio);
		onoff_sensor(data,DISABLE_SENSOR_PINS);
	}
	return count;
}

static ssize_t almf04_show_docalibration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char  safe_duty;
	client = data->client;

	/* check safe duty for validation of cal*/
	safe_duty = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	return sprintf(buf, "%d\n", (get_bit(safe_duty, 7) == 0));
}

static ssize_t almf04_store_docalibration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	int ret, init_state;
	unsigned char  safe_duty;
	unsigned char loop, rd_val;

	client = data->client;

	PINFO("almf04_store_docalibration[%d]",data->platform_data->irq_gpio);
	init_state = write_initcode(client);
	PINFO("write_initcode End[%d]",data->platform_data->irq_gpio);

#if 1 // debugging calibration paused
	if(init_state) {
		PINFO("%s: write_initcode result is failed.....\n", __FUNCTION__);
		cal_result = false;
		write_calibration_data(data, PATH_CAPSENSOR_CAL);
		return count;
	} else {
		PINFO("%s: write_initcode result is successful..... continue next step!!!\n", __FUNCTION__);
	}
#else
	if(init_state)
		return count;
#endif

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x0C);
	mutex_unlock(&data->update_lock);
	if(ret)
		PINFO("i2c_write_fail\n");

	mdelay(500);
	check_firmware_ready(client);

	gpio_set_value(data->platform_data->chip_enable, 1);
	mdelay(10);
	gpio_set_value(data->platform_data->chip_enable, 0);

	mdelay(50);
	check_firmware_ready(client);

	safe_duty = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	PINFO("[%u]safe_duty : %d", data->platform_data->irq_gpio, (get_bit(safe_duty, 7) == 0));

#if 1 // debugging calibration paused
	if(ret || (get_bit(safe_duty, 7) == 1)) {
		// There is i2c failure or safe duty bit is not 1
		PINFO("%s: calibration result is failed.....\n", __FUNCTION__);
		cal_result = false;
	} else {
		PINFO("%s: calibration result is successful!!!!!\n", __FUNCTION__);
		cal_result = true;
	}
#endif

	for(loop = 0 ; loop < CNT_INITCODE ; loop++)
	{
		rd_val = i2c_smbus_read_byte_data(client, InitCodeAddr[loop]);

		if( (InitCodeAddr[loop] == I2C_ADDR_SFDT1_MIN) || (InitCodeAddr[loop] == I2C_ADDR_SFDT1_MAX) ||
			(InitCodeAddr[loop] == I2C_ADDR_SFDT2_MIN) || (InitCodeAddr[loop] == I2C_ADDR_SFDT2_MAX) ||
			(InitCodeAddr[loop] == I2C_ADDR_SFDT3_MIN) || (InitCodeAddr[loop] == I2C_ADDR_SFDT3_MAX) )
			rd_val = rd_val & 0x7F;

		if (InitCodeVal[loop] != rd_val)
		{
			PINFO("%s: Invalid Init Code[Addr:0x%x][Save Value:0x%x][Read Value:0x%x]", __FUNCTION__,InitCodeAddr[loop], InitCodeVal[loop], rd_val);
			cal_result = false;
		}
	}

	write_calibration_data(data, PATH_CAPSENSOR_CAL);

	return count;
}

static ssize_t almf04_store_regreset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop, rdata[2];
	int duty_val[CNT_MAX_CH], per_result[(CNT_MAX_CH -1)];

	int ret;
	client = data->client;

	check_firmware_ready(client);

#if 0 // debugging calibration paused   /* 2019.04.01, LGE want to keep the result of calibration */
	// Whether cal is pass or fail, make it cal_result true to check raw data/CS/CR/count in bypass mode of AAT
	cal_result = true;
	write_calibration_data(data, PATH_CAPSENSOR_CAL);
#endif

	ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x02);
	if(ret)
		PINFO("[%d]i2c_write_fail\n",data-> platform_data->irq_gpio);

	//Percent Result
	for(loop = 0 ; loop < (CNT_MAX_CH -1) ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_CH1_PER_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_CH1_PER_L + (loop*2));
		//============================================================//
		//[191018] ADS Change
		//[START]=====================================================//
		//per_result[loop] = MK_INT(rdata[1], rdata[0]);
		per_result[loop] = MK_INT(rdata[0], rdata[1]);
		//[END]=======================================================//
		per_result[loop] /= 8;
	}

	//Duty
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L + (loop*2));
		//============================================================//
		//[191018] ADS Change
		//[START]=====================================================//
		//duty_val[loop] = MK_INT(rdata[1], rdata[0]);
		duty_val[loop] = MK_INT(rdata[0], rdata[1]);
		//[END]=======================================================//
	}

	#ifdef ALMF04_3CH_SAR
		PINFO("[%d] Result: %2d %2d %2d %6d %6d %6d %6d", data-> platform_data->irq_gpio, 
			per_result[0], per_result[1], per_result[2], duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
	#else
		PINFO("[%d] Result: %2d %2d  0 %6d %6d %6d       0", data->platform_data->irq_gpio,
			per_result[0], per_result[1], duty_val[0], duty_val[1], duty_val[2]);
	#endif

	return count;
}

static int get_bit(unsigned short x, int n) {
	return (x & (1 << n)) >> n;
}

/*
 static short get_abs(short x) {
	return ((x >= 0) ? x : -x);
}
*/
static ssize_t almf04_show_regproxdata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop, rdata[2];
	//============================================================//
	//[191118] ADS Change
	//[START]=====================================================//
	//int duty_val[CNT_MAX_CH], per_result[(CNT_MAX_CH -1)];
	int duty_val[CNT_MAX_CH];
	short per_result[(CNT_MAX_CH -1)];
	//[END]=======================================================//
	int nlength = 0;
	char buf_regproxdata[256] = "";
	char buf_line[256] = "";
	unsigned char init_touch_md;

	client = data->client;
	memset(buf_line, 0, sizeof(buf_line));
	memset(buf_regproxdata, 0, sizeof(buf_regproxdata));

	check_firmware_ready(client); //workaound code for calibration fail

	init_touch_md = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	//Percent Result
	for(loop = 0 ; loop < (CNT_MAX_CH -1) ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_CH1_PER_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_CH1_PER_L + (loop*2));
		//============================================================//
		//[191018] ADS Change
		//[START]=====================================================//
		//per_result[loop] = MK_INT(rdata[1], rdata[0]);
		per_result[loop] = MK_INT(rdata[0], rdata[1]);
		//[END]=======================================================//
		per_result[loop] /= 8;
	}

	//Duty
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L + (loop*2));
		//============================================================//
		//[191018] ADS Change
		//[START]=====================================================//
		//duty_val[loop] = MK_INT(rdata[1], rdata[0]);
		duty_val[loop] = MK_INT(rdata[0], rdata[1]);
		//[END]=======================================================//
	}

	//============================================================//
	//[191018] ADS Change2
	//[START]=====================================================//
	/*#ifdef 3CH_SAR
		sprintf(buf_line, "[R]%6d %6d %6d %6d %6d %6d %6d %6d\n",
			get_bit(init_touch_md, 3), per_result[0], per_result[1], per_result[2], duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
	#else
		sprintf(buf_line, "[R]%6d %6d %6d      0 %6d %6d %6d      0\n",
			get_bit(init_touch_md, 3), per_result[0], per_result[1], duty_val[0], duty_val[1], duty_val[2]);
	#endif*/

	#ifdef ALMF04_3CH_SAR
		sprintf(buf_line, "[R]%6d %6d %6d %6d %6d %6d %6d %6d\n",
			(get_bit(init_touch_md, 3) == 0), per_result[0], per_result[1], per_result[2], duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
	#else
		sprintf(buf_line, "[R]%6d %6d %6d      0 %6d %6d %6d      0\n",
			(get_bit(init_touch_md, 3) == 0), per_result[0], per_result[1], duty_val[0], duty_val[1], duty_val[2]);
	#endif
	//[END]=======================================================//


	nlength = strlen(buf_regproxdata);
	strcpy(&buf_regproxdata[nlength], buf_line);

	return sprintf(buf, "%s", buf_regproxdata);
}

static ssize_t almf04_store_checkallnear(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val == 0)
		check_allnear = false;
	else if (val == 1)
		check_allnear = true;

	printk("almf04_store_checkallnear %d\n",check_allnear);
	return count;
}

static ssize_t almf04_show_count_inputpins(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count_inputpins = 0;

	struct almf04_data *data = dev_get_drvdata(dev);

	count_inputpins = data->platform_data->input_pins_num;
	if (count_inputpins > 1) {
		if (valid_multiple_input_pins(data) == false)
			count_inputpins = 1;
	}
	return sprintf(buf, "%d\n", count_inputpins);
}

static ssize_t almf04_store_firmware(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	const char *fw_name = NULL;
	struct almf04_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	client = data->client;

	fw_name = data->platform_data->fw_name;
	load_firmware(data, client, fw_name);
	return count;
}

static ssize_t almf04_show_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char ic_fw_version;
	unsigned short main_version, sub_version;
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = dev_get_drvdata(dev);
	char buf_line[256] = "";
	int nlength = 0;
	char buf_regproxdata[256] = "";
	client = data->client;

	memset(buf_line, 0, sizeof(buf_line));
	onoff_sensor(data,ENABLE_SENSOR_PINS);

	mdelay(200);

	main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
	sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
        ic_fw_version = MK_INT(main_version, sub_version);
        PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);

	sprintf(buf_line, "%d.%d\n",main_version, sub_version);
	nlength = strlen(buf_regproxdata);
	strcpy(&buf_regproxdata[nlength], buf_line);

	return sprintf(buf,"%s", buf_regproxdata);
}

static ssize_t almf04_show_check_far(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop, rdata[2];
	int duty_val[CNT_MAX_CH];
	unsigned char sys_status;
	unsigned int check_result = 0;
	int poor_contact = 0;

	client = data->client;
	mutex_lock(&data->update_lock);
	check_init_touch_ready(client);

	//Duty
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L + (loop*2));
		//============================================================//
		//[191018] ADS Change
		//[START]=====================================================//
		//duty_val[loop] = MK_INT(rdata[1], rdata[0]);
		duty_val[loop] = MK_INT(rdata[0], rdata[1]);
		//[END]=======================================================//
	}

	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	/* Check poor contact */
#ifdef ALMF04_3CH_SAR
	if( (get_bit(sys_status, 4) == 1) || (get_bit(sys_status, 5) == 1) || (get_bit(sys_status, 6) == 1) )
#else
		if ((get_bit(sys_status, 4) == 1) || (get_bit(sys_status, 5) == 1))
#endif
		{
			poor_contact = 1;
			check_result += 1;
		}

	/* Calibration Check */
	if(get_bit(sys_status, 3) == 1)
		check_result += 1;

	/* Unsafe Duty Range Check */
	if(get_bit(sys_status, 7) == 1){
		check_result += 1;
		PINFO("fail duty range");
	}

	if(gpio_get_value(data->platform_data->irq_gpio) == 1)
	{
		PINFO(" ALMF04 Check_Far IRQ Status NEAR[%d]\n",gpio_get_value(data->platform_data->irq_gpio));
		check_result += 1;
	}

	//============================================================//
	//[191018] ADS Change2
	//[START]=====================================================//
	/*if (check_result > 0)
	{
		#ifdef 3CH_SAR
			PINFO(" ALMF04 [fail] 1.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
					get_bit(sys_status, 3), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
		#else
			PINFO(" ALMF04 [fail] 1.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d\n",
				get_bit(sys_status, 3), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2]);
		#endif
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",0);
	}
	else
	{
		#ifdef 3CH_SAR
			PINFO(" ALMF04 [PASS] 2.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
					get_bit(sys_status, 3), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
		#else
			PINFO(" ALMF04 [PASS] 2.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d\n",
				get_bit(sys_status, 3), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2]);
		#endif
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",1);
	}*/

	if (check_result > 0)
	{
#ifdef ALMF04_3CH_SAR
		PINFO(" ALMF04 [fail] 1.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
				(get_bit(sys_status, 3) == 0), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
#else
		PINFO(" ALMF04 [fail] 1.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d\n",
				(get_bit(sys_status, 3) == 0), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2]);
#endif
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",0);
	}
	else
	{
#ifdef ALMF04_3CH_SAR
		PINFO(" ALMF04 [PASS] 2.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
				(get_bit(sys_status, 3) == 0), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
#else
		PINFO(" ALMF04 [PASS] 2.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d\n",
				(get_bit(sys_status, 3) == 0), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2]);
#endif
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",1);
	}
	//[END]=======================================================//
}

static ssize_t almf04_show_check_mid(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);

	unsigned char loop, rdata[2];
	int duty_val[CNT_MAX_CH];
	unsigned char sys_status;
	unsigned int check_result = 0;
	int poor_contact = 0;

	client = data->client;
	mutex_lock(&data->update_lock);
	check_init_touch_ready(client);

	//Duty
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L + (loop*2));
		//============================================================//
		//[191018] ADS Change
		//[START]=====================================================//
		//duty_val[loop] = MK_INT(rdata[1], rdata[0]);
		duty_val[loop] = MK_INT(rdata[0], rdata[1]);
		//[END]=======================================================//
	}

	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	/* Calibration Check */
	if(get_bit(sys_status, 3) == 1)
		check_result += 1;

	/* Unsafe Duty Range Check */
	if(get_bit(sys_status, 7) == 1)
		check_result += 1;

	/* Check poor contact */
#ifdef ALMF04_3CH_SAR
	if ((get_bit(sys_status, 4) == 1) || (get_bit(sys_status, 5) == 1) || (get_bit(sys_status, 6) == 1))
#else
		if ((get_bit(sys_status, 4) == 1) || (get_bit(sys_status, 5) == 1))
#endif
		{
			poor_contact = 1;
			check_result += 1;
		}

	//============================================================//
	//[191018] ADS Change2
	//[START]=====================================================//
	/*if (check_result > 0)
	{
		#ifdef 3CH_SAR
			PINFO(" ALMF04 [fail] 1.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
				get_bit(sys_status, 3), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
		#else
			PINFO(" ALMF04 [fail] 1.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d\n",
				get_bit(sys_status, 3), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2]);
		#endif
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",0);
	}
	else
	{
		#ifdef 3CH_SAR
			PINFO(" ALMF04 [PASS] 2.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
				get_bit(sys_status, 3), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
		#else
			PINFO(" ALMF04 [PASS] 2.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d\n",
				get_bit(sys_status, 3), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2]);
		#endif
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",1);
	}*/
	if (check_result > 0)
	{
#ifdef ALMF04_3CH_SAR
		PINFO(" ALMF04 [fail] 1.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
				(get_bit(sys_status, 3) == 0), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
#else
		PINFO(" ALMF04 [fail] 1.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d\n",
				(get_bit(sys_status, 3) == 0), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2]);
#endif
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",0);
	}
	else
	{
#ifdef ALMF04_3CH_SAR
		PINFO(" ALMF04 [PASS] 2.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
				(get_bit(sys_status, 3) == 0), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
#else
		PINFO(" ALMF04 [PASS] 2.cal: %d, safe_duty: %d, poor_contact: %d, cr: %d, cs1: %d, cs2: %d\n",
				(get_bit(sys_status, 3) == 0), (get_bit(sys_status, 7) == 0), poor_contact, duty_val[0], duty_val[1], duty_val[2]);
#endif
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",1);
	}
}

static DEVICE_ATTR(onoff,        0664, NULL, almf04_store_onoffsensor);
static DEVICE_ATTR(proxstatus,   0664, almf04_show_proxstatus, NULL);
static DEVICE_ATTR(docalibration,0664, almf04_show_docalibration, almf04_store_docalibration);
static DEVICE_ATTR(reg_ctrl,     0664, almf04_show_reg, almf04_store_reg);
static DEVICE_ATTR(write_reg,    0664, NULL, almf04_store_write_reg);
static DEVICE_ATTR(read_reg,     0664, NULL, almf04_store_read_reg);
static DEVICE_ATTR(regproxdata,  0664, almf04_show_regproxdata, NULL);
static DEVICE_ATTR(regreset,     0664, NULL, almf04_store_regreset);
static DEVICE_ATTR(checkallnear, 0664, NULL, almf04_store_checkallnear);
static DEVICE_ATTR(cntinputpins, 0664, almf04_show_count_inputpins, NULL);
static DEVICE_ATTR(regproxctrl0, 0664, almf04_show_regproxctrl0, NULL);
static DEVICE_ATTR(download,     0664, NULL, almf04_store_firmware);
static DEVICE_ATTR(version,      0664, almf04_show_version, NULL);
static DEVICE_ATTR(check_far,    0664, almf04_show_check_far, NULL);
static DEVICE_ATTR(check_mid,    0664, almf04_show_check_mid, NULL);


static struct attribute *almf04_attributes[] = {
	&dev_attr_onoff.attr,
	&dev_attr_docalibration.attr,
	&dev_attr_proxstatus.attr,
	&dev_attr_reg_ctrl.attr,
	&dev_attr_write_reg.attr,
	&dev_attr_read_reg.attr,
	&dev_attr_regproxdata.attr,
	&dev_attr_regreset.attr,
	&dev_attr_checkallnear.attr,
	&dev_attr_cntinputpins.attr,
	&dev_attr_regproxctrl0.attr,
	&dev_attr_download.attr,
	&dev_attr_version.attr,
	&dev_attr_check_far.attr,
	&dev_attr_check_mid.attr,
	NULL,
};

static struct attribute_group almf04_attr_group = {
	.attrs = almf04_attributes,
};

static void almf04_reschedule_work(struct almf04_data *data,
		unsigned long delay)
{
	int ret;
	struct i2c_client *client = data->client;
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
		dev_err(&client->dev, "almf04_reschedule_work : set wake lock timeout!\n");
		pm_wakeup_event(&client->dev,1500);
		//cancel_delayed_work(&data->dwork);
		ret = queue_delayed_work(almf04_workqueue, &data->dwork, delay);
		if (ret < 0) {
			PINFO("ALMF04 queue_work fail, ret = %d", ret);
		}
	} else {
		PINFO("ALMF04 cap sensor enable pin is already high... power off status");
	}
}

/* assume this is ISR */
static irqreturn_t almf04_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct almf04_data *data = i2c_get_clientdata(client);
	int tmp = -1;
	tmp = atomic_read(&data->i2c_status);

	if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
		dev_err(&client->dev,"almf04_interrupt\n");
		almf04_reschedule_work(data, 0);
	}

	return IRQ_HANDLED;
}

static void almf04_work_handler(struct work_struct *work)
{
	struct almf04_data *data = container_of(work, struct almf04_data, dwork.work);
	struct i2c_client *client = data->client;
	int irq_state;

#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	if(probe_end_flag == true) {
#endif
		data->touch_out = i2c_smbus_read_byte_data(client, I2C_ADDR_TCH_OUTPUT);
		irq_state = gpio_get_value(data->platform_data->irq_gpio);
		PINFO("ALMF04 touch_out[%x] irq_state[%d]", data->touch_out, irq_state);

		/* When I2C fail and abnormal status*/
		if (data->touch_out < 0)
		{
			PINFO("ALMF04 I2C Error[%d]", data->touch_out);
			input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
			input_sync(data->input_dev_cap);

			PINFO("ALMF04 [%d]NEAR ",data->platform_data->irq_gpio);
		}
		else
		{
			if (gpio_get_value(data->platform_data->chip_enable) == 1) { // if power off
				PINFO("ALMF04 cap sensor is already power off : touch_out(0x%x)", data->touch_out);
				return;
			}

#ifdef ALMF04_3CH_SAR
			//==============================================================================================//
			/* General Mode Touch */
			//----------------------------------------------------------------------------------------------//
			if(get_bit(data->touch_out, 7) == 0)
			{
				/* FAR */
				if (irq_state == 0 && (data->touch_out == (CH3_FAR | CH2_FAR | CH1_FAR)))
				{
					data->cap_detection = 0;

					input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
					input_sync(data->input_dev_cap);

					PINFO("ALMF04 [%d]FAR ",data->platform_data->irq_gpio);
				}
				/* NEAR */
				else
				{
					data->cap_detection = 1;

					input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
					input_sync(data->input_dev_cap);

					PINFO("ALMF04 [%d]NEAR ",data->platform_data->irq_gpio);
				}
			}
			//==============================================================================================//
			/* Scenario Mode Touch */
			//----------------------------------------------------------------------------------------------//
			else
			{
				/* FAR */
				//if (irq_state == 0 && ((data->touch_out & 0x03) == SNR_FAR))
				if (irq_state == 0 && ((data->touch_out & 0x43) == SNR_FAR))		//Default Near Bit6 --> 1
				{
					data->cap_detection = 0;

					input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
					input_sync(data->input_dev_cap);

					PINFO("ALMF04 [%d]FAR(SM) ",data->platform_data->irq_gpio);
				}
				/* NEAR */
				else
				{
					data->cap_detection = 1;

					input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
					input_sync(data->input_dev_cap);

					PINFO("ALMF04 [%d]NEAR(SM) ",data->platform_data->irq_gpio);
				}
			}
			//==============================================================================================//
#else
			/* FAR */
			if (irq_state == 0 && (data->touch_out == (CH2_FAR | CH1_FAR)))
			{
				data->cap_detection = 0;

				input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
				input_sync(data->input_dev_cap);

				PINFO("ALMF04 [%d]FAR ", data->platform_data->irq_gpio);
			}
			/* NEAR */
			else
			{
				data->cap_detection = 1;

				input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
				input_sync(data->input_dev_cap);

				PINFO("ALMF04 [%d]NEAR ", data->platform_data->irq_gpio);
			}
#endif
			PINFO("ALMF04 [%d] Work Handler done",data->platform_data->irq_gpio);

		}
#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	}
	else{
		PINFO("ALMF04 probe_end_flag = False[%d]",probe_end_flag);
	}
#endif
}

static int sensor_regulator_configure(struct almf04_data *data, bool on)
{
	struct i2c_client *client = data->client;
	struct almf04_platform_data *pdata = data->platform_data;
	int rc;

	if (on == false)
		goto hw_shutdown;

	pdata->vcc_ana = regulator_get(&client->dev, "Adsemicon,vdd_ana");
	if (IS_ERR(pdata->vcc_ana)) {
		rc = PTR_ERR(pdata->vcc_ana);
		dev_err(&client->dev,
				"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->vcc_ana) > 0) {
		rc = regulator_set_voltage(pdata->vcc_ana, pdata->vdd_ana_supply_min,
				pdata->vdd_ana_supply_max);

		if (rc) {
			dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}

	return 0;

error_set_vtg_vcc_ana:
	regulator_put(pdata->vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);

	regulator_put(pdata->vcc_ana);
	regulator_put(pdata->vcc_dig);

	if (pdata->i2c_pull_up) {
		regulator_put(pdata->vcc_i2c);
	}
	return 0;
}

static int sensor_regulator_power_on(struct almf04_data *data, bool on)
{
	struct i2c_client *client = data->client;
	struct almf04_platform_data *pdata = data->platform_data;

	int rc;

	if (on == false)
		goto power_off;
    
	rc = regulator_set_load(pdata->vcc_ana, pdata->vdd_ana_load_ua);
	if (rc < 0) {
		dev_err(&client->dev,
			"Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(pdata->vcc_ana);
	if (rc) {
		dev_err(&client->dev,
				"Regulator vcc_ana enable failed rc=%d\n", rc);
	}
	return 0;

power_off:
	regulator_disable(pdata->vcc_ana);
	if (pdata->i2c_pull_up) {
		regulator_disable(pdata->vcc_i2c);
	}
	return 0;
}

static int sensor_platform_hw_power_on(struct i2c_client *client, bool on)
{
	sensor_regulator_power_on(i2c_get_clientdata(client), on);
	return 0;
}

static int sensor_platform_hw_init(struct i2c_client *client)
{
	struct almf04_data *data = i2c_get_clientdata(client);
	int error;

	error = sensor_regulator_configure(data, true);

	error = gpio_request(data->platform_data->chip_enable, "almf04_chip_enable");
	if(error) {
		PINFO(" ALMF04 chip_enable request fail\n");
	}
	gpio_direction_output(data->platform_data->chip_enable, 0); // First, set chip_enable gpio direction to output.

	PINFO(" ALMF04 gpio direction output ok\n");

	if (gpio_is_valid(data->platform_data->irq_gpio)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(data->platform_data->irq_gpio, "almf04_irq_gpio");
		if (error) {
			PINFO(" ALMF04 unable to request gpio [%d]\n",
					data->platform_data->irq_gpio);
		}
		error = gpio_direction_input(data->platform_data->irq_gpio);
		if (error) {
			PINFO(" ALMF04 unable to set direction for gpio [%d]\n",
					data->platform_data->irq_gpio);
		}
		data->irq = client->irq = gpio_to_irq(data->platform_data->irq_gpio);
	} else {
		PINFO(" ALMF04 irq gpio not provided\n");
	}
	PINFO(" ALMF04 sensor_platform_hw_init end\n");
	return 0;
}

static void sensor_platform_hw_exit(struct i2c_client *client)
{
	struct almf04_data *data = i2c_get_clientdata(client);;

	sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
	PINFO(" ALMF04 sensor_platform_hw_exit entered\n");
}

static int sensor_parse_dt(struct device *dev,
		struct almf04_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	int ret, err = 0;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
#if defined (CONFIG_MACH_SDM845_JUDYPN)
		{"Adsemicon,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
		{"Adsemicon,irq-gpio2",		&pdata->irq_gpio2,		DT_REQUIRED,	DT_GPIO,	0},
#else
		{"Adsemicon,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
#endif
		{"Adsemicon,vdd_ana_supply_min",	&pdata->vdd_ana_supply_min,	DT_SUGGESTED,	DT_U32,		0},
		{"Adsemicon,vdd_ana_supply_max",	&pdata->vdd_ana_supply_max,	DT_SUGGESTED,	DT_U32,		0},
		{"Adsemicon,vdd_ana_load_ua",	&pdata->vdd_ana_load_ua,	DT_SUGGESTED,	DT_U32,		0},
		{"Adsemicon,chip_enable",   &pdata->chip_enable,    DT_SUGGESTED,   DT_GPIO,     0},
		{"Adsemicon,InputPinsNum",         &pdata->input_pins_num,      DT_SUGGESTED,   DT_U32,  0},
		{"Adsemicon,fw_name",              &pdata->fw_name,             DT_SUGGESTED,   DT_STRING,  0},
		/* add */
		{NULL,				NULL,				0,		0,		0},
	};

	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
			case DT_GPIO:
				ret = of_get_named_gpio(np, itr->dt_name, 0);
				if (ret >= 0) {
					*((int *) itr->ptr_data) = ret;
					ret = 0;
				}
				break;
			case DT_U32:
				ret = of_property_read_u32(np, itr->dt_name,
						(u32 *) itr->ptr_data);
				break;
			case DT_BOOL:
				*((bool *) itr->ptr_data) =
					of_property_read_bool(np, itr->dt_name);
				ret = 0;
				break;
			case DT_STRING:
				ret = of_property_read_string(np, itr->dt_name, itr->ptr_data);
				break;
			default:
				PINFO(" ALMF04 %d is an unknown DT entry type\n",
						itr->type);
				ret = -EBADE;
		}

		PINFO(" ALMF04 DT entry ret:%d name:%s val:%d\n",
				ret, itr->dt_name, *((int *)itr->ptr_data));

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				PINFO(" ALMF04 Missing '%s' DT entry\n",
						itr->dt_name);

				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;
	/*pdata->ppcount = 12;	//no need to set, dt_parse */

	return err;

	return 0;
}

static int almf04_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct almf04_data *data;
#ifdef CONFIG_OF
	struct almf04_platform_data *platform_data;
#endif
	int err = 0;

	/* Check for Normal booting mode or not */
	if (lge_get_boot_mode() == LGE_BOOT_MODE_NORMAL) {
		/* Check for support by carrier */
		if (lge_get_capsensor() == 0)
			return -ENODEV;
	} else {
		PINFO("ALMF04 NOT NORMAL BOOT");
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		return -EIO;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct almf04_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}
	PINFO("ALMF04 probe, kzalloc complete\n");

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
				sizeof(struct almf04_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		data->platform_data = platform_data;
		client->dev.platform_data = platform_data;
		err = sensor_parse_dt(&client->dev, platform_data);
		if (err)
			return err;

#if defined(CONFIG_MACH_SDM845_JUDYPN)
		if(strcmp(lge_get_board_subrevision(),"subrev_0"))
		{
			PINFO("**Capsensor BLSP Change [%d] to [%d] for	subrev**",platform_data->irq_gpio, platform_data->irq_gpio2);
			platform_data->irq_gpio = platform_data->irq_gpio2;
		}
#endif
	} else {
		platform_data = client->dev.platform_data;
	}
#endif
	data->client = client;
	almf04_i2c_client = client;
	i2c_set_clientdata(client, data);
	data->cap_detection = 0;

#ifdef CONFIG_OF
	/* h/w initialization */
	if (platform_data->init)
		err = platform_data->init(client);

	if (platform_data->power_on)
		err = platform_data->power_on(client, true);
#endif
	PINFO("**Capsensor BLSP[%d]",platform_data->irq_gpio);

	client->adapter->retries = 15;

	if (client->adapter->retries == 0)
		goto exit;

	atomic_set(&data->i2c_status, ALMF04_STATUS_RESUME);

	mutex_init(&data->update_lock);
	mutex_init(&data->enable_lock);

	INIT_DELAYED_WORK(&data->dwork, almf04_work_handler);

	err = request_irq(client->irq, almf04_interrupt, IRQ_TYPE_EDGE_FALLING|IRQ_TYPE_EDGE_RISING,
				 ALMF04_DRV_NAME, (void *)client);
	if(err) {
		PINFO("ALMF04 chip_enable request fail\n");
		goto exit_irq_init_failed;
	}

	err = enable_irq_wake(data->irq);

	data->input_dev_cap = input_allocate_device();
	if (!data->input_dev_cap) {
		err = -ENOMEM;
		PINFO("ALMF04 Failed to allocate input device cap !\n");
		goto exit_free_irq;
	}

	set_bit(EV_ABS, data->input_dev_cap->evbit);

	input_set_abs_params(data->input_dev_cap, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_cap->name = ALMF04_DRV_NAME;
	data->input_dev_cap->dev.init_name = ALMF04_DRV_NAME;
	data->input_dev_cap->id.bustype = BUS_I2C;

	input_set_drvdata(data->input_dev_cap, data);

	err = input_register_device(data->input_dev_cap);
	if (err) {
		err = -ENOMEM;
		PINFO("ALMF04 Unable to register input device cap(%s)\n",
				data->input_dev_cap->name);
		goto exit_free_dev_cap;
	}

	if (data->platform_data->fw_name) {
		err = load_firmware(data, client, data->platform_data->fw_name);
		if (err) {
			PINFO("ALMF04 Failed to request firmware\n");
			//goto exit_free_irq;
			goto exit_irq_init_failed;
		}
	}
	PINFO("sysfs create start!\n");
	err = sysfs_create_group(&data->input_dev_cap->dev.kobj, &almf04_attr_group);
	if (err)
		PINFO("input sysfs create fail!\n");

	err = sysfs_create_group(&client->dev.kobj, &almf04_attr_group);
    if (err)
		PINFO("ALMF04 sysfs create fail!\n");

	/* default sensor off */
#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	probe_end_flag = true;
#endif
	onoff_sensor(data, DISABLE_SENSOR_PINS);
	PINFO("interrupt is hooked\n");

        atomic_set(&pm_suspend_flag, DEV_PM_RESUME); // initial value

        return 0;

exit_irq_init_failed:
        gpio_direction_output(data->platform_data->chip_enable, 1); // To avoid power leakage when probe fail
        mutex_destroy(&data->update_lock);
        mutex_destroy(&data->enable_lock);
        if(data->input_dev_cap != NULL) {
            input_unregister_device(data->input_dev_cap);
        }
exit_free_dev_cap:
exit_free_irq:
	free_irq(data->irq, client);
exit:
	PINFO("ALMF04 Error");
	return err;
}

static int almf04_remove(struct i2c_client *client)
{

	struct almf04_data *data = i2c_get_clientdata(client);
	struct almf04_platform_data *pdata = data->platform_data;


	disable_irq_wake(client->irq);

	free_irq(client->irq, client);

	if (pdata->power_on)
		pdata->power_on(client, false);

	if (pdata->exit)
		pdata->exit(client);

	mutex_destroy(&data->update_lock);
	mutex_destroy(&data->enable_lock);
	PINFO("ALMF04 remove\n");
	return 0;
}

static const struct i2c_device_id almf04_id[] = {
	{ "almf04", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, almf04_id);

#ifdef CONFIG_OF
static struct of_device_id almf04_match_table[] = {
	{ .compatible = "almf04",},
	{ },
};
#else
#define almf04_match_table NULL
#endif

static int ALMF04_pm_suspend(struct device *dev)
{
	atomic_set(&pm_suspend_flag, DEV_PM_SUSPEND);
	return 0;
}

static int ALMF04_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data  = i2c_get_clientdata(client);
	int ret = 0;

	if (atomic_read(&pm_suspend_flag) == DEV_PM_SUSPEND_IRQ) {
		if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
			PINFO("queue_delayed_work now.");
			ret = queue_delayed_work(almf04_workqueue, &data->dwork, 0);
			if (ret < 0) {
				PINFO("queue_work fail, ret = %d", ret);
			}
		}
		else {
			PINFO("cap sensor enable pin is already high... power off status");
		}
	}

	atomic_set(&pm_suspend_flag, DEV_PM_RESUME);
	return 0;
}

static const struct dev_pm_ops ALMF04_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(
            ALMF04_pm_suspend, //Get call when suspend is happening
            ALMF04_pm_resume //Get call when resume is happening
	)
};
static struct i2c_driver almf04_driver = {
	.driver = {
		.name   = ALMF04_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = almf04_match_table,
        .pm = &ALMF04_pm_ops,
        .probe_type = PROBE_PREFER_ASYNCHRONOUS,
        },
	.probe  = almf04_probe,
	.remove = almf04_remove,
	.id_table = almf04_id,
};
static void async_almf04_init(void *data, async_cookie_t cookie)
{
	PINFO("ALMF04 init Proximity driver: release.\n");
	almf04_workqueue = create_workqueue("capsensor");
	i2c_add_driver(&almf04_driver);
}

static int __init almf04_init(void)
{
    async_schedule(async_almf04_init, NULL);
	return 0;
}

static void __exit almf04_exit(void)
{
	PINFO("ALMF04 Proximity driver: release.\n");
	if (almf04_workqueue)
		destroy_workqueue(almf04_workqueue);

	almf04_workqueue = NULL;

	i2c_del_driver(&almf04_driver);
}

MODULE_DESCRIPTION("ALMF04 cap sensor driver");
MODULE_LICENSE("GPL");

module_init(almf04_init);
module_exit(almf04_exit);


