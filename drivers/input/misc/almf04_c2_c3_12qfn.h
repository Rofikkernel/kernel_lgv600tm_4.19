#ifndef __ALMF04_12QFN_H__
#define __ALMF04_12QFN_H__

#define USE_ALMF04				//ALMF04 Use status
//#define ALMF04_3CH_SAR                	//SAR Sensing Channel 3

#define SZ_PAGE_DATA                64

#define ADDR_EFLA_STS               0xFF	//eflash status register
#define ADDR_EFLA_PAGE_L            0xFD	//eflash page
#define ADDR_EFLA_PAGE_H            0xFE	//eflash page
#define ADDR_EFLA_CTRL              0xFC	//eflash control register

#if defined(USE_ALMF04)

	#define ADDR_ROM_SAFE				0xFB	//eflash write
	#define VAL_ROM_MASK1				0x02
	#define VAL_ROM_MASK2				0x06

	#define EMD_ALL_ERASE				(0x07 << 1)
	#define EMD_PG_ERASE				(0x04 << 1)
	#define EMD_PG_WRITE				(0x08 << 1)
	#define EMD_PG_READ					0x00

	#define CMD_EEP_START				0x01
	#define CMD_EUM_START				0x03

	#define FLAG_DONE                   0x03
	#define FLAG_DONE_ERASE             0x03

#else

	#define CMD_EFL_L_WR                0x01	//Eflash Write
	#define CMD_EFL_RD                  0x03	//Eflash Read
	#define CMD_EFL_ERASE_ALL           0x07	//Eflash All Page Erase

	#define CMD_EUM_WR                  0x21	//Extra user memory write
	#define CMD_EUM_RD                  0x23	//Extra user memory read
	#define CMD_EUM_ERASE               0x25	//Extra user memory erase

	#define FLAG_DONE                   0x03
	#define FLAG_DONE_ERASE             0x02

#endif

#define FLAG_FUSE                   1
#define FLAG_FW                     2

#define FL_EFLA_TIMEOUT_CNT         20
#define IC_TIMEOUT_CNT				5

#define RTN_FAIL                    0
#define RTN_SUCC                    1
#define RTN_TIMEOUT                 2

#define ON                          1
#define OFF                         2

enum {
	DEV_PM_RESUME = 0,
	DEV_PM_SUSPEND,
	DEV_PM_SUSPEND_IRQ,
};

#if 1 // debugging calibration paused
#define CAP_CAL_RESULT_PASS                     "pass"
#define CAP_CAL_RESULT_FAIL                     "fail"
#endif

#endif

//#define CNT_INITCODE					44
//#define IDX_ICODE_ITSSTVT				6
//
//	static const unsigned char InitCodeAddr[CNT_INITCODE]	= { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0X0C, 0X0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E };
//
//#ifdef ALMF04_3CH_SAR
//	static const unsigned char InitCodeVal[CNT_INITCODE]	= { 0x00, 0x52, 0x00, 0x52, 0x00, 0x52, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x0B, 0x00, 0x33, 0x13, 0x13, 0x13, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64, 0x50, 0x85, 0x20, 0x0B, 0x07, 0x0B, 0x07, 0x0B, 0x07, 0x33, 0xFF, 0xFF, 0xFF, 0x01, 0x7F, 0x01,	0x7F, 0x01, 0x7F, 0x9E }; 
//#else
//	static const unsigned char InitCodeVal[CNT_INITCODE]	= { 0x00, 0x52, 0x00, 0x52, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x00, 0x0B, 0x00, 0x33, 0x13, 0x13, 0x00, 0x64, 0x64, 0x64, 0x64, 0x00, 0x00, 0x50, 0x85, 0x20, 0x0B, 0x07, 0x0B, 0x07, 0x00, 0x00, 0x33, 0xFF, 0xFF, 0x00, 0x01, 0x7F, 0x01,	0x7F, 0x00, 0x00, 0x00 }; 
//#endif

#define IDX_ICODE_ITSSTVT				6

#ifdef ALMF04_3CH_SAR
	#define CNT_INITCODE					44
	#define MAX_CNT_INITCODE				CNT_INITCODE-1
	static const unsigned char InitCodeAddr[CNT_INITCODE] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0X0C, 0X0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E };
	static const unsigned char InitCodeVal[CNT_INITCODE] =  { 0x00, 0x52, 0x00, 0x52, 0x00, 0x52, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x0B, 0x00, 0x33, 0x13, 0x13, 0x13, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64, 0x50, 0x85, 0x20, 0x0B, 0x07, 0x0B, 0x07, 0x0B, 0x07, 0x33, 0xFF, 0xFF, 0xFF, 0x01, 0x7F, 0x01, 0x7F, 0x01, 0x7F, 0x9E };
#else /*ALMF04_3CH_SAR*/
#if defined (CONFIG_MACH_KONA_TIMELM)
	#define CNT_INITCODE					31
	#define MAX_CNT_INITCODE				CNT_INITCODE-1
	static const unsigned char InitCodeAddr[CNT_INITCODE] = { 0x01, 0x02, 0x03, 0x04, 0x07, 0x08, 0x09, 0x0A, 0x0D, 0X0E, 0X0F, 0x10, 0x11, 0x14, 0x15, 0x16, 0x17, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2A, 0x2B };
	static const unsigned char InitCodeVal[CNT_INITCODE]  = { 0x00, 0x31, 0x01, 0x9A, 0x00, 0x52, 0x01, 0x9A, 0x07, 0x30, 0x33, 0x13, 0x13, 0xAA, 0x64, 0x82, 0x64, 0xD0, 0x85, 0x20, 0x0C, 0x07, 0x0B, 0x07, 0x33, 0x11, 0xFF, 0x0E, 0x12, 0x06, 0x09 };
#else 	
	#define CNT_INITCODE					31
	#define MAX_CNT_INITCODE				CNT_INITCODE-1
	static const unsigned char InitCodeAddr[CNT_INITCODE] = { 0x01, 0x02, 0x03, 0x04, 0x07, 0x08, 0x09, 0x0A, 0X0D, 0x0E, 0x0F, 0x10, 0x11, 0x14, 0x15, 0x16, 0x17, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2A, 0x2B };
	static const unsigned char InitCodeVal[CNT_INITCODE] =  { 0x00, 0x52, 0x00, 0x52, 0x00, 0x0C, 0x00, 0x0C, 0x0B, 0x00, 0x33, 0x13, 0x13, 0x64, 0x64, 0x64, 0x64, 0x50, 0x85, 0x20, 0x0B, 0x07, 0x0B, 0x07, 0x33, 0xFF, 0xFF, 0x01, 0x7F, 0x01, 0x7F };
#endif /*CONFIG_MACH_KONA_TIMELM*/
#endif /*ALMF04_3CH_SAR*/

/*! ERROR LOG LEVEL */
#define LOG_LEVEL_E 3
/*! NOTICE LOG LEVEL */
#define LOG_LEVEL_N 5
/*! INFORMATION LOG LEVEL */
#define LOG_LEVEL_I 6
/*! DEBUG LOG LEVEL */
#define LOG_LEVEL_D 7

#ifndef LOG_LEVEL
/*! LOG LEVEL DEFINATION */
#define LOG_LEVEL LOG_LEVEL_I
#endif

#ifndef MODULE_TAG
/*! MODULE TAG DEFINATION */
#define MODULE_TAG "<atmf04_12qfn>"
#endif

#if (LOG_LEVEL >= LOG_LEVEL_E)
/*! print error message */
#define PERR(fmt, args...) \
	pr_err(MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PERR(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_N)
/*! print notice message */
#define PNOTICE(fmt, args...) \
	pr_notice(MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PNOTICE(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_I)
/*! print information message */
#define PINFO(fmt, args...) pr_info(MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PINFO(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_D)
/*! print debug message */
#define PDEBUG(fmt, args...) pr_devel(MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PDEBUG(fmt, args...)

/* I2C Register */
#define	I2C_ADDR_SSTVT_CH1_H			0x01
#define	I2C_ADDR_SSTVT_CH1_L			0x02
#define	I2C_ADDR_SSTVT_CH2_H			0x03
#define	I2C_ADDR_SSTVT_CH2_L			0x04
#define	I2C_ADDR_SSTVT_CH3_H			0x05
#define	I2C_ADDR_SSTVT_CH3_L			0x06
#define	I2C_ADDR_IT_SSTVT_CH1_H			0x07
#define	I2C_ADDR_IT_SSTVT_CH1_L			0x08
#define	I2C_ADDR_IT_SSTVT_CH2_H			0x09
#define	I2C_ADDR_IT_SSTVT_CH2_L			0x0A
#define	I2C_ADDR_IT_SSTVT_CH3_H			0x0B
#define	I2C_ADDR_IT_SSTVT_CH3_L			0x0C

#define	I2C_ADDR_PC1_CHK_CNT			0x25
#define	I2C_ADDR_PC2_CHK_CNT			0x26
#define	I2C_ADDR_PC3_CHK_CNT			0x27
#define	I2C_ADDR_SFDT1_MIN				0x28
#define	I2C_ADDR_SFDT1_MAX				0x29
#define	I2C_ADDR_SFDT2_MIN				0x2A
#define	I2C_ADDR_SFDT2_MAX				0x2B
#define	I2C_ADDR_SFDT3_MIN				0x2C
#define	I2C_ADDR_SFDT3_MAX				0x2D
#define	I2C_ADDR_SYS_CTRL				0x31
#define	I2C_ADDR_SYS_STAT				0x32
#define	I2C_ADDR_SYS_STAT2				0x33
#define	I2C_ADDR_TCH_OUTPUT				0x34
#define	I2C_ADDR_CH1_PER_H				0x35
#define	I2C_ADDR_CH1_PER_L				0x36
#define	I2C_ADDR_CH2_PER_H				0x37
#define	I2C_ADDR_CH2_PER_L				0x38
#define	I2C_ADDR_CH3_PER_H				0x39
#define	I2C_ADDR_CH3_PER_L				0x3A
#define	I2C_ADDR_CR_DUTY_H				0x3B
#define	I2C_ADDR_CR_DUTY_L				0x3C
#define	I2C_ADDR_CS1_DUTY_H				0x3D
#define	I2C_ADDR_CS1_DUTY_L				0x3E
#define	I2C_ADDR_CS2_DUTY_H				0x3F
#define	I2C_ADDR_CS2_DUTY_L				0x40
#define	I2C_ADDR_CS3_DUTY_H				0x41
#define	I2C_ADDR_CS3_DUTY_L				0x42

#define I2C_ADDR_PGM_VER_MAIN			0x71
#define I2C_ADDR_PGM_VER_SUB			0x72

//Calibration Data Backup/Restore
#define I2C_ADDR_CMD_OPT 					0x7E
#define I2C_ADDR_COMMAND 					0x7F
#define I2C_ADDR_REQ_DATA					0x80
#define CMD_R_CD_DUTY						0x04		//Cal Data Duty Read
#define CMD_R_CD_REF						0x05		//Cal Data Ref Read
#define CMD_W_CD_DUTY						0x84		//Cal Data Duty Read
#define CMD_W_CD_REF						0x85		//Cal Data Ref Read
#endif
