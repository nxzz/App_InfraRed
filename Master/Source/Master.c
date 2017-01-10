/****************************************************************************
 * (C) Tokyo Cosmos Electric, Inc. (TOCOS) - all rights reserved.
 *
 * Condition to use: (refer to detailed conditions in Japanese)
 *   - The full or part of source code is limited to use for TWE (TOCOS
 *     Wireless Engine) as compiled and flash programmed.
 *   - The full or part of source code is prohibited to distribute without
 *     permission from TOCOS.
 *
 * 利用条件:
 *   - 本ソースコードは、別途ソースコードライセンス記述が無い限り東京コスモス電機が著作権を
 *     保有しています。
 *   - 本ソースコードは、無保証・無サポートです。本ソースコードや生成物を用いたいかなる損害
 *     についても東京コスモス電機は保証致しません。不具合等の報告は歓迎いたします。
 *   - 本ソースコードは、東京コスモス電機が販売する TWE シリーズ上で実行する前提で公開
 *     しています。他のマイコン等への移植・流用は一部であっても出来ません。
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "Master.h"

#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "../../Master/Source/Version.h"
#include "utils.h"

#include "config.h"
#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
// Select Modules (define befor include "ToCoNet.h")
//#define ToCoNet_USE_MOD_NBSCAN // Neighbour scan module
//#define ToCoNet_USE_MOD_NBSCAN_SLAVE

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef struct
{
    // Transmit Power
    uint8 u8Power;

    // MAC retry
    uint8 u8MacRetry;

    // MAC
    uint8 u8channel;
    uint16 u16addr;

    // LED Counter
    uint32 u32LedCt;

    // シーケンス番号
    uint32 u32Seq;

    // スリープカウンタ
    uint8 u8SleepCt;

    // TX 中
    bool_t bOnTx;

    // メインアプリケーション処理部
	void *prPrsEv; //!< vProcessEvCoreSlpまたはvProcessEvCorePwrなどの処理部へのポインタ
} tsAppData;


/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static void vInitHardware(int f_warm_start);

void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt);
static void vHandleSerialInput(void);

static void vTransmit();

static void vSetIrReading(bool_t bStart);
static uint16 getWidth(uint16 primary, uint16 secondary);
static void vDecodeIrCmd();
static void vDumpIrCmds();
static void vDumpEdgeTiming(uint16 count);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* Version/build information. This is not used in the application unless we
   are in serial debug mode. However the 'used' attribute ensures it is
   present in all binary files, allowing easy identifaction... */

/* Local data used by the tag during operation */
static tsAppData sAppData;

PUBLIC tsFILE sSerStream;
tsSerialPortSetup sSerPort;

// Wakeup port
const uint32 u32DioPortWakeUp = 1UL << 7; // UART Rx Port

#define IR_TIMER_PRESCALE (8)
#define T_us (1<<(IR_TIMER_PRESCALE-4))
#define LEADER_TIMEOUT_CLK (9400/T_us)
// NEC_LEADER_us (9000)
#define NEC_0_1_CLK (1688/T_us)
#define NEC_REPEAT_CLK (3375/T_us)
#define NEC_TIMEOUT_CLK (4900/T_us)
// KADEN_LEADER_us (3200)
#define KADEN_LEADER_MAX_CNT (3600/T_us)
#define KADEN_0_1_CLK (1200/T_us)
#define KADEN_TIMEOUT_CLK (2000/T_us)
// SONY_LEADER_us (2400)
#define SONY_LEADER_MIN_CNT (2000/T_us)
#define SONY_LEADER_MAX_CNT (2800/T_us)
#define SONY_0_1_CLK (1500/T_us)
#define SONY_TIMEOUT_CLK (2200/T_us)
#define TIMER_PERIOD (0x4000)

#define MAX_PULSE_COUNT 1024
static uint16 u16RisingEdge[MAX_PULSE_COUNT];
static volatile uint16 u16RisingCount;
static uint16 u16FallingEdge[MAX_PULSE_COUNT];
static uint16 u16FallingCount;
typedef enum {
	E_IR_NEC = 1,
	E_IR_NEC_REPEAT,
	E_IR_KADEN,
	E_IR_KADEN_REPEAT,
	E_IR_SONY
} teIrType;
#define MAX_IR_CMDS 64
typedef struct {
	uint8 irtype;	// 1:NEC, 2:KADEN, 3:SONY
	uint8 bits;
	uint8 codes[10];
} tsIrCmd;
static tsIrCmd sIrCmds[MAX_IR_CMDS];
static tsIrCmd *pIrCmd = sIrCmds;
static uint16 u16IrCmdCount;
static uint16 u16TimeoutCnt = NEC_TIMEOUT_CLK;
static bool_t bReading = FALSE;

/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbAppColdStart(bool_t bAfterAhiInit)
{
	//static uint8 u8WkState;
	if (!bAfterAhiInit) {
		// before AHI init, very first of code.

		// Register modules
		ToCoNet_REG_MOD_ALL();

	} else {
		// disable brown out detect
		vAHI_BrownOutConfigure(0,//0:2.0V 1:2.3V
				FALSE,
				FALSE,
				FALSE,
				FALSE);

		// clear application context
		memset (&sAppData, 0x00, sizeof(sAppData));
		sAppData.u8channel = 18;
		sAppData.u8Power = 3; // MAX
		sAppData.u8MacRetry = 3; // DEFAULT

		// ToCoNet configuration
		sToCoNet_AppContext.u32AppId = APP_ID;
		sToCoNet_AppContext.u8Channel = CHANNEL;
		sToCoNet_AppContext.u8CPUClk = 3;

		sToCoNet_AppContext.bRxOnIdle = FALSE; // 受信しない
		sToCoNet_AppContext.u16TickHz = 1000; // システムTICKを高速化

		sToCoNet_AppContext.u8CCA_Level = 1; // CCA 設定の最小化(要電流グラフチェック)
		sToCoNet_AppContext.u8CCA_Retry = 0;

		// others
		SPRINTF_vInit128();

		// Register
		ToCoNet_Event_Register_State_Machine(vProcessEvCore);
		sAppData.prPrsEv = (void*) vProcessEvCore;

		// Others
		vInitHardware(FALSE);

		// MAC start
		ToCoNet_vMacStart();
	}
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static bool_t bWakeupByButton;

void cbAppWarmStart(bool_t bAfterAhiInit)
{
	if (!bAfterAhiInit) {
		// before AHI init, very first of code.
		//  to check interrupt source, etc.
		bWakeupByButton = FALSE;

		if(u8AHI_WakeTimerFiredStatus()) {
			// wake up timer
		} else
		if(u32AHI_DioWakeStatus() & u32DioPortWakeUp) {
			// woke up from DIO events
			bWakeupByButton = TRUE;
		} else {
			bWakeupByButton = FALSE;
		}
	} else {
		// Initialize hardware
		vInitHardware(TRUE);

		// MAC start
		ToCoNet_vMacStart();
	}
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: vMain
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbToCoNet_vMain(void)
{
	/* handle uart input */
	vHandleSerialInput();
}

/****************************************************************************
 *
 * NAME: cbToCoNet_vNwkEvent
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
	switch(eEvent) {
	default:
		break;
	}
}

/****************************************************************************
 *
 * NAME: cbvMcRxHandler
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbToCoNet_vRxEvent(tsRxDataApp *pRx) {
#if 0
	int i;
	// print coming payload
	vfPrintf(&sSerStream, LB"[PKT Ad:%04x,Ln:%03d,Seq:%03d,Lq:%03d,Tms:%05d \"",
			pRx->u32SrcAddr,
			pRx->u8Len+4, // Actual payload byte: the network layer uses additional 4 bytes.
			pRx->u8Seq,
			pRx->u8Lqi,
			pRx->u32Tick & 0xFFFF);
	for (i = 0; i < pRx->u8Len; i++) {
		if (i < 32) {
			sSerStream.bPutChar(sSerStream.u8Device,
					(pRx->auData[i] >= 0x20 && pRx->auData[i] <= 0x7f) ? pRx->auData[i] : '.');
		} else {
			vfPrintf(&sSerStream, "..");
			break;
		}
	}
#endif

	pRx->auData[pRx->u8Len] = 0;
	vfPrintf(&sSerStream, LB"RX %05d: %s", pRx->u32Tick & 0xFFFF, pRx->auData);
}

/****************************************************************************
 *
 * NAME: cbvMcEvTxHandler
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	if (sAppData.bOnTx) {
		vTransmit();
	}
	return;
}

/****************************************************************************
 *
 * NAME: cbToCoNet_vHwEvent
 *
 * DESCRIPTION:
 * Process any hardware events.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId
 *                  u32ItemBitmap
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	switch (u32DeviceId) {
    case E_AHI_DEVICE_TICK_TIMER:
		// LED BLINK
   		vPortSet_TrueAsLo(PORT_OUT3, u32TickCount_ms & 0x400);

   		// LED ON when receive
   		if (u32TickCount_ms - sAppData.u32LedCt < 300) {
   			vPortSetLo(PORT_OUT1);
   		} else {
  			vPortSetHi(PORT_OUT1);
   		}
    	break;

    case E_AHI_DEVICE_TIMER3:
    	_C {
			vDecodeIrCmd();
    	}
    	break;

    case E_AHI_DEVICE_SYSCTRL:
		_C {
			if (u32ItemBitmap & PORT_IR_RISING_MASK) {
				if (u16RisingCount == 1) {
					// determine IR type by leader pulse length
					pIrCmd = &sIrCmds[u16IrCmdCount];
					uint16 leaderLength = getWidth(u16FallingEdge[0], u16RisingEdge[0]);
					if (leaderLength < SONY_LEADER_MAX_CNT) {
						pIrCmd->irtype = E_IR_SONY;
						u16TimeoutCnt = SONY_TIMEOUT_CLK;
					} else if (leaderLength < KADEN_LEADER_MAX_CNT) {
						pIrCmd->irtype = E_IR_KADEN;
						u16TimeoutCnt = KADEN_TIMEOUT_CLK;
					} else {
						pIrCmd->irtype = E_IR_NEC;
						u16TimeoutCnt = NEC_TIMEOUT_CLK;
					}
				}
				vAHI_TimerStartSingleShot(E_AHI_TIMER_3, 0, u16TimeoutCnt);
			}
		}
		break;

    default:
    	break;
    }
}

/****************************************************************************
 *
 * NAME: cbToCoNet_u8HwInt
 *
 * DESCRIPTION:
 *   called during an interrupt
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId
 *                  u32ItemBitmap
 *
 * RETURNS:
 *                  FALSE -  interrupt is not handled, escalated to further
 *                           event call (cbToCoNet_vHwEvent).
 *                  TRUE  -  interrupt is handled, no further call.
 *
 * NOTES:
 *   Do not put a big job here.
 ****************************************************************************/
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
    switch (u32DeviceId) {

    case E_AHI_DEVICE_SYSCTRL:
		_C {
			uint16 t = u16AHI_TimerReadCount(E_AHI_TIMER_2);
			if (u32ItemBitmap & PORT_IR_FALLING_MASK) {
				if (u16FallingCount < MAX_PULSE_COUNT) {
					u16FallingEdge[u16FallingCount++] = t;
				}
			}
			else if (u32ItemBitmap & PORT_IR_RISING_MASK) {
				if (u16RisingCount < MAX_PULSE_COUNT) {
					u16RisingEdge[u16RisingCount++] = t;
				}
			}
		}
		break;
    default:
    	break;
    }
	return FALSE;
}

/****************************************************************************
 *
 * NAME: vInitHardware
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static void vInitHardware(int f_warm_start)
{
	// Serial Initialize
	vSerialInit(UART_BAUD, NULL);

	ToCoNet_vDebugInit(&sSerStream);
	ToCoNet_vDebugLevel(0);

	/// IOs
	vPortAsOutput(PORT_OUT1);
	vPortAsOutput(PORT_OUT2);
	vPortAsOutput(PORT_OUT3);
	vPortAsOutput(PORT_OUT4);
	vPortSetHi(PORT_OUT1);
	vPortSetHi(PORT_OUT2);
	vPortSetHi(PORT_OUT3);
	vPortDisablePullup(PORT_OUT1);
	vPortDisablePullup(PORT_OUT2);
	vPortDisablePullup(PORT_OUT3);
	vPortDisablePullup(PORT_OUT4);
	vPortAsInput(PORT_INPUT2);

	// Timer
	vAHI_TimerEnable(E_AHI_TIMER_2, IR_TIMER_PRESCALE, FALSE, TRUE, FALSE);
	vAHI_TimerEnable(E_AHI_TIMER_3, IR_TIMER_PRESCALE, FALSE, TRUE, FALSE);
}

/****************************************************************************
 *
 * NAME: vInitHardware
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[96];
	static uint8 au8SerialRxBuffer[32];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = u32Baud;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT_SLAVE;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInitEx(&sSerPort, pUartOpt);

	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT_SLAVE;
}

/****************************************************************************
 *
 * NAME: vHandleSerialInput
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
static void vHandleSerialInput(void)
{
    // handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char;

		i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

		vfPrintf(&sSerStream, "\n\r# [%c] --> ", i16Char);
	    SERIAL_vFlush(sSerStream.u8Device);

		switch(i16Char) {

		case '>': case '.':
			/* channel up */
			sAppData.u8channel++;
			if (sAppData.u8channel > 26) sAppData.u8channel = 11;
			sToCoNet_AppContext.u8Channel = sAppData.u8channel;
			ToCoNet_vRfConfig();
			vfPrintf(&sSerStream, "set channel to %d.", sAppData.u8channel);
			break;

		case '<': case ',':
			/* channel down */
			sAppData.u8channel--;
			if (sAppData.u8channel < 11) sAppData.u8channel = 26;
			sToCoNet_AppContext.u8Channel = sAppData.u8channel;
			ToCoNet_vRfConfig();
			vfPrintf(&sSerStream, "set channel to %d.", sAppData.u8channel);
			break;

		case 'd': case 'D':
			_C {
				static uint8 u8DgbLvl;

				u8DgbLvl++;
				if(u8DgbLvl > 5) u8DgbLvl = 0;
				ToCoNet_vDebugLevel(u8DgbLvl);

				vfPrintf(&sSerStream, "set NwkCode debug level to %d.", u8DgbLvl);
			}
			break;

		case 's': case 'S':
			// スリープのテストコード
			_C {
				// print message.
				sAppData.u8SleepCt++;

				// stop interrupt source, if interrupt source is still running.
				;

				vfPrintf(&sSerStream, "now sleeping" LB);
				SERIAL_vFlush(sSerStream.u8Device); // flushing

				if (i16Char == 's') {
					vAHI_UartDisable(sSerStream.u8Device);
				}

				// set UART Rx port as interrupt source
				vAHI_DioSetDirection(u32DioPortWakeUp, 0); // set as input

				(void)u32AHI_DioInterruptStatus(); // clear interrupt register
				vAHI_DioWakeEnable(u32DioPortWakeUp, 0); // also use as DIO WAKE SOURCE
				// vAHI_DioWakeEdge(0, PORT_INPUT_MASK); // 割り込みエッジ（立下りに設定）
				vAHI_DioWakeEdge(u32DioPortWakeUp, 0); // 割り込みエッジ（立上がりに設定）
				// vAHI_DioWakeEnable(0, PORT_INPUT_MASK); // DISABLE DIO WAKE SOURCE

				// wake up using wakeup timer as well.
				ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, 0, FALSE, TRUE); // PERIODIC RAM OFF SLEEP USING WK0
			}
			break;

		case 'p':
			// 出力調整のテスト
			_C {
				static uint8 u8pow = 3; // (MIN)0..3(MAX)

				u8pow = (u8pow + 1) % 4;
				vfPrintf(&sSerStream, "set power to %d.", u8pow);

				sToCoNet_AppContext.u8TxPower = u8pow;
				ToCoNet_vRfConfig();
			}
			break;

		case 'r':
			_C {
				ToCoNet_Event_Process(E_EVENT_IR_DUMP, 0, sAppData.prPrsEv);
			}
			break;

		case 't': // パケット送信してみる
			_C {
				if (!sAppData.bOnTx) {
					vTransmit();
					sAppData.bOnTx = TRUE;
				} else {
					sAppData.bOnTx = FALSE;
				}
			}
			break;

		default:
			break;
		}

		vfPrintf(&sSerStream, LB);
	    SERIAL_vFlush(sSerStream.u8Device);
	}
}

/****************************************************************************
 *
 * NAME: vProcessEvent
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			// ここで UART のメッセージを出力すれば安全である。
			if (u32evarg & EVARG_START_UP_WAKEUP_RAMHOLD_MASK) {
				vfPrintf(&sSerStream, LB "RAMHOLD");
			}
			if (u32evarg & EVARG_START_UP_WAKEUP_MASK) {
				vfPrintf(&sSerStream, LB "Wake up by %s. SleepCt=%d",
						bWakeupByButton ? "UART PORT" : "WAKE TIMER",
						sAppData.u8SleepCt);
			} else {
				vfPrintf(&sSerStream, "\r\n*** ToCoNet App_InfraRed %d.%02d-%d ***", VERSION_MAIN, VERSION_SUB, VERSION_VAR);
				vfPrintf(&sSerStream, "\r\n*** %08x ***", ToCoNet_u32GetSerial());
			}
			ToCoNet_Event_SetState(pEv, E_STATE_IR_READING);
		}
		break;

	case E_STATE_RUNNING:
		if (eEvent == E_EVENT_IR_START) {
			vPortSetHi(PORT_OUT2);
			memset (&sIrCmds, 0x00, sizeof(sIrCmds));
			u16RisingCount = 0;
			u16FallingCount = 0;
			vAHI_TimerStartRepeat(E_AHI_TIMER_2, 0, TIMER_PERIOD);
			ToCoNet_Event_SetState(pEv, E_STATE_IR_READING);
		}
		break;

	case E_STATE_IR_READING:
		if (eEvent == E_EVENT_NEW_STATE) {
			vPortSetHi(PORT_OUT2);
			memset (&sIrCmds, 0x00, sizeof(sIrCmds));
			u16RisingCount = 0;
			u16FallingCount = 0;
			vAHI_TimerStartRepeat(E_AHI_TIMER_2, 0, TIMER_PERIOD);
			vfPrintf(&sSerStream, "* Start IR reading.\r\n");
			vSetIrReading(TRUE);
		} else if (eEvent == E_EVENT_IR_DUMP) {
			ToCoNet_Event_SetState(pEv, E_STATE_IR_FINISHED);
		} else if (eEvent == E_EVENT_IR_TIMEOUT) {
			vDumpIrCmds();
		}
		break;

	case E_STATE_IR_FINISHED:
		if (eEvent == E_EVENT_NEW_STATE) {
			vSetIrReading(FALSE);
			vPortSetHi(PORT_OUT2);
			vfPrintf(&sSerStream, "* Stop IR reading.\r\n");
			vDecodeIrCmd();
			vDumpIrCmds();
			ToCoNet_Event_SetState(pEv, E_STATE_IR_READING);
		}
		break;

	default:
		break;
	}
}


/**
 * 送信関数(ブロードキャスト)
 */
static void vTransmit() {
	_C {
		// transmit Ack back
		tsTxDataApp tsTx;
		memset(&tsTx, 0, sizeof(tsTxDataApp));

		sAppData.u32Seq++;

		tsTx.u32SrcAddr = ToCoNet_u32GetSerial(); // 自身のアドレス
		tsTx.u32DstAddr = 0xFFFF; // ブロードキャスト

		tsTx.bAckReq = FALSE;

		tsTx.u8Retry = 0; // 再送なし
		tsTx.u16DelayMin = 0;
		tsTx.u16DelayMax = 0;
		tsTx.u16RetryDur = 0; // 遅延は0で速やかに送信

		tsTx.u8CbId = sAppData.u32Seq & 0xFF;
		tsTx.u8Seq = sAppData.u32Seq & 0xFF;
		tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;

		// SPRINTF でメッセージを作成
		SPRINTF_vRewind();
		vfPrintf(SPRINTF_Stream, "MSG: %08X:%04X", ToCoNet_u32GetSerial(), sAppData.u32Seq & 0xFFFF);
		memcpy(tsTx.auData, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
		tsTx.u8Len = SPRINTF_u16Length();

		// 送信
		if (ToCoNet_bMacTxReq(&tsTx)) {
			ToCoNet_Tx_vProcessQueue(); // 速やかに送信処理する(実験的な機能)
			vfPrintf(&sSerStream, ".");
		} else {
			vfPrintf(&sSerStream, "x");
		}

	}
}

/**
 * Start/Stop Infra-Red signal reading.
 * @param start
 */
static void vSetIrReading(bool_t bStart) {
	if (bStart) {
		// Set interrupt
		vAHI_DioInterruptEdge(PORT_IR_RISING_MASK, PORT_IR_FALLING_MASK);
		vAHI_DioInterruptEnable(PORT_IR_RECEIVE_MASK, 0);
	} else {
		vAHI_DioInterruptEnable(0, 0);
		vAHI_TimerStop(E_AHI_TIMER_2);
		vAHI_TimerStop(E_AHI_TIMER_3);
	}
}

static uint16 getWidth(uint16 primary, uint16 secondary) {
	if (secondary < primary) {
		secondary += TIMER_PERIOD;
	}
	return (secondary - primary);
}

static void vDecodeIrCmd() {
	if (u16RisingCount < 2 || (u16RisingCount != u16FallingCount)) {
		u16FallingCount = u16RisingCount = 0;
		return;
	}
	tsIrCmd *pCmd = &sIrCmds[u16IrCmdCount];
	memset (&pCmd->codes, 0x00, sizeof(pCmd->codes));
	uint16 u16threshold = NEC_0_1_CLK;
	switch (pCmd->irtype) {
	case E_IR_NEC:
		u16threshold = NEC_0_1_CLK;
		break;
	case E_IR_KADEN:
		u16threshold = KADEN_0_1_CLK;
		break;
	case E_IR_SONY:
		u16threshold = SONY_0_1_CLK;
		break;
	default:
		break;
	}
	uint8 i;
	for (i = 1; i < u16RisingCount; i++) {
		uint16 edgeSpan = getWidth(u16RisingEdge[i - 1], u16RisingEdge[i]);
		if (edgeSpan > u16threshold) {
			pCmd->codes[i/8] |= (0x80>>(i&0x7));
		}
	}
	pCmd->bits = u16RisingCount - 1;
	u16IrCmdCount++;
	u16FallingCount = u16RisingCount = 0;
	vfPrintf(&sSerStream, " %d", u16IrCmdCount);
}

static void vDumpIrCmds() {
	uint16 i;
	if (u16IrCmdCount > 0) {
		tsIrCmd *pCmd = sIrCmds;
		vfPrintf(&sSerStream, LB"cnt=%d", u16IrCmdCount);
		for (i = 0; i < u16IrCmdCount; i++) {
			pCmd = &sIrCmds[i];
			vfPrintf(&sSerStream, LB"type:%d, bits:%d,", pCmd->irtype, pCmd->bits);
			vWait(4096);
			uint8 c;
			for (c = 0; c <= (pCmd->bits) / 8; c++) {
				vfPrintf(&sSerStream, " %02x", pCmd->codes[c]);
				vWait(4096);
			}
		}
		uint16 count = pCmd->bits + 1;
		if (pCmd->irtype != E_IR_SONY) {
			count++;
		}
		vDumpEdgeTiming(count);
		u16IrCmdCount = 0;
	}
}

static void vDumpEdgeTiming(uint16 count) {
	u16IrCmdCount = 0;
	uint16 i;
	uint16 width;
	vfPrintf(&sSerStream, LB"%d Pulse T=%dus"LB, count, T_us);
	for (i = 0; i < count; i++) {
		width = getWidth(u16FallingEdge[i], u16RisingEdge[i]);
		vfPrintf(&sSerStream, " %d(%d)", width, T_us*width);
		vWait(4096);
	}
	vfPrintf(&sSerStream, LB"Falling"LB);
	for (i = 1; i < count; i++) {
		width = getWidth(u16FallingEdge[i - 1], u16FallingEdge[i]);
		vfPrintf(&sSerStream, " %d(%d)", width, T_us*width);
		vWait(4096);
	}
	vfPrintf(&sSerStream, LB"Rising"LB);
	for (i = 1; i < count; i++) {
		width = getWidth(u16RisingEdge[i - 1], u16RisingEdge[i]);
		vfPrintf(&sSerStream, " %d(%d)", width, T_us*width);
		vWait(4096);
	}
	vfPrintf(&sSerStream, LB);
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
