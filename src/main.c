 /*!
    \file    main.c
    \brief   dual CAN communication in normal mode

*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "gd32c10x.h"

#include "i2c.h"
#include "can.h"
#include "systick.h"
#include "dual_dfs.h"


int flgCAN0Get = 0;
int flgCAN1Get = 0;

int CAN0_NUM_BUFF_MSGS = 0;
int CAN1_NUM_BUFF_MSGS = 0;

int can0_buffer_index = 0;
int can1_buffer_index = 0;

uint8_t CAN0_DATA_BUFFER[MAX_CAN_RECV][100];
uint8_t CAN1_DATA_BUFFER[MAX_CAN_RECV][100];

uint8_t can0config[CANCONFIG_SIZE];
uint8_t can1config[CANCONFIG_SIZE];

can_receive_message_struct g_receive_message0;
can_receive_message_struct g_receive_message1;
can_trasnmit_message_struct g_transmit_message;

void
long2char(uint32_t __t, uint8_t *str)
{
	str[0] = (__t >> 24) & 0xff;
	str[1] = (__t >> 16) & 0xff;
	str[2] = (__t >> 8) & 0xff;
	str[3] = (__t >> 0) & 0xff;
}

void
savecandata(uint32_t canp)
{
	int *canbc = canp == CAN0 ? &CAN0_NUM_BUFF_MSGS : &CAN1_NUM_BUFF_MSGS;
	int *canbi = canp == CAN0 ? &can0_buffer_index : &can1_buffer_index;
	uint32_t id = 0;
	can_receive_message_struct *canrm;
	uint8_t *candb;

	*canbi = *canbi + 1 > (MAX_CAN_RECV - 1) ? 0 : *canbi + 1;
	candb = canp == CAN0 ? &CAN0_DATA_BUFFER[*canbi][0] : &CAN1_DATA_BUFFER[*canbi][0];

	canrm = canp == CAN0 ? &g_receive_message0 : &g_receive_message1;
	if (canrm->rx_ff == CAN_FF_EXTENDED) {
		id = canrm->rx_efid;
		candb[4] = 1;
	} else {
		id = canrm->rx_sfid;
		candb[4] = 0;
	}
	long2char(id, candb);

	candb[5] = (canrm->rx_ft == CAN_FT_REMOTE) ? 1 : 0;
	candb[7] = canrm->rx_dlen;

	for (int i = 0; i < candb[7]; i++) {
		candb[8 + i] = canrm->rx_data[i];
	}

	*canbc = *canbc + 1 > MAX_CAN_RECV ? MAX_CAN_RECV : *canbc + 1;
}

int
geti2cDta(uint8_t *dta)
{
	int len = 0;

	if (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND)) {
		return 0;
	}
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
printf("geti2cDta flag_addsend\r\n");

	/* wait for data */
	while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE))
		continue;
	while (i2c_flag_get(I2C0, I2C_FLAG_RBNE)) {
		dta[len++] = i2c_data_receive(I2C0);
		/* XXX is this really necessary? */
		for (int i = 0; i < 3000; i++) {
			__NOP();
		}

		if (len > 73) { /* XXX ??? some limitation from arduino? */
			/*i2c_flag_clear(I2C0, I2C_FLAG_RBNE);*/
			return 0;
		}
	}

	/* wait for stop */
	while (!i2c_flag_get(I2C0, I2C_FLAG_STPDET))
		continue;

	i2c_enable(I2C0); /* XXX necessary ? */
	return len;
}

static inline int
wait_i2c_flag(i2c_flag_enum flag, int clear_it)
{
	uint32_t tout = 0;

	while (!i2c_flag_get(I2C0, flag)) {
		__NOP();
		tout++;
		if (tout > 5000) {
			i2c_flag_clear(I2C0, flag);
			return 0;
		}
	}
	if (clear_it)
		i2c_flag_clear(I2C0, flag);

	return 1;
}

int
sendi2cDta(uint8_t *dta, int dlen)
{
	/* wait until ADDSEND bit is set, and clear it */
	if (!wait_i2c_flag(I2C_FLAG_ADDSEND, 1))
		return 0;

	/* wait until the transmission data register is empty */
	if (!wait_i2c_flag(I2C_FLAG_TBE, 0))
		return 0;

	for (int i = 0; i < dlen; i++) {
		/* send a data byte */
		i2c_data_transmit(I2C0, dta[i]);
		/* wait until the transmission data register is empty */
		if (!wait_i2c_flag(I2C_FLAG_TBE, 0))
			return 0;
	}

	/* the master doesn't acknowledge for the last byte, clear AERR */
	if (!wait_i2c_flag(I2C_FLAG_AERR, 1))
		return 0;

	return dlen;
}

void
CANX_Send_From_I2C(uint32_t can_periph, uint8_t *str)
{
	uint32_t id = 0;

	id |= str[1];
	id <<= 8;
	id |= str[2];
	id <<= 8;
	id |= str[3];
	id <<= 8;
	id |= str[4];

	/* initialize transmit message */
	g_transmit_message.tx_sfid = id;
	g_transmit_message.tx_efid = id;
	g_transmit_message.tx_ft = str[5] ? CAN_FT_REMOTE : CAN_FT_DATA;
	g_transmit_message.tx_ff = str[6] ? CAN_FF_EXTENDED : CAN_FF_STANDARD;
	g_transmit_message.tx_dlen = str[8];
	for (int i = 0; i < str[8]; i++) {
		g_transmit_message.tx_data[i] = str[9 + i];
	}

	can_message_transmit(can_periph, &g_transmit_message);
}

/* XXX just working out the indentation to make main() more readable for now. */
void i2c_loop(uint8_t *, uint8_t *);

void
setup_serial(void)
{
	nvic_irq_enable(USART0_IRQn, 2, 2);

	/* enable GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOA);

	/* enable USART clock */
	rcu_periph_clock_enable(RCU_USART0);

	/* connect port to USARTx_Tx */
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
	/* connect port to USARTx_Rx */
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

	/* USART configure */
	usart_deinit(USART0);
	usart_baudrate_set(USART0, 115200U);
	usart_receive_config(USART0, USART_RECEIVE_ENABLE);
	usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
	usart_enable(USART0);

	usart_interrupt_enable(USART0, USART_INT_RBNE);
}

static uint8_t dbgpingpong = 0;

int
main(void)
{
	uint32_t i2c_spd = 400000U;
	uint32_t i2c_duc = I2C_DTCY_16_9;
	/* configure board */
	systick_config();

	/* activate GPIO clocks */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);

	setup_serial();
delay_1ms(5000);
	/* I2C configure */
	i2c_gpio_config();

	static uint32_t cfgs[5][2] = {
		{ 1000000U, I2C_DTCY_16_9 },
		{ 1000000U, I2C_DTCY_2 },
		{ 400000U, I2C_DTCY_16_9 },
		{ 400000U, I2C_DTCY_2 },
		{ 100000U, I2C_DTCY_2 }
	};
	uint32_t dbgcnt = 0;
	int cfgindex = 0;
i2cdbgloop:
	i2c_spd = cfgs[cfgindex][0];
	i2c_duc = cfgs[cfgindex][1];
	i2c_config(0x41, i2c_spd, i2c_duc);
	printf("gd32e103 i2c cfg %u %s\n", i2c_spd, i2c_duc ? "16/9" : "/2");
	for (int i = 0; i < 10; i++) {
		if (wait_i2c_flag(I2C_FLAG_ADDSEND, 0))
			break;
		delay_1ms(5);
	}
	if (wait_i2c_flag(I2C_FLAG_ADDSEND, 0) == 0) {
		cfgindex++;
		if (cfgindex < 5)
			goto i2cdbgloop;
		printf("gd32e103 i2c failure...\r\n")
	} else
		printf("gd32e103 got addressed on i2c bus\r\n");

	/* CAN configure */
	can_gpio_config();

	/* initialize receive message */
	can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &g_receive_message0);
	can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &g_receive_message1);

	uint8_t i2cDtaFromRP2040[CANCONFIG_SIZE];
	uint8_t dtaSendToRP2040[100] = {0};

	while(1) {
		i2c_loop(&i2cDtaFromRP2040[0], &dtaSendToRP2040[0]);

		/* ISR set FIFO not empty flag for CAN0. Buffer the message data. */
		if (flgCAN0Get) {
			flgCAN0Get = 0;
			savecandata(CAN0);
		}
		if (flgCAN1Get) {
			flgCAN1Get = 0;
			savecandata(CAN1);
		}
	}
}

void
canrecv_info(uint32_t cp, uint8_t *dtaSendToRP2040)
{
	int cbc = cp == CAN0 ? CAN0_NUM_BUFF_MSGS : CAN1_NUM_BUFF_MSGS;
	int cbi = cp == CAN0 ? can0_buffer_index : can1_buffer_index;
	uint8_t *cdb = cp == CAN0 ? &CAN0_DATA_BUFFER[cbi][0] : &CAN1_DATA_BUFFER[cbi][0];

	if (cbc > 0) {
		for (int i = 0; i < 8; i++) {
			dtaSendToRP2040[i] = cdb[i];
		}
		sendi2cDta(dtaSendToRP2040, 8);
	}
}

void
canaddr_recv1(uint32_t cp, uint8_t *dtaSendToRP2040)
{
	int *canbc = cp == CAN0 ? &CAN0_NUM_BUFF_MSGS : &CAN1_NUM_BUFF_MSGS;
	int cbc = cp == CAN0 ? CAN0_NUM_BUFF_MSGS : CAN1_NUM_BUFF_MSGS;
	int *canbi = cp == CAN0 ? &can0_buffer_index : &can1_buffer_index;
	int cbi = cp == CAN0 ? can0_buffer_index : can1_buffer_index;
	uint8_t *cdb = cp == CAN0 ? &CAN0_DATA_BUFFER[cbi][0] : &CAN1_DATA_BUFFER[cbi][0];

	if (cbc > 0) {
		if (cdb[7] <= 32) {
			for (int i = 0; i < cdb[7]; i++) {
				dtaSendToRP2040[i] = cdb[8 + i];
			}
			sendi2cDta(dtaSendToRP2040, cdb[7]);

			cbi--;
			*canbi = cbi < 0 ? MAX_CAN_RECV - 1 : cbi;
			*canbc = cbc - 1;
		} else {
			for (int i = 0; i < 32; i++) {
				dtaSendToRP2040[i] = cdb[8 + i];
			}
			sendi2cDta(dtaSendToRP2040, 32);
		}
	}
}

void
canaddr_recv2(uint32_t cp, uint8_t *dtaSendToRP2040)
{
	int *canbc = cp == CAN0 ? &CAN0_NUM_BUFF_MSGS : &CAN1_NUM_BUFF_MSGS;
	int cbc = cp == CAN0 ? CAN0_NUM_BUFF_MSGS : CAN1_NUM_BUFF_MSGS;
	int *canbi = cp == CAN0 ? &can0_buffer_index : &can1_buffer_index;
	int cbi = cp == CAN0 ? can0_buffer_index : can1_buffer_index;
	uint8_t *cdb = cp == CAN0 ? &CAN0_DATA_BUFFER[cbi][0] : &CAN1_DATA_BUFFER[cbi][0];

	if (cbc && (cdb[7] > 32)) {
		for (int i = 0; i < (cdb[7] - 32); i++) {
			dtaSendToRP2040[i] = cdb[40 + i];
		}
		sendi2cDta(dtaSendToRP2040, cdb[7] - 32);
		cbi--;
		*canbi = cbi < 0 ? MAX_CAN_RECV - 1 : cbi;
		*canbc = cbc - 1;
	}
}
/* XXX should deduplicate some code from cases below */
void
i2c_loop(uint8_t *i2cDtaFromRP2040, uint8_t *dtaSendToRP2040)
{
	int len = geti2cDta(i2cDtaFromRP2040);

	if (len < 1)
		return;
	printf("i2c_loop %d %x\n", len, i2cDtaFromRP2040[0]);
	switch (i2cDtaFromRP2040[0]) {
/* CAN data functions */
	case CAN0_SEND_MSG:
		CANX_Send_From_I2C(CAN0, i2cDtaFromRP2040);
		break;
	case CAN1_SEND_MSG:
		CANX_Send_From_I2C(CAN1, i2cDtaFromRP2040);
		break;
	case CAN0_MSG_RECV_NUM:
		dtaSendToRP2040[0] = CAN0_NUM_BUFF_MSGS;
		sendi2cDta(dtaSendToRP2040, 1);
		break;
	case CAN1_MSG_RECV_NUM:
		dtaSendToRP2040[0] = CAN1_NUM_BUFF_MSGS;
		sendi2cDta(dtaSendToRP2040, 1);
		break;
	case CAN0_RECV_INFO:
		canrecv_info(CAN0, dtaSendToRP2040);
		break;
	case REG_ADDR_RECV1:
		canaddr_recv1(CAN0, dtaSendToRP2040);
		break;
	case REG_ADDR_RECV2:
		canaddr_recv2(CAN0, dtaSendToRP2040);
		break;
	case CAN1_RECV_INFO:
		canrecv_info(CAN1, dtaSendToRP2040);
		break;
	case REG1_ADDR_RECV1:
		canaddr_recv1(CAN1, dtaSendToRP2040);
		break;
	case REG1_ADDR_RECV2:
		canaddr_recv2(CAN0, dtaSendToRP2040);
		break;
/* CAN control functions */
	case CAN0_CONFIG:
		memcpy(can0config, &i2cDtaFromRP2040[1], CANCONFIG_SIZE);
		can_param_config(CAN0, can0config);
		/* If config was called more than once, buffers should be emptied. */
		can0_buffer_index = CAN0_NUM_BUFF_MSGS = 0;
		break;
	case CAN1_CONFIG:
		memcpy(can1config, &i2cDtaFromRP2040[1], CANCONFIG_SIZE);
		can_param_config(CAN1, can1config);
		can1_buffer_index = CAN1_NUM_BUFF_MSGS = 0;
		break;
	case CAN0_SLEEP:
		can_working_mode_set(CAN0, CAN_MODE_SLEEP);
		break;
	case CAN1_SLEEP:
		can_working_mode_set(CAN1, CAN_MODE_SLEEP);
		break;
	case CAN0_WAKE:
		can_working_mode_set(CAN0, CAN_MODE_SLEEP);
		break;
	case CAN1_WAKE:
		can_working_mode_set(CAN1, CAN_MODE_SLEEP);
		break;
	case DBG_PINGPONG:
		dbgpingpong = i2cDtaFromRP2040[1];
		if (dbgpingpong > 0)
			dbgpingpong++;
		if (sendi2cDta(&dbgpingpong, 1) == 1)
			break;
		break;
	default:
		break;
	}
}

#if /* XXX */1
/* retarget the C library printf function to the usart */
int fputc(int ch, FILE *f)
{
	usart_data_transmit(USART0, (uint8_t)ch);
	while (RESET == usart_flag_get(USART0, USART_FLAG_TBE));
	return ch;
}
#endif

#if 0 /* currently irrelevant, but necessary for --specs=nano.specs */
void
_exit(int st)
{
	while (1);
}
#endif