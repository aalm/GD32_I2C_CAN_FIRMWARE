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

#if DEBUG /* XXX idk about this... */
#define CANFW_DBG
#endif

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
canSaveData(uint32_t can_periph)
{
	uint32_t id = 0;

	if (CAN0 == can_periph) {
		can0_buffer_index++;
		if (can0_buffer_index > (MAX_CAN_RECV - 1)) {
			can0_buffer_index = 0;
		}

		if (g_receive_message0.rx_ff == CAN_FF_EXTENDED) {
			id = g_receive_message0.rx_efid;
			CAN0_DATA_BUFFER[can0_buffer_index][4] = 1;
		} else {
			id = g_receive_message0.rx_sfid;
			CAN0_DATA_BUFFER[can0_buffer_index][4] = 0;
		}

		long2char(id, &CAN0_DATA_BUFFER[can0_buffer_index][0]);

		CAN0_DATA_BUFFER[can0_buffer_index][5] = (g_receive_message0.rx_ft == CAN_FT_REMOTE) ? 1 : 0;
		CAN0_DATA_BUFFER[can0_buffer_index][7] = g_receive_message0.rx_dlen;

		for (int i = 0; i < CAN0_DATA_BUFFER[can0_buffer_index][7]; i++) {
			CAN0_DATA_BUFFER[can0_buffer_index][8 + i] = g_receive_message0.rx_data[i];
		}

		CAN0_NUM_BUFF_MSGS++;
		if (CAN0_NUM_BUFF_MSGS > MAX_CAN_RECV) {
#if DEBUG
			printf("CAN0 message buffer is full.\n");
#endif
			CAN0_NUM_BUFF_MSGS = MAX_CAN_RECV;
		}
	} else {
		can1_buffer_index++;
		if (can1_buffer_index > (MAX_CAN_RECV - 1)) {
			can1_buffer_index = 0;
		}

		if (g_receive_message1.rx_ff == CAN_FF_EXTENDED) {
			id = g_receive_message1.rx_efid;
			CAN1_DATA_BUFFER[can1_buffer_index][4] = 1;
		} else {
			id = g_receive_message1.rx_sfid;
			CAN1_DATA_BUFFER[can1_buffer_index][4] = 0;
		}

		long2char(id, &CAN1_DATA_BUFFER[can1_buffer_index][0]);

		CAN1_DATA_BUFFER[can1_buffer_index][5] = (g_receive_message1.rx_ft == CAN_FT_REMOTE) ? 1 : 0;
		CAN1_DATA_BUFFER[can1_buffer_index][7] = g_receive_message1.rx_dlen;

		for (int i = 0; i < CAN1_DATA_BUFFER[can1_buffer_index][7]; i++) {
			CAN1_DATA_BUFFER[can1_buffer_index][8 + i] = g_receive_message1.rx_data[i];
		}

		CAN1_NUM_BUFF_MSGS++;
		if (CAN1_NUM_BUFF_MSGS > MAX_CAN_RECV) {
#if DEBUG
			printf("CAN1 message buffer is full.\n");
#endif
			CAN1_NUM_BUFF_MSGS = MAX_CAN_RECV;
		}
	}
}


int
geti2cDta(uint8_t *dta)
{
	int len = 0;
	uint32_t tout = 0;

	if (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND)) {
		return len;
	}

	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

#if DEBUG
	while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE)) {
		__NOP();
		tout++;
		if (tout > 5000) {
			//i2c_config();
			//i2c_flag_clear(I2C0, I2C_FLAG_RBNE);
			return 0;
		}
	}
#else
	while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE))
		continue;
#endif
	tout = 0;

	while (i2c_flag_get(I2C0, I2C_FLAG_RBNE)) {
		dta[len++] = i2c_data_receive(I2C0);
		for (int i = 0; i < 3000; i++) {
			__NOP();
		}

		if (len > 73) {
			//i2c_flag_clear(I2C0, I2C_FLAG_RBNE);
			return 0;
		}
	}

#if DEBUG
	while (!i2c_flag_get(I2C0, I2C_FLAG_STPDET)) {
		__NOP();
		tout++;
		if (tout > 5000) {
			//i2c_config();
			//i2c_flag_clear(I2C0, I2C_FLAG_STPDET);
			return 0;
		}
	}
#else
	while (!i2c_flag_get(I2C0, I2C_FLAG_STPDET))
		continue;
#endif
	i2c_enable(I2C0);
	return len;
}


int
sendi2cDta(uint8_t *dta, int dlen)
{
	uint32_t tout = 0;
	/* wait until ADDSEND bit is set */
	//if(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))return 0;
	while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND)) {
		__NOP();
		tout++;
		if (tout > 5000) {
			//i2c_config();
			i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
			return 0;
		}
	}
	tout = 0;
	//if(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))return 0;
	/* clear ADDSEND bit */
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

	/* wait until the transmission data register is empty */
	while (!i2c_flag_get(I2C0, I2C_FLAG_TBE)) {
		__NOP();
		tout++;
		if (tout > 5000) {
			i2c_flag_clear(I2C0, I2C_FLAG_TBE);
			return 0;
		}
	}
	tout = 0;

	for (int i = 0; i < dlen; i++) {
		/* send a data byte */
		i2c_data_transmit(I2C0, dta[i]);
		/* wait until the transmission data register is empty */
		while (!i2c_flag_get(I2C0, I2C_FLAG_TBE)) {
			__NOP();
			tout++;
			if(tout > 5000) {
				i2c_flag_clear(I2C0, I2C_FLAG_TBE);
				return 0;
			}
		}
		tout = 0;
	}

	/* the master doesn't acknowledge for the last byte */
	while (!i2c_flag_get(I2C0, I2C_FLAG_AERR)) {
		__NOP();
		tout++;
		if (tout > 5000) {
			i2c_flag_clear(I2C0, I2C_FLAG_AERR);
			return 0;
		}
	}
	/* clear the bit of AERR */
	i2c_flag_clear(I2C0, I2C_FLAG_AERR);

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
#ifdef CANFW_DBG
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
#endif
}

int
main(void)
{
	/* configure board */
	systick_config();

	/* activate GPIO clocks */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);

	setup_serial();

	/* I2C configure */
	i2c_gpio_config();
	i2c_config();

#ifdef CANFW_DBG
	printf("\r\nI2C0 initialized.");
	printf("\r\nThe speed is %d MHz.", I2C_SPEED / 1000);
#endif

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
			canSaveData(CAN0);
		}
		if (flgCAN1Get) {
			flgCAN1Get = 0;
			canSaveData(CAN1);
		}
	}
}

void
i2c_loop(uint8_t *i2cDtaFromRP2040, uint8_t *dtaSendToRP2040)
{
	int len = geti2cDta(i2cDtaFromRP2040);
	int i;

	if (len < 1)
		return;

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
		if (CAN0_NUM_BUFF_MSGS > 0) {
			for (i = 0; i < 8; i++) {
				dtaSendToRP2040[i] = CAN0_DATA_BUFFER[can0_buffer_index][i];
			}
			sendi2cDta(dtaSendToRP2040, 8);
		}
		break;
	case REG_ADDR_RECV1:
		if (CAN0_NUM_BUFF_MSGS > 0) {
			if (CAN0_DATA_BUFFER[can0_buffer_index][7] <= 32) {
				for (i = 0; i<CAN0_DATA_BUFFER[can0_buffer_index][7]; i++) {
					dtaSendToRP2040[i] = CAN0_DATA_BUFFER[can0_buffer_index][8 + i];
				}
				sendi2cDta(dtaSendToRP2040, CAN0_DATA_BUFFER[can0_buffer_index][7]);
				can0_buffer_index--;
				if (can0_buffer_index < 0) {
					can0_buffer_index = MAX_CAN_RECV - 1;
				}
				CAN0_NUM_BUFF_MSGS--;
			} else {
				for (i = 0; i < 32; i++) {
					dtaSendToRP2040[i] = CAN0_DATA_BUFFER[can0_buffer_index][8 + i];
				}
				sendi2cDta(dtaSendToRP2040, 32);
			}
		}
		break;
	case REG_ADDR_RECV2:
		if (CAN0_NUM_BUFF_MSGS && (CAN0_DATA_BUFFER[can0_buffer_index][7] > 32)) {
			for (i = 0; i < (CAN0_DATA_BUFFER[can0_buffer_index][7] - 32); i++) {
				dtaSendToRP2040[i] = CAN0_DATA_BUFFER[can0_buffer_index][40 + i];
			}
			sendi2cDta(dtaSendToRP2040, CAN0_DATA_BUFFER[can0_buffer_index][7] - 32);
			can0_buffer_index--;
			if (can0_buffer_index < 0) {
				can0_buffer_index = MAX_CAN_RECV - 1;
			}
			CAN0_NUM_BUFF_MSGS--;
		}
		break;
	case CAN1_RECV_INFO:
		if (CAN1_NUM_BUFF_MSGS > 0) {
			for(i = 0; i < 8; i++) {
				dtaSendToRP2040[i] = CAN1_DATA_BUFFER[can1_buffer_index][i];
			}
			sendi2cDta(dtaSendToRP2040, 8);
		}
		break;
	case REG1_ADDR_RECV1:
		if (CAN1_NUM_BUFF_MSGS > 0) {
			if (CAN1_DATA_BUFFER[can1_buffer_index][7] <= 32) {
				for (i = 0; i<CAN1_DATA_BUFFER[can1_buffer_index][7]; i++) {
					dtaSendToRP2040[i] = CAN1_DATA_BUFFER[can1_buffer_index][8 + i];
				}
				sendi2cDta(dtaSendToRP2040, CAN1_DATA_BUFFER[can1_buffer_index][7]);
				can1_buffer_index--;
				if (can1_buffer_index < 0) {
					can1_buffer_index = MAX_CAN_RECV - 1;
				}
				CAN1_NUM_BUFF_MSGS--;
			} else {
				for (i = 0; i < 32; i++) {
					dtaSendToRP2040[i] = CAN1_DATA_BUFFER[can1_buffer_index][8 + i];
				}
				sendi2cDta(dtaSendToRP2040, 32);
			}
		}
		break;
	case REG1_ADDR_RECV2:
		if (CAN1_NUM_BUFF_MSGS && (CAN1_DATA_BUFFER[can1_buffer_index][7] > 32)) {
			for (i = 0; i < (CAN1_DATA_BUFFER[can1_buffer_index][7] - 32); i++) {
				dtaSendToRP2040[i] = CAN1_DATA_BUFFER[can1_buffer_index][40 + i];
			}
			sendi2cDta(dtaSendToRP2040, CAN1_DATA_BUFFER[can1_buffer_index][7] - 32);
			can1_buffer_index--;
			if (can1_buffer_index < 0) {
				can1_buffer_index = MAX_CAN_RECV - 1;
			}
			CAN1_NUM_BUFF_MSGS--;
		}
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
	default:
		break;
	}
}

#if 0
#ifdef CANFW_DBG
/* retarget the C library printf function to the usart */
int fputc(int ch, FILE *f)
{
	usart_data_transmit(USART0, (uint8_t)ch);
	while (RESET == usart_flag_get(USART0, USART_FLAG_TBE));
	return ch;
}
#endif
#endif