#ifndef __DUAL_DFS_H__
#define __DUAL_DFS_H__

#define CANCONFIG_SIZE		8 + 14 * 10	/* 14 filters per peripheral. I.e 14 for CAN0, 14 for CAN1. */
/* Maximum number of messages to be buffered. For 100ms cycle times, this gives 1s lead time. */
#define MAX_CAN_RECV		10


#define CAN0_MSG_RECV_NUM	0x04
#define CAN0_SEND_MSG		0x05
#define CAN0_RECV_INFO		0x06	/* id, rtr, ext, fd */
#define REG_ADDR_RECV1		0x07
#define REG_ADDR_RECV2		0x08
#define CAN0_CONFIG		0x09
#define CAN0_SLEEP		0x0a
#define CAN0_WAKE		0x0b


#define CAN1_MSG_RECV_NUM	0x14
#define CAN1_SEND_MSG		0x15
#define CAN1_RECV_INFO		0x16
#define REG1_ADDR_RECV1		0x17
#define REG1_ADDR_RECV2		0x18
#define CAN1_CONFIG		0x19
#define CAN1_SLEEP		0x1a
#define CAN1_WAKE		0x1b

#define	DBG_PINGPONG		0x20

#endif
