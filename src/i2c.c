/*!
    \file    i2c.c
    \brief   I2C configuration file

    \version 2022-09-16, V1.0.0, demo for GD32C10x
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32c10x.h"

#include "i2c.h"

#define I2C_SCL_PIN             GPIO_PIN_6
#define I2C_SDA_PIN             GPIO_PIN_7

static uint8_t i2c_slaveaddr = 0x41;
static uint32_t i2c_speed = 400000U;
static uint32_t i2c_dutycycle = I2C_DTCY_16_9;

/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void
i2c_gpio_config(void)
{
	/* enable GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN | I2C_SDA_PIN);
}

/*!
    \brief      configure the I2CX interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
void
i2c_config(uint8_t slaveaddr, uint32_t speed, uint32_t dutycycle)
{
	i2c_slaveaddr = slaveaddr;
	i2c_speed = speed;
	i2c_dutycycle = dutycycle;

	/* enable I2C clock */
	rcu_periph_clock_enable(RCU_I2C0);
	/* configure I2C clock */
	i2c_clock_config(I2C0, speed, dutycycle);
	/* configure I2C address */
	i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, 0, slaveaddr);
	/* enable I2C0 */
	i2c_enable(I2C0);
	/* enable acknowledge */
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

/*!
    \brief      reset I2C bus
    \param[in]  none
    \param[out] none
    \retval     none
*/
void
i2c_bus_reset(void)
{
	i2c_deinit(I2C0);
	/* configure SDA/SCL for GPIO */
	GPIO_BC(GPIOB) |= I2C_SCL_PIN;
	GPIO_BC(GPIOB) |= I2C_SDA_PIN;
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, I2C_SCL_PIN | I2C_SDA_PIN);
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	GPIO_BOP(GPIOB) |= I2C_SCL_PIN;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	GPIO_BOP(GPIOB) |= I2C_SDA_PIN;
	/* connect I2C_SCL_GPIO_PIN to I2C_SCL */
	/* connect I2C_SDA_GPIO_PIN to I2C_SDA */
	gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN | I2C_SDA_PIN);
	/* configure the I2CX interface */
	i2c_config(i2c_slaveaddr, i2c_speed, i2c_dutycycle);
}