/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under 
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING 
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT 
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR 
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE 
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software 
* and to discontinue the availability of this software.  By using this software, 
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2013 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_serial_user.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for Serial module.
* Creation Date: 2015/8/8
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt INTST0 r_uart0_interrupt_send
#pragma interrupt INTSR0 r_uart0_interrupt_receive
#pragma interrupt INTIIC10 r_iic10_interrupt
#pragma interrupt INTIICA0 r_iica0_interrupt
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern volatile uint8_t * gp_uart0_tx_address;         /* uart0 send buffer address */
extern volatile uint16_t  g_uart0_tx_count;            /* uart0 send data number */
extern volatile uint8_t * gp_uart0_rx_address;         /* uart0 receive buffer address */
extern volatile uint16_t  g_uart0_rx_count;            /* uart0 receive data number */
extern volatile uint16_t  g_uart0_rx_length;           /* uart0 receive data length */
extern volatile uint8_t   g_iic10_master_status_flag;  /* iic10 start flag for send address check by master mode */
extern volatile uint8_t * gp_iic10_tx_address;         /* iic10 send data pointer by master mode */
extern volatile uint16_t  g_iic10_tx_count;            /* iic10 send data size by master mode */
extern volatile uint8_t * gp_iic10_rx_address;         /* iic10 receive data pointer by master mode */
extern volatile uint16_t  g_iic10_rx_count;            /* iic10 receive data size by master mode */
extern volatile uint16_t  g_iic10_rx_length;           /* iic10 receive data length by master mode */
extern volatile uint8_t   g_iica0_master_status_flag;  /* iica0 master flag */ 
extern volatile uint8_t   g_iica0_slave_status_flag;   /* iica0 slave flag */
extern volatile uint8_t * gp_iica0_rx_address;         /* iica0 receive buffer address */
extern volatile uint16_t  g_iica0_rx_cnt;              /* iica0 receive data length */
extern volatile uint16_t  g_iica0_rx_len;              /* iica0 receive data count */
extern volatile uint8_t * gp_iica0_tx_address;         /* iica0 send buffer address */
extern volatile uint16_t  g_iica0_tx_cnt;              /* iica0 send data count */
/* Start user code for global. Do not edit comment generated here */
extern uint8_t rt;
extern uint16_t youmen;
extern unsigned char yingji;
extern uint8_t senddate;
typedef struct PID{float P,pout,I,iout,D,dout,IMAX,OUT;}PID;
extern PID PID_ROL,PID_PIT,PID_YAW;
void receivedo(void);


extern struct _pid{
        float kp;
			  float ki;
	      float kd;
	      float increment;
	      float increment_max;
	      float kp_out;
			  float ki_out;
	      float kd_out;
	      float pid_out;
          };

extern struct _tache{
    struct _pid shell;
    struct _pid core;	
          };
	

extern struct _ctrl{
	unsigned char  ctrlRate;
      struct _tache pitch;    
	    struct _tache roll;  
	    struct _tache yaw;   
            };
	    
extern struct _ctrl ctrl;



/*End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_uart0_interrupt_receive
* Description  : This function is INTSR0 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_uart0_interrupt_receive(void)
{
    uint8_t rx_data;
    uint8_t err_type;
    
    err_type = (uint8_t)(SSR01 & 0x0007U);
    SIR01 = (uint16_t)err_type;
    rx_data = RXD0;

    if (g_uart0_rx_length > g_uart0_rx_count)
    {
        *gp_uart0_rx_address = rx_data;
        gp_uart0_rx_address++;
        g_uart0_rx_count++;

        if (g_uart0_rx_length == g_uart0_rx_count)
        {
            r_uart0_callback_receiveend();
        }
    }
}

/***********************************************************************************************************************
* Function Name: r_uart0_interrupt_send
* Description  : This function is INTST0 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_uart0_interrupt_send(void)
{
    if (g_uart0_tx_count > 0U)
    {
        TXD0 = *gp_uart0_tx_address;
        gp_uart0_tx_address++;
        g_uart0_tx_count--;
    }
    else
    {
        r_uart0_callback_sendend();
    }
}

/***********************************************************************************************************************
* Function Name: r_uart0_callback_receiveend
* Description  : This function is a callback function when UART0 finishes reception.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_uart0_callback_receiveend(void)
{
    /* Start user code. Do not edit comment generated here */
    receivedo();	
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_uart0_callback_sendend
* Description  : This function is a callback function when UART0 finishes transmission.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_uart0_callback_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_iic10_interrupt
* Description  : This function is INTIIC10 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_iic10_interrupt(void)
{
    if (((SSR02 & _0002_SAU_PARITY_ERROR) == 0x0002U) && (g_iic10_tx_count != 0U))
    {
        r_iic10_callback_master_error(MD_NACK);
    }
    else
    {
        /* Control for master send */
        if ((g_iic10_master_status_flag & _01_SAU_IIC_SEND_FLAG) == 1U)
        {
            if (g_iic10_tx_count > 0U)
            {
                SIO10 = *gp_iic10_tx_address;
                gp_iic10_tx_address++;
                g_iic10_tx_count--;
            }
            else
            {
                R_IIC10_StopCondition();
                r_iic10_callback_master_sendend();
            }
        }
        /* Control for master receive */
        else 
        {
            if ((g_iic10_master_status_flag & _04_SAU_IIC_SENDED_ADDRESS_FLAG) == 0U)
            {
                ST0 |= _0004_SAU_CH2_STOP_TRG_ON;
                SCR02 &= ~_C000_SAU_RECEPTION_TRANSMISSION;
                SCR02 |= _4000_SAU_RECEPTION;
                SS0 |= _0004_SAU_CH2_START_TRG_ON;
                g_iic10_master_status_flag |= _04_SAU_IIC_SENDED_ADDRESS_FLAG;
                
                if (g_iic10_rx_length == 1U)
                {
                    SOE0 &= ~_0004_SAU_CH2_OUTPUT_ENABLE;    /* disable IIC10 out */
                }
                
                SIO10 = 0xFFU;
            }
            else
            {
                if (g_iic10_rx_count < g_iic10_rx_length)
                {
                    *gp_iic10_rx_address = SIO10;
                    gp_iic10_rx_address++;
                    g_iic10_rx_count++;
                    
                    if (g_iic10_rx_count == (g_iic10_rx_length - 1U))
                    {
                        SOE0 &= ~_0004_SAU_CH2_OUTPUT_ENABLE;    /* disable IIC10 out */
                        SIO10 = 0xFFU;
                    }
                    else if (g_iic10_rx_count == g_iic10_rx_length)
                    {
                        R_IIC10_StopCondition();
                        r_iic10_callback_master_receiveend();
                    }
                    else
                    {
                        SIO10 = 0xFFU;
                    }
                }
            }
        }
    }
}

/***********************************************************************************************************************
* Function Name: r_iic10_callback_master_error
* Description  : This function is a callback function when IIC10 master error occurs.
* Arguments    : flag -
*                    status flag
* Return Value : None
***********************************************************************************************************************/
static void r_iic10_callback_master_error(MD_STATUS flag)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_iic10_callback_master_receiveend
* Description  : This function is a callback function when IIC10 finishes master reception.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_iic10_callback_master_receiveend(void)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_iic10_callback_master_sendend
* Description  : This function is a callback function when IIC10 finishes master transmission.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_iic10_callback_master_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_iica0_interrupt
* Description  : This function is INTIICA0 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_iica0_interrupt(void)
{
    if ((IICS0 & _80_IICA_STATUS_MASTER) == 0x80U)
    {
        iica0_master_handler();
    }
}

/***********************************************************************************************************************
* Function Name: iica0_master_handler
* Description  : This function is IICA0 master handler.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void iica0_master_handler(void)
{
    if (!((0U == IICBSY0) && (g_iica0_tx_cnt != 0U)))
    {
        if ((g_iica0_master_status_flag & _80_IICA_ADDRESS_COMPLETE) == 0U)
        {
            if (1U == ACKD0)
            {
                g_iica0_master_status_flag |= _80_IICA_ADDRESS_COMPLETE;
                
                if (1U == TRC0)
                {
                    WTIM0 = 1U;
                    
                    if (g_iica0_tx_cnt > 0U)
                    {
                        IICA0 = *gp_iica0_tx_address;
                        gp_iica0_tx_address++;
                        g_iica0_tx_cnt--;
                    }
                }
                else
                {
                    ACKE0 = 1U;
                    WTIM0 = 0U;
                    WREL0 = 1U;
                }
            }
        }
        else
        {
            /* Master send control */
            if (1U == TRC0)
            {
                if (!((0U == ACKD0) && (g_iica0_tx_cnt != 0U)))
                {
                    if (g_iica0_tx_cnt > 0U)
                    {
                        IICA0 = *gp_iica0_tx_address;
                        gp_iica0_tx_address++;
                        g_iica0_tx_cnt--;
                    }
                }
            }
            /* Master receive control */
            else
            {
                if (g_iica0_rx_cnt < g_iica0_rx_len)
                {
                    *gp_iica0_rx_address = IICA0;
                    gp_iica0_rx_address++;
                    g_iica0_rx_cnt++;
                    
                    if (g_iica0_rx_cnt == g_iica0_rx_len)
                    {
                        ACKE0 = 0U;
                        WREL0 = 1U;
                        WTIM0 = 1U;
                    }
                    else
                    {
                        WREL0 = 1U;
                    }
                }
            }
        }
    }
}

/* Start user code for adding. Do not edit comment generated here */


void receivedo(void)
{
	if(rt==0x31)
	{
	if(youmen<62000)
	{
	youmen+=1660;//+=1660		
	}
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}	
	if(rt==0x32)
	{
	if(youmen>28800)
	{
	youmen-=1660;//-=1660
	}
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}
	////////////////////////////////////////////////////////////////////////////////////	
	if(rt==0x33)
	{
	yingji=1;
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}	
	if(rt==0x34)
	{
	yingji=0;
	youmen=28800;
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}
	///////////////////////////////////////////////////////////////////////////////////////
	/*if(rt==0x35)
	{
	senddate=1;
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}
	if(rt==0x36)
	{
	senddate=0;
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}*/
	///////////////////////////////////////////////////////////////////////////////////////		
	if(rt==0x37)
	{
	ctrl.pitch.core.kp += 0.1;
	ctrl.roll.core.kp += 0.1;
	//PID_ROL.P+=0.1;
	//PID_PIT.P+=0.1;
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}
	if(rt==0x38)
	{
	ctrl.pitch.core.kp -= 0.1;
	ctrl.roll.core.kp -= 0.1;
	//PID_ROL.P-=0.1;
	//PID_PIT.P-=0.1;
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}
	///////////////////////////////////////////////////////////////////////////////////////
	if(rt==0x39)
	{
	ctrl.pitch.core.kd += 0.01;
	ctrl.roll.core.kd += 0.01;
	//PID_ROL.D+=0.01;
	//PID_PIT.D+=0.01;
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}
	if(rt==0x30)
	{
	ctrl.pitch.core.kd -= 0.01;
	ctrl.roll.core.kd -= 0.01;	
	//PID_ROL.D-=0.01;
	//PID_PIT.D-=0.01;
	R_UART0_Receive(&rt, 1);
	P5.4=0;
	return;
	}
}         
/* End user code. Do not edit comment generated here */
