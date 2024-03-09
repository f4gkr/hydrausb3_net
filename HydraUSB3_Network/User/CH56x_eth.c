/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_eth.c
* Author             : WCH, bvernoux, sazarian
* Version            : V0.1
* Date               : 2024/04/03
* Description
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* Copyright (c) 2024 Sylvain AZARIAN
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_eth.h"
#include "string.h"
#include "CH56x_flash.h"

__attribute__ ((aligned(16))) ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];    /* MAC receive descriptor, 16-byte aligned*/

__attribute__ ((aligned(16))) ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];    /* MAC send descriptor, 16-byte aligned */

__attribute__ ((aligned(16))) uint8_t MACRxBuf[ETH_RXBUFNB*ETH_RX_BUF_SZE];    /* MAC receive buffer, 16-byte aligned */

__attribute__ ((aligned(4))) uint8_t  MACTxBuf[ETH_TXBUFNB*ETH_TX_BUF_SZE];    /* MAC send buffer, 4-byte aligned */


//ETH_DMADESCTypeDef *pDMARxSet;
//ETH_DMADESCTypeDef *pDMATxSet;

ETH_DMADESCTypeDef *DMATxDescToSet;
ETH_DMADESCTypeDef *DMARxDescToGet;

volatile uint8_t LinkSta = 0; //0: No valid link established   1:Valid link established

/*********************************************************************
 * @fn      ETH_StructInit
 *
 * @brief   Fills each ETH_InitStruct member with its default value.
 *
 * @param   ETH_InitStruct - pointer to a ETH_InitTypeDef structure
 *        which will be initialized.
 *
 * @return  none
 */
void ETH_StructInit(ETH_InitTypeDef *ETH_InitStruct)
{
    /*------------------------   MAC   -----------------------------------*/
    ETH_InitStruct->ETH_AutoNegotiation = ETH_AutoNegotiation_Disable;
    ETH_InitStruct->ETH_Watchdog = ETH_Watchdog_Enable;
    ETH_InitStruct->ETH_Jabber = ETH_Jabber_Enable;
    ETH_InitStruct->ETH_InterFrameGap = ETH_InterFrameGap_96Bit;
    ETH_InitStruct->ETH_CarrierSense = ETH_CarrierSense_Enable;
    ETH_InitStruct->ETH_Speed = ETH_Speed_10M;
    ETH_InitStruct->ETH_ReceiveOwn = ETH_ReceiveOwn_Enable;
    ETH_InitStruct->ETH_LoopbackMode = ETH_LoopbackMode_Disable;
    ETH_InitStruct->ETH_Mode = ETH_Mode_HalfDuplex;
    ETH_InitStruct->ETH_ChecksumOffload = ETH_ChecksumOffload_Disable;
    ETH_InitStruct->ETH_RetryTransmission = ETH_RetryTransmission_Enable;
    ETH_InitStruct->ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
    ETH_InitStruct->ETH_BackOffLimit = ETH_BackOffLimit_10;
    ETH_InitStruct->ETH_DeferralCheck = ETH_DeferralCheck_Disable;
    ETH_InitStruct->ETH_ReceiveAll = ETH_ReceiveAll_Disable;
    ETH_InitStruct->ETH_SourceAddrFilter = ETH_SourceAddrFilter_Disable;
    ETH_InitStruct->ETH_PassControlFrames = ETH_PassControlFrames_BlockAll;
    ETH_InitStruct->ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Disable;
    ETH_InitStruct->ETH_DestinationAddrFilter = ETH_DestinationAddrFilter_Normal;
    ETH_InitStruct->ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
    ETH_InitStruct->ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
    ETH_InitStruct->ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
    ETH_InitStruct->ETH_HashTableHigh = 0x0;
    ETH_InitStruct->ETH_HashTableLow = 0x0;
    ETH_InitStruct->ETH_PauseTime = 0x0;
    ETH_InitStruct->ETH_ZeroQuantaPause = ETH_ZeroQuantaPause_Disable;
    ETH_InitStruct->ETH_PauseLowThreshold = ETH_PauseLowThreshold_Minus4;
    ETH_InitStruct->ETH_UnicastPauseFrameDetect = ETH_UnicastPauseFrameDetect_Disable;
    ETH_InitStruct->ETH_ReceiveFlowControl = ETH_ReceiveFlowControl_Disable;
    ETH_InitStruct->ETH_TransmitFlowControl = ETH_TransmitFlowControl_Disable;
    ETH_InitStruct->ETH_VLANTagComparison = ETH_VLANTagComparison_16Bit;
    ETH_InitStruct->ETH_VLANTagIdentifier = 0x0;
    /*------------------------   DMA   -----------------------------------*/
    ETH_InitStruct->ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Disable;
    ETH_InitStruct->ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
    ETH_InitStruct->ETH_FlushReceivedFrame = ETH_FlushReceivedFrame_Enable;
    ETH_InitStruct->ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
    ETH_InitStruct->ETH_TransmitThresholdControl = ETH_TransmitThresholdControl_64Bytes;
    ETH_InitStruct->ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
    ETH_InitStruct->ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;
    ETH_InitStruct->ETH_ReceiveThresholdControl = ETH_ReceiveThresholdControl_64Bytes;
    ETH_InitStruct->ETH_SecondFrameOperate = ETH_SecondFrameOperate_Disable;
    ETH_InitStruct->ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
    ETH_InitStruct->ETH_FixedBurst = ETH_FixedBurst_Disable;
    ETH_InitStruct->ETH_RxDMABurstLength = ETH_RxDMABurstLength_1Beat;
    ETH_InitStruct->ETH_TxDMABurstLength = ETH_TxDMABurstLength_1Beat;
    ETH_InitStruct->ETH_DescriptorSkipLength = 0x0;
    ETH_InitStruct->ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_1_1;
}


/*******************************************************************************
 * @fn     ETH_SoftwareReset
 *
 * @brief  ETH software reset
 *
 * @return   None
 */
void ETH_SoftwareReset(void)
{
    /* Set the SWR bit: resets all MAC subsystem internal registers and logic */
    /* After reset all the registers holds their respective reset values */
    ETH->DMABMR |= ETH_DMABMR_SR;
}

/*******************************************************************************
 * @fn       RGMII_TXC_Delay
 *
 * @brief    ETH send clock polarity and timing adjustment
 *
 * @param    clock_polarity - send clock polarity
 *           delay_time - delay time(unit - half nanosecond)
 *
 * @return   None
 */
void RGMII_TXC_Delay(uint8_t clock_polarity,uint8_t delay_time)
{
    if(clock_polarity)
        ETH->MACCR |= (uint32_t)(1<<1);
    else
        ETH->MACCR &= ~(uint32_t)(1<<1);

    if(delay_time <= 7)
        ETH->MACCR |= (uint32_t)(delay_time<<29);
    else
        printf("Error:delay_time is out of range!\n");
}


/*******************************************************************************
 * @fn     ETH_ReadPHYRegister
 *
 * @brief  Read PHY register
 *
 * @param   PHYAddress - PHY address
 *          PHYReg - PHY register address
 *
 * @return   Value of PHY register
 */
uint16_t ETH_ReadPHYRegister(uint16_t PHYAddress, uint16_t PHYReg)
{
    uint32_t tmpreg = 0;
    uint32_t timeout = 0;

    /* Get the ETHERNET MACMIIAR value */
    tmpreg = ETH->MACMIIAR;
    /* Keep only the CSR Clock Range CR[2:0] bits value */
    tmpreg &= ~MACMIIAR_CR_MASK;
    /* Prepare the MII address register value */
    tmpreg |= (((uint32_t)PHYAddress<<11) & ETH_MACMIIAR_PA); /* Set the PHY device address */
    tmpreg |= (((uint32_t)PHYReg<<6) & ETH_MACMIIAR_MR);      /* Set the PHY register address */
    tmpreg &= ~ETH_MACMIIAR_MW;                              /* Set the read mode */
    tmpreg |= ETH_MACMIIAR_MB;                               /* Set the MII Busy bit */
    /* Write the result value into the MII Address register */
    ETH->MACMIIAR = tmpreg;
    /* Check for the Busy flag */
    do
    {
        timeout++;
        tmpreg = ETH->MACMIIAR;
    } while ((tmpreg & ETH_MACMIIAR_MB) && (timeout < (uint32_t)PHY_READ_TO));
    /* Return ERROR in case of timeout */
    if(timeout == PHY_READ_TO)
    {
        return (uint16_t)ETH_ERROR;
    }
    /* Return data register value */
    return (uint16_t)(ETH->MACMIIDR);
}

/*******************************************************************************
 * @fn      ETH_WritePHYRegister
 *
 * @brief   Write PHY register
 *
 * @param   PHYAddress - PHY address
 *          PHYReg - PHY register address
 *          PHYValue - Value will be written of PHY register
 *
 * @return   Execution status
 */
uint32_t ETH_WritePHYRegister(uint16_t PHYAddress, uint16_t PHYReg, uint16_t PHYValue)
{
    uint32_t tmpreg = 0;
    uint32_t timeout = 0;

    /* Get the ETHERNET MACMIIAR value */
    tmpreg = ETH->MACMIIAR;
    /* Keep only the CSR Clock Range CR[2:0] bits value */
    tmpreg &= ~MACMIIAR_CR_MASK;
    /* Prepare the MII register address value */
    tmpreg |= (((uint32_t)PHYAddress<<11) & ETH_MACMIIAR_PA); /* Set the PHY device address */
    tmpreg |= (((uint32_t)PHYReg<<6) & ETH_MACMIIAR_MR);      /* Set the PHY register address */
    tmpreg |= ETH_MACMIIAR_MW;                               /* Set the write mode */
    tmpreg |= ETH_MACMIIAR_MB;                               /* Set the MII Busy bit */
    /* Give the value to the MII data register */
    ETH->MACMIIDR = PHYValue;
    /* Write the result value into the MII Address register */
    ETH->MACMIIAR = tmpreg;
    /* Check for the Busy flag */
    do
    {
        timeout++;
        tmpreg = ETH->MACMIIAR;
    } while ((tmpreg & ETH_MACMIIAR_MB) && (timeout < (uint32_t)PHY_WRITE_TO));
    /* Return ERROR in case of timeout */
    if(timeout == PHY_WRITE_TO)
    {
        return ETH_ERROR;
    }
    /* Return SUCCESS */
    return ETH_SUCCESS;
}


/*******************************************************************************
 * @fn     ETH_DMAITConfig
 *
 * @brief  Configuration DMA interrupt
 *
 * @param   ETH_DMA_IT - Type of DMA interrupt
 *          NewState - Enable DMA interrupt or Disable DMA interrupt
 *
 * @return   None
 */
void ETH_DMAITConfig(uint32_t ETH_DMA_IT, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        /* Enable the selected ETHERNET DMA interrupts */
        ETH->DMAIER |= ETH_DMA_IT;
    }
    else
    {
        /* Disable the selected ETHERNET DMA interrupts */
        ETH->DMAIER &= ~(uint32_t)ETH_DMA_IT;
    }
}

/*******************************************************************************
 * @fn     ETH_DMAClearITPendingBit
 *
 * @brief  Clear DMA interrupt flag
 *
 * @param   ETH_DMA_IT - Type of DMA interrupt
 *
 * @return   None
 */
void ETH_DMAClearITPendingBit(uint32_t ETH_DMA_IT)
{
    /* Clear the selected ETHERNET DMA IT */
    ETH->DMASR = (uint32_t) ETH_DMA_IT;
}

/*******************************************************************************
 * @fn     ETH_DMATxDescChainInit
 *
 * @brief  transmit descriptor initialization
 *
 * @param   DMARxDescTab - pointer to the transmit descriptor table
 *          RxBuff - pointer to the transmit buffer (transmit queue)
 *          RxBuffCount - Number of transmit descriptor or transmit queue
 *
 * @return   None
 */
void ETH_DMATxDescChainInit(ETH_DMADESCTypeDef *DMATxDescTab, uint8_t* TxBuff, uint32_t TxBuffCount)
{
    uint8_t i = 0;
    ETH_DMADESCTypeDef *DMATxDesc;

    DMATxDescToSet = DMATxDescTab;

    for(i = 0; i < TxBuffCount; i++)
    {
        DMATxDesc = DMATxDescTab + i;
        DMATxDesc->Status = ETH_DMATxDesc_TCH | ETH_DMATxDesc_IC;
        DMATxDesc->Buffer1Addr = (uint32_t)(&TxBuff[i * ETH_MAX_PACKET_SIZE]);

        if(i < (TxBuffCount - 1))
        {
            DMATxDesc->Buffer2NextDescAddr = (uint32_t)(DMATxDescTab + i + 1);
        }
        else
        {
            DMATxDesc->Buffer2NextDescAddr = (uint32_t)DMATxDescTab;
        }
    }

    ETH->DMATDLAR = (uint32_t)DMATxDescTab;
}

/*******************************************************************************
 * @fn     ETH_DMARxDescChainInit
 *
 * @brief  Receive descriptor initialization
 *
 * @param   DMARxDescTab - pointer to the receive descriptor table
 *          RxBuff - pointer to the receive buffer (receive queue)
 *          RxBuffCount - Number of receive descriptor or receive queue
 *
 * @return   None
 **/
void ETH_DMARxDescChainInit(ETH_DMADESCTypeDef *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount)
{
    uint8_t i = 0;
    ETH_DMADESCTypeDef *DMARxDesc;

    DMARxDescToGet = DMARxDescTab;

    for(i = 0; i < RxBuffCount; i++)
    {
        DMARxDesc = DMARxDescTab + i;
        DMARxDesc->Status = ETH_DMARxDesc_OWN;
        DMARxDesc->ControlBufferSize = (uint32_t)ETH_MAX_PACKET_SIZE;
        DMARxDesc->Buffer1Addr = (uint32_t)(&RxBuff[i * ETH_MAX_PACKET_SIZE]);

        if(i < (RxBuffCount - 1))
        {
            DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab + i + 1);
        }
        else
        {
            DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab);
        }
    }

    ETH->DMARDLAR = (uint32_t)DMARxDescTab;
}

/*********************************************************************
 * @fn      ETH_GetMacAddr
 *
 * @brief   Get the MAC address
 *
 * @return  none.
 */
void ETH_GetMacAddr( uint8_t *p )
{
    uint8_t rc, buf[8];

    rc = FLASH_ROMA_READ((0x7FFE4 - 0x8000), (puint32_t)buf, 8);
    for(rc = 0; rc < 6; rc++)
        p[0 + rc] = buf[5 - rc];
}

/*********************************************************************
 * @fn      ETH_GPIOInit
 *
 * @brief   PHY RGMII interface GPIO initialization.
 *
 * @return  none
 */
void ETH_GPIOInit(void)
{
    R32_PA_DIR |=  bEMCO;
    R32_PA_DIR &= ~bEMCI;
    R32_PB_DIR |=  bMDCK | bETHT3 | bETHT2 | \
                   bETHT1 | bETHT0 | bETHTEN | bETHTC;
    R32_PB_DIR &= ~(bETHRC | bETHR3 | bETHR2 | \
                    bETHR1 | bETHR0 | bETHRDV | bMDIO);
}

/*********************************************************************
 * @fn      ETH_Start
 *
 * @brief   Enable ETH MAC and DMA reception/transmission.
 *
 * @return  none
 */
void ETH_Start(void)
{
    ETH->MACCR |= ETH_MACCR_RE;
    ETH->DMAOMR |= ETH_DMAOMR_FTF;
    ETH->DMAOMR |= ETH_DMAOMR_ST;
    ETH->DMAOMR |= ETH_DMAOMR_SR;
}

/*********************************************************************
 * @fn      ETH_PHYLink
 *
 * @brief   Configure MAC parameters after the PHY Link is successful.
 *
 * @param   none.
 *
 * @return  none.
 */
void ETH_PHYLink( void )
{
    uint32_t phy_stat;

    ETH_WritePHYRegister( PHY_ADDRESS, 0x1F,0x0a43 );
    /*In some cases the status is not updated in time,
     * so read this register twice to get the correct status value.*/
    ETH_ReadPHYRegister( PHY_ADDRESS, 0x1A);
    phy_stat = ETH_ReadPHYRegister( PHY_ADDRESS, 0x1A);

    if( phy_stat & 0x04 )
    {
        if( phy_stat & 0x08 )
        {
            ETH->MACCR |= ETH_Mode_FullDuplex;
        }
        else
        {
            ETH->MACCR &= ~ETH_Mode_FullDuplex;
        }
        if( (phy_stat & 0x30) == 0x00 )
        {
            ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
        }
        else if( (phy_stat & 0x30) == 0x10 )
        {
            ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
            ETH->MACCR |= ETH_Speed_100M;
        }
        else if( (phy_stat & 0x30) == 0x20 )
        {
            ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
            ETH->MACCR |= ETH_Speed_1000M;
        }
        ETH_Start( );
        LinkSta = 1;
    }
    else {
        LinkSta = 0;
    }
    phy_stat = ETH_ReadPHYRegister( PHY_ADDRESS, 0x1D);   /* Clear the Interrupt status */
}

/*********************************************************************
 * @fn      ETH_IRQHandler
 *
 * @brief   Ethernet Interrupt Service program
 *
 * @return  none
 */


__attribute__((interrupt("WCH-Interrupt-fast"))) void ETH_IRQHandler(void)
{
    uint32_t int_sta, length, buffer;

    int_sta = ETH->DMASR;

    if (int_sta & ETH_DMA_IT_AIS)
    {
        if (int_sta & ETH_DMA_IT_RBU)
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_RBU);
        }
        ETH_DMAClearITPendingBit(ETH_DMA_IT_AIS);
    }
    if( int_sta & ETH_DMA_IT_NIS )
    {
        if( int_sta & ETH_DMA_IT_R )
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
            /*If you don't use the Ethernet library,
             * you can do some data processing operations here*/
            for(uint8_t i = 0; i < ETH_RXBUFNB; i++)
            {
                if(!(DMARxDescToGet->Status & ETH_DMARxDesc_OWN))
                {
                    if( !(DMARxDescToGet->Status & ETH_DMARxDesc_ES) &&\
                      (DMARxDescToGet->Status & ETH_DMARxDesc_LS) &&\
                      (DMARxDescToGet->Status & ETH_DMARxDesc_FS) &&\
                      !(DMARxDescToGet->Status & ETH_DMARxDesc_MAMPCE))
                    {
                        /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
                        length = (DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> 16;
                        /* Get the addrees of the actual buffer */
                        buffer = DMARxDescToGet->Buffer1Addr;

                        /* Do something*/
                        printf("rec data:%d bytes\r\n",length);
                        printf("data:%x\r\n",*((uint8_t *)buffer));
                    }
                }
                DMARxDescToGet->Status = ETH_DMARxDesc_OWN;
                DMARxDescToGet = (ETH_DMADESCTypeDef *)DMARxDescToGet->Buffer2NextDescAddr;
            }
        }
        if( int_sta & ETH_DMA_IT_T )
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_T);
        }
        if (int_sta & ETH_DMA_IT_TBU)
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_TBU);
        }
        ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
    }
}

/*******************************************************************************
 * @fn     ETH_ClockEnable
 *
 * @brief  ETH peripheral clock initialization
 *
 * @return   None
 */
void ETH_ClockEnable(void)
{
    R8_SAFE_ACCESS_SIG = 0x57;   /* enable safe access mode */
    R8_SAFE_ACCESS_SIG = 0xa8;

    R8_CLK_MOD_AUX = (R8_CLK_MOD_AUX & (~RB_SRC_125M_MSK)) | RB_EXT_125M_EN;

    R8_SAFE_ACCESS_SIG = 0;
}

/*******************************************************************************
 * @fn      ETH_RegInit
 *
 * @brief   ETH register & PHY register initialization
 *
 * @param   ETH_InitStruct - pointer to the ETH config struct
 *          PHYAddress - PHY address
 *
 * @return  Execution status
 */
uint32_t ETH_RegInit(ETH_InitTypeDef* ETH_InitStruct, uint16_t PHYAddress)
{
    uint32_t tmpreg = 0;

    /*---------------------- Physical layer configuration -------------------*/
    /* Set the SMI interface clock, set as the main frequency divided by 42  */
    tmpreg = ETH->MACMIIAR;
    tmpreg &= MACMIIAR_CR_MASK;
    tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div42;
    ETH->MACMIIAR = (uint32_t)tmpreg;

    /*------------------------ MAC register configuration  ----------------------- --------------------*/
    tmpreg = ETH->MACCR;
    tmpreg &= MACCR_CLEAR_MASK;
    tmpreg |= (uint32_t)(ETH_InitStruct->ETH_AutoNegotiation |
                  ETH_InitStruct->ETH_Watchdog |
                  ETH_InitStruct->ETH_Jabber |
                  ETH_InitStruct->ETH_InterFrameGap |
                  ETH_InitStruct->ETH_CarrierSense |
                  ETH_InitStruct->ETH_Speed |
                  ETH_InitStruct->ETH_ReceiveOwn |
                  ETH_InitStruct->ETH_LoopbackMode |
                  ETH_InitStruct->ETH_Mode |
                  ETH_InitStruct->ETH_ChecksumOffload |
                  ETH_InitStruct->ETH_RetryTransmission |
                  ETH_InitStruct->ETH_AutomaticPadCRCStrip |
                  ETH_InitStruct->ETH_BackOffLimit |
                  ETH_InitStruct->ETH_DeferralCheck);
    /* Write MAC Control Register */
    ETH->MACCR = (uint32_t)tmpreg;

    ETH->MACFFR = (uint32_t)(ETH_InitStruct->ETH_ReceiveAll |
                          ETH_InitStruct->ETH_SourceAddrFilter |
                          ETH_InitStruct->ETH_PassControlFrames |
                          ETH_InitStruct->ETH_BroadcastFramesReception |
                          ETH_InitStruct->ETH_DestinationAddrFilter |
                          ETH_InitStruct->ETH_PromiscuousMode |
                          ETH_InitStruct->ETH_MulticastFramesFilter |
                          ETH_InitStruct->ETH_UnicastFramesFilter);
    /*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
    /* Write to ETHERNET MACHTHR */
    ETH->MACHTHR = (uint32_t)ETH_InitStruct->ETH_HashTableHigh;
    /* Write to ETHERNET MACHTLR */
    ETH->MACHTLR = (uint32_t)ETH_InitStruct->ETH_HashTableLow;
    /*----------------------- ETHERNET MACFCR Configuration --------------------*/
    /* Get the ETHERNET MACFCR value */
    tmpreg = ETH->MACFCR;
    /* Clear xx bits */
    tmpreg &= MACFCR_CLEAR_MASK;
    tmpreg |= (uint32_t)((ETH_InitStruct->ETH_PauseTime << 16) |
                     ETH_InitStruct->ETH_ZeroQuantaPause |
                     ETH_InitStruct->ETH_PauseLowThreshold |
                     ETH_InitStruct->ETH_UnicastPauseFrameDetect |
                     ETH_InitStruct->ETH_ReceiveFlowControl |
                     ETH_InitStruct->ETH_TransmitFlowControl);
    ETH->MACFCR = (uint32_t)tmpreg;

    ETH->MACVLANTR = (uint32_t)(ETH_InitStruct->ETH_VLANTagComparison |
                               ETH_InitStruct->ETH_VLANTagIdentifier);

    tmpreg = ETH->DMAOMR;
    tmpreg &= DMAOMR_CLEAR_MASK;
    tmpreg |= (uint32_t)(ETH_InitStruct->ETH_DropTCPIPChecksumErrorFrame |
                    ETH_InitStruct->ETH_ReceiveStoreForward |
                    ETH_InitStruct->ETH_FlushReceivedFrame |
                    ETH_InitStruct->ETH_TransmitStoreForward |
                    ETH_InitStruct->ETH_TransmitThresholdControl |
                    ETH_InitStruct->ETH_ForwardErrorFrames |
                    ETH_InitStruct->ETH_ForwardUndersizedGoodFrames |
                    ETH_InitStruct->ETH_ReceiveThresholdControl |
                    ETH_InitStruct->ETH_SecondFrameOperate);
    ETH->DMAOMR = (uint32_t)tmpreg;

    /* Reset the physical layer */
    ETH_WritePHYRegister(PHYAddress, PHY_BCR, PHY_Reset);
    return ETH_SUCCESS;
}

/*********************************************************************
 * @fn      RTL8211FS_Interrupt_Init
 *
 * @brief   Enable Link Status Change Interrupt
 *          and Auto-Negotiation Completed Interrupt
 *
 * @return  none
 */
void RTL8211FS_Interrupt_Init(void)
{
    uint16_t RegValue;

    ETH_WritePHYRegister(PHY_ADDRESS, 0x1f, 0x0a42 );
    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, 0x12);
    /* Enable Link Status Change Interrupt and Auto-Negotiation Completed Interrupt*/
    RegValue |= (1<<4)|(1<<3);
    ETH_WritePHYRegister(PHY_ADDRESS, 0x12, RegValue );

    ETH_WritePHYRegister(PHY_ADDRESS, 0x1f, 0x0a43 );
    ETH_ReadPHYRegister(PHY_ADDRESS, 0x1d);        /* Clear the Interrupt status */
}

/*********************************************************************
 * @fn      GPIO_Interrupt_Init
 *
 * @brief   Interrupt pin initialization
 *
 * @return  none
 */
void GPIO_Interrupt_Init(void)
{
    /* PB15 is set to GPIO interrupt trigger pin, Falling edge trigger interrupt  */
    R32_PB_PU = (1<<15);                            /* PB15 pull-up */
    GPIOB_ITModeCfg( GPIO_Pin_15, GPIO_ITMode_FallEdge );
    R8_GPIO_INT_FLAG = 0xff;                        /* Clear the Interrupt status */
    PFIC_EnableIRQ(GPIO_IRQn);
}

/*******************************************************************************
 * @fn       ETH_MACAddressConfig
 *
 * @brief    Mac address initialization
 *
 * @param    MacAddr - Mac address register
 *           PHYAddress - pointer to the Mac address string
 *
 * @return   None
 */
void ETH_MACAddressConfig(uint32_t MacAddr, uint8_t *Addr)
{
    uint32_t tmpreg;

    /* Calculate the selected MAC address high register */
    tmpreg = ((uint32_t)Addr[5] << 8) | (uint32_t)Addr[4];

    /* Load the selected MAC address high register */
    (*(__IO uint32_t *) (ETH_MAC_ADDR_HBASE + MacAddr)) = tmpreg;

    /* Calculate the selected MAC address low register */
    tmpreg = ((uint32_t)Addr[3] << 24) | ((uint32_t)Addr[2] << 16) | ((uint32_t)Addr[1] << 8) | Addr[0];

    /* Load the selected MAC address low register */
    (*(__IO uint32_t *) (ETH_MAC_ADDR_LBASE + MacAddr)) = tmpreg;
}

/*********************************************************************
 * @fn      ETH_Configuration
 *
 * @brief   Ethernet configure.
 *
 * @return  none
 */
void ETH_Configuration( uint8_t *macAddr )
{
    ETH_InitTypeDef ETH_InitStructure;
    uint16_t timeout = 10000;

    /* Enable Ethernet 125MHz clock */
    ETH_ClockEnable();

    /* Enable RGMII GPIO */
    ETH_GPIOInit();

    /* Software reset */
    ETH_SoftwareReset();

    /* Wait for software reset */
    do{
        DelayUs(10);
        if( !--timeout )  break;
    }while(ETH->DMABMR & ETH_DMABMR_SR);

    /* Enable MAC Transmitter */
    ETH->MACCR |= ETH_MACCR_TE;

    /* Mask the interrupt that Tx good frame count counter reaches half the maximum value */
    ETH->MMCTIMR = ETH_MMCTIMR_TGFM;
    /* Mask the interrupt that Rx good unicast frames counter reaches half the maximum value */
    /* Mask the interrupt that Rx crc error counter reaches half the maximum value */
    ETH->MMCRIMR = ETH_MMCRIMR_RGUFM | ETH_MMCRIMR_RFCEM;

    /* Configure MAC address */
    ETH_MACAddressConfig(ETH_MAC_Address0,macAddr);

    /* ETHERNET Configuration */
    /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
    ETH_StructInit(&ETH_InitStructure);
    /* Fill ETH_InitStructure parameters */
    /*------------------------   MAC   -----------------------------------*/
    ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
    ETH_InitStructure.ETH_Speed = ETH_Speed_1000M;
    ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
    ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
    ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
    ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
    /* Filter function configuration */
    ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
    ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
    ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
    ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
    ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
    /*------------------------   DMA   -----------------------------------*/
    /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
    the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum,
    if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
    ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
    ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
    ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
    ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Enable;
    ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Enable;
    ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Disable;

    /* Configure Ethernet */
    ETH_RegInit( &ETH_InitStructure, PHY_ADDRESS );

    /* Enable the Ethernet Interrupt */
    ETH_DMAITConfig(ETH_DMA_IT_NIS |\
                ETH_DMA_IT_R |\
                ETH_DMA_IT_T |\
                ETH_DMA_IT_AIS |\
                ETH_DMA_IT_RBU,\
                ENABLE);
    /* Set the interrupt priority */
    PFIC_SetPriority(ETH_IRQn, 0x20);

    /* ETH send clock polarity and timing adjustment */
    RGMII_TXC_Delay(0, 4);

    /* Enable the RTL8211FS Interrupt */
    RTL8211FS_Interrupt_Init();

    /* Enable the GPIO Interrupt */
    GPIO_Interrupt_Init();
}

/*********************************************************************
 * @fn      MACRAW_Tx
 *
 * @brief   Ethernet sends data frames in chain mode.
 *
 * @param   buff   send buffer pointer
 *          len    Send data length
 *
 * @return  Send status.
 */
uint32_t MACRAW_Tx(uint8_t *buff, uint16_t len)
{
    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (uint32_t)RESET)
    {
        /* Return ERROR: OWN bit set */
        return ETH_ERROR;
    }
    /* Setting the Frame Length: bits[12:0] */
    DMATxDescToSet->ControlBufferSize = (len & ETH_DMATxDesc_TBS1);

    memcpy((uint8_t *)DMATxDescToSet->Buffer1Addr, buff, len);

    /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
    DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;

    /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
    DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

    /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
    /* Clear TBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_TBUS;
    /* Resume DMA transmission*/
    ETH->DMATPDR = 0;

    /* Update the ETHERNET DMA global Tx descriptor with next Tx descriptor */
    /* Chained Mode */
    /* Selects the next DMA Tx descriptor list for next buffer to send */
    DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);
    /* Return SUCCESS */
    return ETH_SUCCESS;
}

/*********************************************************************
 * @fn      ETH_Init
 *
 * @brief   Ethernet initialization.
 *
 * @return  none
 */
void ETH_Init( uint8_t *macAddr )
{
    ETH_Configuration( macAddr );
    ETH_DMATxDescChainInit(DMATxDscrTab, MACTxBuf, ETH_TXBUFNB);
    ETH_DMARxDescChainInit(DMARxDscrTab, MACRxBuf, ETH_RXBUFNB);
    PFIC_EnableIRQ(ETH_IRQn);
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void GPIO_IRQHandler(void) 
{
    if(GPIOB_15_ReadITFlagBit())
    {
        ETH_PHYLink();
    }
    GPIOB_15_ClearITFlagBit();
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void TMR1_IRQHandler(void) {
    if(R8_TMR1_INT_FLAG & RB_TMR_IF_CYC_END)
    {
        R8_TMR1_INTER_EN = RB_TMR_IF_CYC_END;
        R8_TMR1_INT_FLAG =  RB_TMR_IF_CYC_END;
    }
}