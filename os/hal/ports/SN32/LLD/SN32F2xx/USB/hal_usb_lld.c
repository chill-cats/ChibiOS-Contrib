/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_usb_lld.c
 * @brief   PLATFORM USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>
#include "hal.h"

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if (SN32_USB_USE_USB1 == TRUE) || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static uint8_t nakcnt[USB_MAX_ENDPOINTS + 1] = {0, 0, 0, 0, 0};

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static union {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  .ep_mode          = USB_EP_MODE_TYPE_CTRL,
  .setup_cb         = _usb_ep0setup,
  .in_cb            = _usb_ep0in,
  .out_cb           = _usb_ep0out,
  .in_maxsize       = 0x40,
  .out_maxsize      = 0x40,
  .in_state         = &ep0_state.in,
  .out_state        = &ep0_state.out,
  .ep_buffers       = 1,
  .setup_buf        = ep0setup_buffer
};
#if (USB_USE_WAIT == TRUE) || defined(__DOXYGEN__)
#define _usb_isr_invoke_tx_complete_cb(usbp, ep) {                          \
  (usbp)->transmitting &= ~(1 << (ep));                                     \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(usbp)->epc[ep]->in_state->thread, MSG_OK);            \
  osalSysUnlockFromISR();                                                   \
}
#else
#define _usb_isr_invoke_tx_complete_cb(usbp, ep) {                          \
  (usbp)->transmitting &= ~(1 << (ep));                                     \
}
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void sn32_usb_read_fifo(usbep_t ep, uint8_t *buf, size_t sz, bool intr) {
    size_t ep_offset;
    if (ep == 0){
        ep_offset = 0;
    } else {
        ep_offset = SN32_USB->EPBUFOS[ep-1];
    }
    size_t off;
    size_t chunk;
    uint32_t data;

    //Limit size to max packet size allowed in USB ram
    //if (sz > wUSB_EPnMaxPacketSize[ep]) {
    //    sz = wUSB_EPnMaxPacketSize[ep];
    //}

    off = 0;
    while (off != sz) {
        chunk = 4;
        if (off + chunk > sz)
            chunk = sz - off;

        if(intr)
        {
            SN32_USB->RWADDR = off + ep_offset;
            SN32_USB->RWSTATUS = 0x02;
            while (SN32_USB->RWSTATUS & 0x02);
            data = SN32_USB->RWDATA;
        }
        else
        {
            SN32_USB->RWADDR2 = off + ep_offset;
            SN32_USB->RWSTATUS2 = 0x02;
            while (SN32_USB->RWSTATUS2 & 0x02);
            data = SN32_USB->RWDATA2;
        }

        //dest, src, size
        memcpy(buf, &data, chunk);

        off += chunk;
        buf += chunk;
    }
}

static void sn32_usb_write_fifo(usbep_t ep, const uint8_t *buf, size_t sz, bool intr) {
    size_t ep_offset;
    if (ep == 0){
        ep_offset = 0;
    } else {
        ep_offset = SN32_USB->EPBUFOS[ep-1];
    }
    size_t off;
    size_t chunk;
    uint32_t data;

    //Limit size to max packet size allowed in USB ram
    //if (sz > wUSB_EPnMaxPacketSize[ep]) {
    //    sz = wUSB_EPnMaxPacketSize[ep];
    //}

    off = 0;

    while (off != sz) {
        chunk = 4;
        if (off + chunk > sz)
            chunk = sz - off;

        //dest, src, size
        memcpy(&data, buf, chunk);

        if(intr)
        {
            SN32_USB->RWADDR = off + ep_offset;
            SN32_USB->RWDATA = data;
            SN32_USB->RWSTATUS = 0x01;
            while (SN32_USB->RWSTATUS & 0x01);
        }
        else
        {
            SN32_USB->RWADDR2 = off + ep_offset;
            SN32_USB->RWDATA2 = data;
            SN32_USB->RWSTATUS2 = 0x01;
            while (SN32_USB->RWSTATUS2 & 0x01);
        }

        off += chunk;
        buf += chunk;
    }
}

/**
 * @brief   USB shared ISR.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void usb_lld_serve_interrupt(USBDriver *usbp)
{
    uint32_t iwIntFlag;
    size_t n;

    /* Get Interrupt Status and clear immediately. */
    iwIntFlag = SN32_USB->INSTS;
    /* Keep only PRESETUP & ERR_SETUP flags. */
    SN32_USB->INSTSC = ~(mskEP0_PRESETUP | mskERR_SETUP);

    if ((iwIntFlag == 0) | (iwIntFlag & mskBUS_WAKEUP))
    {
        //@20160902 add for EMC protection
        SN32_USB->CFG |= (mskESD_EN|mskPHY_EN);
        __USB_CLRINSTS(mskBUS_WAKEUP);
        return;
    }

    /////////////////////////////////////////////////
    /* Device Status Interrupt (BusReset, Suspend) */
    /////////////////////////////////////////////////
    if (iwIntFlag & (mskBUS_RESET | mskBUS_SUSPEND | mskBUS_RESUME))
    {
        if (iwIntFlag & mskBUS_RESET)
        {
            /* BusReset */
            SN32_USB->CFG |= (mskESD_EN|mskPHY_EN);
            __USB_CLRINSTS(mskBUS_RESET);
            _usb_reset(usbp);
        }
        else if (iwIntFlag & mskBUS_SUSPEND)
        {
            /* Suspend */
            SN32_USB->CFG &= ~(mskESD_EN|mskPHY_EN);
            __USB_CLRINSTS(mskBUS_SUSPEND);
            _usb_suspend(usbp);
        }
        else if(iwIntFlag & mskBUS_RESUME)
        {
            /* Resume */
            SN32_USB->CFG |= (mskESD_EN|mskPHY_EN);
            __USB_CLRINSTS(mskBUS_RESUME);
            _usb_wakeup(usbp);
        }
    }
    /////////////////////////////////////////////////
    /* Device Status Interrupt (SETUP, IN, OUT)      */
    /////////////////////////////////////////////////
    else if (iwIntFlag & mskERR_SETUP) {
        __USB_CLRINSTS(mskERR_SETUP);
        usb_lld_stall_in(usbp, 0);
    }
    else if (iwIntFlag & (mskEP0_SETUP|mskEP0_IN|mskEP0_OUT|mskEP0_IN_STALL|mskEP0_OUT_STALL))
    {
        const USBEndpointConfig *epcp = usbp->epc[0];

        if (iwIntFlag & mskEP0_SETUP)
        {
            /* SETUP */
            /* Clear receiving in the chibios state machine */
            (usbp)->receiving &= ~1;
            __USB_CLRINSTS(mskEP0_PRESETUP);
            /* Call SETUP function (ChibiOS core), which prepares
             * for send or receive and releases the buffer
             */
            _usb_isr_invoke_setup_cb(usbp, 0);
            __USB_CLRINSTS(mskEP0_SETUP);
        }
        else if (iwIntFlag & mskEP0_IN)
        {
            USBInEndpointState *isp = epcp->in_state;

            /* IN */
            /* Special case for SetAddress for EP0*/
            if((((uint16_t)usbp->setup[0]<<8)|usbp->setup[1]) == 0x0500)
            {
              usbp->address = usbp->setup[2];
              usb_lld_set_address(usbp);
              _usb_isr_invoke_event_cb(usbp, USB_EVENT_ADDRESS);
              usbp->state = USB_SELECTED;
              usb_lld_stall_in(usbp, 0);
            }

            isp->txcnt += isp->txlast;
            n = isp->txsize - isp->txcnt;
            if (n > 0) {
                /* Transfer not completed, there are more packets to send.*/
                if (n > epcp->in_maxsize)
                    n = epcp->in_maxsize;

                /* Writes the packet from the defined buffer.*/
                isp->txbuf += isp->txlast;
                isp->txlast = n;

                sn32_usb_write_fifo(0, isp->txbuf, n, true);

                EPCTL_SET_STAT_ACK(USB_EP0, n);
            }
            else
            {
                //EPCTL_SET_STAT_NAK(USB_EP0); //not needed

                _usb_isr_invoke_in_cb(usbp, 0);
            }
            __USB_CLRINSTS(mskEP0_IN);

        }
        else if (iwIntFlag & mskEP0_OUT)
        {
            USBOutEndpointState *osp = epcp->out_state;
            /* OUT */

            n = SN32_USB->EPCTL[0] & mskEPn_CNT;
            if (n > epcp->out_maxsize)
                n = epcp->out_maxsize;

            //Just being paranoid here. keep here while debugging EP handling issue
            //TODO: clean it up when packets are properly handled
            if (epcp->out_state->rxsize >= n) {
                //we are ok to copy n bytes to buf
                sn32_usb_read_fifo(USB_EP0, osp->rxbuf, n, true);
                epcp->out_state->rxsize -= n;
            }
            else if (epcp->out_state->rxsize > 0) {
                //we dont have enough buffer to receive n bytes
                //copy only size availabe on buffer
                n = epcp->out_state->rxsize;
                sn32_usb_read_fifo(USB_EP0, osp->rxbuf, n, true);
                epcp->out_state->rxsize -= n;
            }
            else {
                //well buffer is 0 size. strange. do nothing.
                n = 0;
            }

            epcp->out_state->rxbuf += n;
            epcp->out_state->rxcnt += n;
            if (epcp->out_state->rxpkts > 0) {
                epcp->out_state->rxpkts -= 1;
            }

            if (epcp->out_state->rxpkts == 0)
            {
                //done with transfer
                //EPCTL_SET_STAT_NAK(USB_EP0); //useless mcu resets it anyways
                _usb_isr_invoke_out_cb(usbp, 0);
            }
            else {
                //more to receive
                EPCTL_SET_STAT_ACK(USB_EP0, 0);
            }
            __USB_CLRINSTS(mskEP0_OUT);

        }
        else if (iwIntFlag & (mskEP0_IN_STALL))
        {
            /* EP0_IN_STALL */
            usb_lld_stall_in(usbp, 0);
            SN32_USB->INSTSC = (mskEP0_IN_STALL);
        }
        else if (iwIntFlag & (mskEP0_OUT_STALL))
        {
            /* EP0_OUT_STALL */
            usb_lld_stall_out(usbp, 0);
            SN32_USB->INSTSC = (mskEP0_OUT_STALL);
        }
    }
    /////////////////////////////////////////////////
    /* Device Status Interrupt (EPnACK)            */
    /////////////////////////////////////////////////
    else if (iwIntFlag & (mskEP4_ACK|mskEP3_ACK|mskEP2_ACK|mskEP1_ACK))
    {
        usbep_t ep;
        // Determine the interrupting endpoint, direction, and clear the interrupt flag
        for(int i=1; i <= USB_MAX_ENDPOINTS; i++) {
            if (iwIntFlag & mskEPn_ACK(i)){
                ep = i;
            }
        }
        handleACK(usbp, ep);
        __USB_CLRINSTS(mskEPn_ACK(ep));
    }
    else if (iwIntFlag & (mskEP4_NAK|mskEP3_NAK|mskEP2_NAK|mskEP1_NAK))
    {
        usbep_t ep;
        // Determine the interrupting endpoint, direction, and clear the interrupt flag
        for(int i=1; i <= USB_MAX_ENDPOINTS; i++) {
            if (iwIntFlag & mskEPn_NAK(i)){
                ep = i;
            }
        }
        handleNAK(usbp, ep);
        __USB_CLRINSTS(mskEPn_NAK(ep));
    }

    /////////////////////////////////////////////////
    /* Device Status Interrupt (SOF)               */
    /////////////////////////////////////////////////
    if ((iwIntFlag & mskUSB_SOF) && (SN32_USB->INTEN & mskUSB_SOF_IE))
    {
        /* SOF */
        _usb_isr_invoke_sof_cb(usbp);
        __USB_CLRINSTS(mskUSB_SOF);
    }
}

void handleACK(USBDriver* usbp, usbep_t ep) {
    uint8_t out = 0;
    uint8_t cnt = 0;
    size_t n;

    if(ep > 0 && ep <= USB_MAX_ENDPOINTS) {
        out = ( SN32_USB->CFG & mskEPn_DIR(ep) ) == mskEPn_DIR(ep);
        cnt = SN32_USB->EPCTL[ep] & mskEPn_CNT;
    }
    else {
        return;
    }
    nakcnt[ep] = 0;

    // Get the endpoint config and state
    const USBEndpointConfig *epcp = usbp->epc[ep];
    USBInEndpointState *isp = epcp->in_state;
    USBOutEndpointState *osp = epcp->out_state;

    // Process based on endpoint direction
    if(out)
    {
        // Read size of received data
        n = cnt;

        if (n > epcp->out_maxsize)
            n = epcp->out_maxsize;

        //state is NAK already
        //Just being paranoid here. keep here while debugging EP handling issue
        //TODO: clean it up when packets are properly handled
        if (epcp->out_state->rxsize >= n) {
            //we are ok to copy n bytes to buf
            sn32_usb_read_fifo(ep, osp->rxbuf, n, true);
            epcp->out_state->rxsize -= n;
        }
        else if (epcp->out_state->rxsize > 0) {
            //we dont have enough buffer to receive n bytes
            //copy only size availabe on buffer
            n = epcp->out_state->rxsize;
            sn32_usb_read_fifo(ep, osp->rxbuf, n, true);
            epcp->out_state->rxsize -= n;
        }
        else {
            //well buffer is 0 size. strange. do nothing.
            n = 0;
        }
        osp->rxbuf += n;

        epcp->out_state->rxcnt += n;
        if (epcp->out_state->rxpkts > 0) {
            epcp->out_state->rxpkts -= 1;
        }

        if (n < epcp->out_maxsize || epcp->out_state->rxpkts == 0)
        {
            _usb_isr_invoke_out_cb(usbp, ep);
        }
        else
        {
            //not done. keep on receiving
            EPCTL_SET_STAT_ACK(ep, 0);
        }
    }
    else
    {
        // Process transmit queue
        isp->txcnt += isp->txlast;
        n = isp->txsize - isp->txcnt;

        if (n > 0)
        {
            /* Transfer not completed, there are more packets to send.*/
            if (n > epcp->in_maxsize)
            {
                n = epcp->in_maxsize;
            }

            /* Writes the packet from the defined buffer.*/
            isp->txbuf += isp->txlast;
            isp->txlast = n;

            sn32_usb_write_fifo(ep, isp->txbuf, n, true);

            EPCTL_SET_STAT_ACK(ep, n);
        }
        else
        {
            //EPCTL_SET_STAT_NAK(ep); //not needed here it is autoreset to NAK already

            _usb_isr_invoke_in_cb(usbp, ep);
        }
    }
}

void handleNAK(USBDriver *usbp, usbep_t ep) {
    uint8_t out = 0;

    if(ep > 0 && ep <= USB_MAX_ENDPOINTS) {
        out = ( SN32_USB->CFG & mskEPn_DIR(ep) ) == mskEPn_DIR(ep);
    }
    else {
        return;
    }


    if(out)
    {
        // no ack required here
    }
    else
    {
        // This is not a retransmission, retransmission is transparent and happens on phy layer
        // NAK happens when host polls IN EP and device has nothing to send
        // It has been observed that sometimes USB phy doesn't generate ACK (unknown why)
        // (count ACK interrupts didn't match count of usb_lld_start_in calls per EP)
        // However while USB transmitting and qmk thread wants to send another packet qmk goes to
        // infinite sleep, expecting that successfull USB transmission will wake it up
        // If USB transmission never completes (no ACK) then qmk never wakes up and keyboard locks up
        // To prevent this every NAK (1ms or 8ms depending on host poll interval) was calling
        // callbacks and wake up function to wake up qmk thread, however packet was not delivered to host
        // (for unknown reason) and thus we have seen:
        // 1) stuck keypresses when usb packets to press key delivered but key release packet lost
        // 2) untyped key when usb packet to press key was lost but key release packet delivered
        // Because callback was called every NAK some qmk features didnt work such as CONSOLE
        // since callback might release buffers and endup in deadlock via disabled interrupts
        // callback for keyboard is empty thus its repated calling is harmless
        #if defined(SN32_USB_ORIGINAL_NAK_HANDLING)
        _usb_isr_invoke_in_cb(usbp, ep);
        #else
        //To fake missing ACK we can send 0 sized packet
        //however (again for unknown reason) packets now being delivered to host as well!
        //- value 2 has been selected to allow at least 2 NAK delivery (2ms or 16ms depending on
        //host polling interval) between moment qmk called start_in and moment USB phy actually
        //started transmission
        //- value 10 was selected arbitrary.
        //- values 3-10 we are delivering 0 sized packet trying to get at least one ack
        if (nakcnt[ep] > 0) {
            //qmk called start_in
            if (nakcnt[ep] > 10) {
                //11-....
                //consider packet undeliverable but ack it to the qmk
                nakcnt[ep] = 0;
                _usb_isr_invoke_in_cb(usbp, ep);
            }
            else if (nakcnt[ep] > 2) {
                //3-10
                nakcnt[ep]++;
                EPCTL_SET_STAT_ACK(ep, 0);
            }
            else {
                //1-2
                //give it sometime to deliver the packet
                nakcnt[ep]++;
            }
        }
        #endif
    }
}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

/**
 * @brief   SN32 USB Interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_USB_HANDLER) {

    OSAL_IRQ_PROLOGUE();
    usb_lld_serve_interrupt(&USBD1);
    OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {
#if SN32_USB_USE_USB1 == TRUE
    /* Driver initialization.*/
    usbObjectInit(&USBD1);
#endif
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {
  if (usbp->state == USB_STOP) {
    /* Clock activation.*/
#if SN32_USB_USE_USB1
    if (&USBD1 == usbp) {
      /* USB clock enabled.*/
      sys1EnableUSB();
      /* Initialize USB EP1~EP4 RAM Start address base on 64-bytes. */
      USB_SET_BUFFER_OFST(1, EP1_BUFFER_OFFSET_VALUE);
      USB_SET_BUFFER_OFST(2, EP2_BUFFER_OFFSET_VALUE);
      USB_SET_BUFFER_OFST(3, EP3_BUFFER_OFFSET_VALUE);
      USB_SET_BUFFER_OFST(4, EP4_BUFFER_OFFSET_VALUE);
#if (USB_ENDPOINTS_NUMBER > 4)
      USB_SET_BUFFER_OFST(5, EP5_BUFFER_OFFSET_VALUE);
      USB_SET_BUFFER_OFST(6, EP6_BUFFER_OFFSET_VALUE);
#endif /* (USB_ENDPOINTS_NUMBER > 4) */
      /* Powers up the transceiver while holding the USB in reset state.*/
      SN32_USB->SGCTL = mskBUS_J_STATE;
      SN32_USB->CFG = (mskVREG33_EN|mskPHY_EN|mskDPPU_EN|mskSIE_EN|mskESD_EN);
      /* Set up hardware configuration.*/
      SN32_USB->PHYPRM = 0x80000000;
      SN32_USB->PHYPRM2 = 0x00004004;
      /* Enable the USB Bus Interrupts.*/
      SN32_USB->INTEN = mskBUS_IE;

      nvicEnableVector(SN32_USB_NUMBER, SN32_USB_IRQ_PRIORITY);
      /* Releases the reset state.*/
      SN32_USB->SGCTL &= ~mskBUS_DRVEN;
    }
#endif
    /* Reset procedure enforced on driver start.*/
    usb_lld_reset(usbp);
  }
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {
  /* If in ready state then disables the USB clock.*/
  if (usbp->state != USB_STOP) {
#if SN32_USB_USE_USB1 == TRUE
    /* Disables the peripheral.*/
    if (&USBD1 == usbp) {
        nvicDisableVector(SN32_USB_NUMBER);
        sys1DisableUSB();
    }
#endif
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {
    /* Post reset initialization.*/
    SN32_USB->INSTSC = (0xFFFFFFFF);

    /* Set the address to zero during enumeration.*/
    usbp->address = 0;
    SN32_USB->ADDR  = 0;

    /* EP0 initialization.*/
    usbp->epc[0] = &ep0config;
    usb_lld_init_endpoint(usbp, 0);

    /* Enable other interrupts.*/
    SN32_USB->INTEN |= (mskUSB_IE|mskEPnACK_EN|mskBUSWK_IE|mskUSB_SOF_IE);
    SN32_USB->INTEN |= (mskEP1_NAK_EN|mskEP2_NAK_EN|mskEP3_NAK_EN|mskEP4_NAK_EN);
#if (USB_ENDPOINTS_NUMBER > 4)
    SN32_USB->INTEN |= (mskEP5_NAK_EN|mskEP6_NAK_EN);
#endif /* (USB_ENDPOINTS_NUMBER > 4) */

}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {

    SN32_USB->ADDR = usbp->address & 0x7F;
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
    const USBEndpointConfig *epcp = usbp->epc[ep];

    /* Make sure direction flags are not set.*/
    if(ep > 0 && ep <= USB_MAX_ENDPOINTS) {
        SN32_USB->CFG &= ~mskEPn_DIR(ep);
    }

    /* Set the endpoint type. */
    switch (epcp->ep_mode & USB_EP_MODE_TYPE) {
        case USB_EP_MODE_TYPE_ISOC:
            break;
        case USB_EP_MODE_TYPE_BULK:
            break;
        case USB_EP_MODE_TYPE_INTR:
            break;
        default:
            break;
    }

    /* IN endpoint? */
    if (epcp->in_state != NULL) {
        // Set endpoint direction flag in USB configuration register
        if(ep ==0) {
            usb_lld_stall_in(usbp, 0);
        } else if(ep <= USB_MAX_ENDPOINTS) {
            SN32_USB->CFG &= ~mskEPn_DIR(ep);
        }
    }

    /* OUT endpoint? */
    if (epcp->out_state != NULL) {
        // Set endpoint direction flag in USB configuration register
        if(ep ==0) {
            usb_lld_stall_out(usbp, 0);
        } else if(ep <= USB_MAX_ENDPOINTS) {
            SN32_USB->CFG |= mskEPn_DIR(ep);
        }
    }

    /* Enable endpoint. */
    if(ep <= USB_MAX_ENDPOINTS) {
        SN32_USB->EPCTL[ep] |= mskEPn_ENDP_EN;
    }
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {

    /* Disabling all endpoints.*/
    for(usbep_t ep=1; ep <= USB_MAX_ENDPOINTS; ep++) {
        SN32_USB->EPCTL[ep] = 0;
        SN32_USB->CFG &= ~mskEPn_DIR(ep);
    }
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
    if (ep > USB_MAX_ENDPOINTS)
        return EP_STATUS_DISABLED;
    if ((SN32_USB->EPCTL[ep] & mskEPn_ENDP_EN) != mskEPn_ENDP_EN)
        return EP_STATUS_DISABLED;
    if ((SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE) == mskEPn_ENDP_STATE_STALL)
        return EP_STATUS_STALLED;
    return EP_STATUS_ACTIVE;
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
    if (ep > USB_MAX_ENDPOINTS)
        return EP_STATUS_DISABLED;
    if ((SN32_USB->EPCTL[ep] & mskEPn_ENDP_EN) != mskEPn_ENDP_EN)
        return EP_STATUS_DISABLED;
    if ((SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE) == mskEPn_ENDP_STATE_STALL)
        return EP_STATUS_STALLED;
    return EP_STATUS_ACTIVE;
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */

void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {

    sn32_usb_read_fifo(ep, buf, 8, false);
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {

    USBOutEndpointState *osp = usbp->epc[ep]->out_state;

    /* Transfer initialization.*/
    if (osp->rxsize == 0)         /* Special case for zero sized packets.*/
        osp->rxpkts = 1;
    else
        osp->rxpkts = (uint16_t)((osp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                                    usbp->epc[ep]->out_maxsize);
    osp->rxcnt = 0;//haven't received anything yet
    EPCTL_SET_STAT_ACK(ep, 0);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep)
{
    size_t n;
    USBInEndpointState *isp = usbp->epc[ep]->in_state;

    /* Transfer initialization.*/
    //who handles 0 packet ack on setup?
    n = isp->txsize;

    if((n >= 0) || (ep == 0))
    {
        if (n > (size_t)usbp->epc[ep]->in_maxsize)
            n = (size_t)usbp->epc[ep]->in_maxsize;

        isp->txlast = n;

        sn32_usb_write_fifo(ep, isp->txbuf, n, false);

        nakcnt[ep] = 1;
        EPCTL_SET_STAT_ACK(ep, n);
    }
    else
    {
        _usb_isr_invoke_in_cb(usbp, ep);
    }

}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
    if (ep == 0 ) {
        if (SN32_USB->INSTS & mskEP0_PRESETUP) {
            return;
        }
        if (SN32_USB->INSTS & mskEP0_OUT_STALL) {
            SN32_USB->EPCTL[ep] |= mskEP0_OUT_STALL_EN;
            return;
        }
    }
    EPCTL_SET_STAT_STALL(ep);
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
    if (ep == 0 ) {
        if (SN32_USB->INSTS & mskEP0_PRESETUP) {
            return;
        }
        if (SN32_USB->INSTS & mskEP0_IN_STALL) {
            SN32_USB->EPCTL[ep] |= mskEP0_IN_STALL_EN;
            return;
        }
    }
    EPCTL_SET_STAT_STALL(ep);
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
    /* Makes sure to not put to NAK an endpoint that is already
       transferring.*/
    if (!(SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE_NAK))
        EPCTL_SET_STAT_NAK(ep);
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
    /* Makes sure to not put to NAK an endpoint that is already
       transferring.*/
    if (!(SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE_NAK))
        EPCTL_SET_STAT_NAK(ep);
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
