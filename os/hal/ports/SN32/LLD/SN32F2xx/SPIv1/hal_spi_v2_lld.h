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
 * @file    hal_spi_v2_lld.h
 * @brief   SN32F2xx SPI (v2) subsystem low level driver header.
 *
 * @addtogroup SPI_V2
 * @{
 */

#ifndef HAL_SPI_V2_LLD_H
#define HAL_SPI_V2_LLD_H

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Circular mode support flag.
 */
#define SPI_SUPPORTS_CIRCULAR           FALSE

/**
 * @brief   Slave mode support flag.
 */
#define SPI_SUPPORTS_SLAVE_MODE         TRUE


// FIFO Threshold Values: 0-7
#define SPI_CTRL0_RXFIFOTH(threshold)   ((threshold) << 15)
#define SPI_CTRL0_RXFIFOTH_0            SPI_CTRL0_RXFIFOTH(0)
#define SPI_CTRL0_RXFIFOTH_1            SPI_CTRL0_RXFIFOTH(1)
#define SPI_CTRL0_RXFIFOTH_2            SPI_CTRL0_RXFIFOTH(2)
#define SPI_CTRL0_RXFIFOTH_3            SPI_CTRL0_RXFIFOTH(3)
#define SPI_CTRL0_RXFIFOTH_4            SPI_CTRL0_RXFIFOTH(4)
#define SPI_CTRL0_RXFIFOTH_5            SPI_CTRL0_RXFIFOTH(5)
#define SPI_CTRL0_RXFIFOTH_6            SPI_CTRL0_RXFIFOTH(6)
#define SPI_CTRL0_RXFIFOTH_7            SPI_CTRL0_RXFIFOTH(7)

#define SPI_CTRL0_TXFIFOTH(threshold)   ((threshold) << 12)
#define SPI_CTRL0_TXFIFOTH_0            SPI_CTRL0_TXFIFOTH(0)
#define SPI_CTRL0_TXFIFOTH_1            SPI_CTRL0_TXFIFOTH(1)
#define SPI_CTRL0_TXFIFOTH_2            SPI_CTRL0_TXFIFOTH(2)
#define SPI_CTRL0_TXFIFOTH_3            SPI_CTRL0_TXFIFOTH(3)
#define SPI_CTRL0_TXFIFOTH_4            SPI_CTRL0_TXFIFOTH(4)
#define SPI_CTRL0_TXFIFOTH_5            SPI_CTRL0_TXFIFOTH(5)
#define SPI_CTRL0_TXFIFOTH_6            SPI_CTRL0_TXFIFOTH(6)
#define SPI_CTRL0_TXFIFOTH_7            SPI_CTRL0_TXFIFOTH(7)

// Data length Values 3-16
#define SPI_CTRL0_DL(length)            ((length - 1) << 8)
#define SPI_CTRL0_DL_3                  SPI_CTRL0_DL(3)
#define SPI_CTRL0_DL_4                  SPI_CTRL0_DL(4)
#define SPI_CTRL0_DL_5                  SPI_CTRL0_DL(5)
#define SPI_CTRL0_DL_6                  SPI_CTRL0_DL(6)
#define SPI_CTRL0_DL_7                  SPI_CTRL0_DL(7)
#define SPI_CTRL0_DL_8                  SPI_CTRL0_DL(8)
#define SPI_CTRL0_DL_9                  SPI_CTRL0_DL(9)
#define SPI_CTRL0_DL_10                 SPI_CTRL0_DL(10)
#define SPI_CTRL0_DL_11                 SPI_CTRL0_DL(11)
#define SPI_CTRL0_DL_12                 SPI_CTRL0_DL(12)
#define SPI_CTRL0_DL_13                 SPI_CTRL0_DL(13)
#define SPI_CTRL0_DL_14                 SPI_CTRL0_DL(14)
#define SPI_CTRL0_DL_15                 SPI_CTRL0_DL(15)
#define SPI_CTRL0_DL_16                 SPI_CTRL0_DL(16)

#define SPI_CTRL0_SELDIS_EN             (0 << 18)
#define SPI_CTRL0_SELDIS_DIS            (1 << 18)

#define SPI_CTRL1_CPHA_FALLING_EDGE     (0 << 2)
#define SPI_CTRL1_CPHA_RISING_EDGE      (1 << 2)

#define SPI_CTRL1_CPOL_LOW_LEVEL        (0 << 1)
#define SPI_CTRL1_CPOL_HIGH_LEVEL       (1 << 1)
#define SPI_CTRL1_MLSB_MSB              (0 << 1)
#define SPI_CTRL1_MLSB_LSB              (1 << 1)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SN32F2xx configuration options
 * @{
 */
/**
 * @brief   SPI0 driver enable switch.
 * @details If set to @p TRUE the support for SPI0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SN32_SPI_USE_SPI0) || defined(__DOXYGEN__)
#define SN32_SPI_USE_SPI0               FALSE
#endif
/** @} */

/**
 * @brief   SPI1 driver enable switch.
 * @details If set to @p TRUE the support for SPI0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SN32_SPI_USE_SPI1) || defined(__DOXYGEN__)
#define SN32_SPI_USE_SPI1               FALSE
#endif
/** @} */

/**
 * @brief   SPI interrupt priority level setting.
 */
#if !defined(SN32_SPI0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SN32_SPI0_IRQ_PRIORITY          3
#endif

#if !defined(SN32_SPI1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SN32_SPI1_IRQ_PRIORITY          3
#endif
/** @} */


/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SN32_SPI_USE_SPI0 && !SN32_HAS_SPI0
#error "SPI0 not present in the selected device"
#endif

#if SN32_SPI_USE_SPI1 && !SN32_HAS_SPI1
#error "SPI1 not present in the selected device"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define SPI_FIFO_RESET(spip) (spip->spi->CTRL0_b.FRESET = 0b11);

/**
 * @brief   Low level fields of the SPI driver structure.
 */
#define spi_lld_driver_fields                                               \
  /* Pointer to SPI Register.*/                                             \
  SN_SPI_Type                           *spi;                               \
  const uint8_t                         *txbuf;                             \
  uint8_t                               *rxbuf;                             \
  size_t                                count;                              \
  size_t                                idx;

/**
 * @brief   Low level fields of the SPI configuration structure.
 */
#define spi_lld_config_fields                                               \
  uint32_t                              ctrl0;                              \
  uint8_t                               ctrl1;                              \
  uint8_t                               clkdiv;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (SN32_SPI_USE_SPI0 == TRUE) && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
#endif

#if (SN32_SPI_USE_SPI1 == TRUE) && !defined(__DOXYGEN__)
extern SPIDriver SPID2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  msg_t spi_lld_start(SPIDriver *spip);
  void spi_lld_stop(SPIDriver *spip);
#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
  void spi_lld_select(SPIDriver *spip);
  void spi_lld_unselect(SPIDriver *spip);
#endif
  msg_t spi_lld_ignore(SPIDriver *spip, size_t n);
  msg_t spi_lld_exchange(SPIDriver *spip, size_t n,
                         const void *txbuf, void *rxbuf);
  msg_t spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
  msg_t spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
  msg_t spi_lld_stop_transfer(SPIDriver *spip, size_t *sizep);
  uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* HAL_SPI_V2_LLD_H */

/** @} */
