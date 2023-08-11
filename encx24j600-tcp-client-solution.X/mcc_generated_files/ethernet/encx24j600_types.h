/**
 * ENCx24J600 Ethernet Controller Header File
 *
 * @file encx24j600_types.h
 *
 * @defgroup encx24j600 ENCX24J600
 *
 * @brief This file provides register definitions for the ENCx24J600 controller.
 *
 * @version ENCx24J600 Driver Version 6.0.0
 */
/*
© [2023] Microchip Technology Inc. and its subsidiaries.

	Subject to your compliance with these terms, you may use Microchip
	software and any derivatives exclusively with Microchip products.
	You are responsible for complying with 3rd party license terms
	applicable to your use of 3rd party software (including open source
	software) that may accompany Microchip software. SOFTWARE IS "AS IS".
	NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
	SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, 
	MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
	WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
	INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
	KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
	MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
	FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S 
	TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
	EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
	THIS SOFTWARE.
*/

#ifndef ENCX24J600_TYPES_H
#define ENCX24J600_TYPES_H

// Macros

// Controller Options

// ENC424J600 config

#define RAMSIZE_ETH 0x6000ul

/**
 * @ingroup encx24j600
 * @def TXSTART
 * @brief Transmit buffer start.
 */
#define TXSTART 0x0000ul

/**
 * @ingroup encx24j600
 * @def TXEND
 * @brief Transmit buffer end.
 */
#define TXEND 0x15FFul

/**
 * @ingroup encx24j600
 * @def RXSTART
 * @brief Receive buffer start (must be an even memory address).
 */
#define RXSTART 0x1600ul

/**
 * @ingroup encx24j600
 * @def RXEND
 * @brief Receive buffer end.
 */
#define RXEND (RAMSIZE_ETH - 2)

#define ETH_HEADER_SIZE 14
#define IP_PROTOCOL_POS 23

// ESTAT helpers

#define ESTAT_PHYLINK 0x0100
#define ESTAT_RSTDONE 0x0800
#define ESTAT_PHYDPX 0x0400
#define ESTAT_PHYRDY 0x0200
#define ESTAT_PKTCNT 0x00FF
#define ESTAT_RXBUSY 0x2000
#define ESTAT_CLKRDY 0x1000

// MACON2 helpers

#define MACON2_FULDPX_ON 0x0001
#define MACON2_FULDPX_OFF 0xFFFE

// Some EIR helpers

#define EIR_CRYPTEN 0x8000
#define EIR_LINKIF 0x0800
#define EIR_TXIF 0x0008
#define EIR_PKTIF 0x0040
#define EIR_RXABTIF 0x0002
#define EIR_PCFULIF 0x0001

// Some ECON1 helpers

#define ECON1_RXEN 0x0001
#define ECON1_TXRTS 0x0002
#define ECON1_DMANOCS 0x0004
#define ECON1_DMACSSD 0x0008
#define ECON1_DMACPY 0x0010
#define ECON1_DMAST 0x0020
#define ECON1_PKTDEC 0x0100
#define ECON1_AESST 0x0800
#define ECON1_HASHLST 0x1000
#define ECON1_HASHOP 0x2000
#define ECON1_HASHEN 0x4000
#define ECON1_MODEXST 0x8000

// PHCON1 helpers

/**
 * @ingroup encx24j600
 * @def PHCON1_PSLEEP
 * @brief Bit mask for PHY Sleep Enable bit. Physical Layer (PHY) power is down.
 */
#define PHCON1_PSLEEP 0x0800

/**
 * @ingroup encx24j600
 * @def PHCON1_PWAKE
 * @brief Bit mask for PHY Wake-Up as a normal operation
 */
#define PHCON1_PWAKE 0xF7FF

// ECON2 helpers

#define ECON2_ETHEN 0x8000
#define ECON2_AUTOFC 0x0080
#define ECON2_STRCH 0x4000
#define ECON2_ETHRST 0x0010
#define ECON2_RXRST 0x0020

/**
 * @ingroup encx24j600
 * @enum encX24J600_phy_registers_t
 * @brief ENCX24J600 PHY registers.
 */
typedef enum
{
    PHCON1 = 0x00,  /**<PHY Control Register 1*/
    PHSTAT1 = 0x01, /**<Physical Layer Status Register 1*/
    PHANA = 0x04,   /**<PHY Auto-Negotiation Advertisement Register*/
    PHANLPA = 0x05, /**<PHY Auto-Negotiation Link Partner Ability Register*/
    PHANE = 0x06,   /**<PHY Auto-Negotiation Expansion Register*/
    PHCON2 = 0x11,  /**<PHY Control Register 2*/
    PHSTAT2 = 0x1B, /**<Physical Layer Status Register 2*/
    PHSTAT3 = 0x1F  /**<Physical Layer Status Register 3*/
} encX24J600_phy_registers_t;

#endif /* ENCX24J600_TYPES_H */