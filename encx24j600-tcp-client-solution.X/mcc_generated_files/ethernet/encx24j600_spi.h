/**
 * ENCx24J600 spi Header File
 *
 * @file encx24j600_spi.h
 *
 * @ingroup encx24j600
 *
 * @brief This header file provides the serial interface API for the ENCx24J600 devices.
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

#ifndef ENCX24J600_SPI_H
#define ENCX24J600_SPI_H

// Include Headers

#include "encx24j600_types.h"
#include "../system/system.h"
#include "../spi/spi_interface.h"

// Macros

#define byteSwap16(a) ((((uint16_t)a & (uint16_t)0xFF00) >> 8) | (((uint16_t)a & (uint16_t)0x00FF) << 8))
#define byteReverse32(a) ((((uint32_t)a & (uint32_t)0xff000000) >> 24) | \
                          (((uint32_t)a & (uint32_t)0x00ff0000) >> 8)  | \
                          (((uint32_t)a & (uint32_t)0x0000ff00) << 8)  | \
                          (((uint32_t)a & (uint32_t)0x000000ff) << 24))

#define byteReverse24(a) (((((uint32_t)a & (uint32_t)0x00FF00) >> 8) | (((uint32_t)a & (uint32_t)0x0000FF) << 8)) << 8 | (uint32_t)a >> 0x10)

// host to network & network to host macros
#ifndef htons
#define htons(a) byteSwap16(a)
#endif
#ifndef ntohs
#define ntohs(a) byteSwap16(a)
#endif
#ifndef htonl
#define htonl(a) byteReverse32(a)
#endif
#ifndef ntohl
#define ntohl(a) byteReverse32(a)
#endif

#define convert_hton24(a) byteReverse24(a)

/**
 * @ingroup encx24j600
 * @def ETH_NCS_HIGH()
 * @brief Sets Ethernet Chip Select to high.
 */
#define ETH_NCS_HIGH() do{ETH_CS_SetHigh();} while(0)  //Use the Ethernet Chip select as per hardware specification.

/**
 * @ingroup encx24j600
 * @def ETH_NCS_LOW()
 * @brief Sets Ethernet Chip Select to low.
 */
#define ETH_NCS_LOW() do{ETH_CS_SetLow();} while(0)  //Use the Ethernet Chip select as per hardware specification.

/**
 * @ingroup encx24j600
 * @def ETH_READ8()
 * @brief Reads the Serial Peripheral Interface (SPI) byte.
 */
#define ETH_READ8() SPI1_ByteExchange(0)

/**
 * @ingroup encx24j600
 * @def ETH_WRITE8(a)
 * @brief Packet write in progress, not ready for transmit.
 */
#define ETH_WRITE8(a) SPI1_ByteExchange(a)

#define SFR_BANK0 0x00
#define SFR_BANK1 0x20
#define SFR_BANK2 0x40
#define SFR_BANK3 0x60
#define UNBANKED 0X80
#define SFR_COMMON 0xE0
#define BANK_MASK 0xE0
#define SFR_MASK 0x1F

// Enums

/**
 * @ingroup encx24j600
 * @enum sfr_bank_t
 * @brief Special Function Register (SFR) Banks.
 */
typedef enum
{
    sfr_bank0 = SFR_BANK0,  /**<Bank 0*/
    sfr_bank1 = SFR_BANK1,  /**<Bank 1*/
    sfr_bank2 = SFR_BANK2,  /**<Bank 2*/
    sfr_bank3 = SFR_BANK3,  /**<Bank 3*/
    sfr_common = SFR_COMMON /**<Common Set of Registers (last five locations of each bank)*/
} sfr_bank_t;

/**
 * @ingroup encx24j600
 * @enum encX24J600_registers_t
 * @brief SFR addresses for the ENCx24J600 SPI mode.
 */
typedef enum
{
    XJ600_ETXSTL = (SFR_BANK0 | 0x00),   /**<TX Start Address*/
    XJ600_ETXLENL = (SFR_BANK0 | 0x02),  /**<TX Length*/
    XJ600_ERXSTL = (SFR_BANK0 | 0x04),   /**<RX Buffer Start Address*/
    XJ600_ERXTAILL = (SFR_BANK0 | 0x06), /**<RX Tail Pointer*/
    XJ600_ERXHEADL = (SFR_BANK0 | 0x08), /**<RX Head Pointer*/
    XJ600_EDMASTL = (SFR_BANK0 | 0x0A),  /**<DMA Start Address*/
    XJ600_EDMALENL = (SFR_BANK0 | 0x0C), /**<DMA Length*/
    XJ600_EDMADSTL = (SFR_BANK0 | 0x0E), /**<DMA Destination Address*/
    XJ600_EDMACSL = (SFR_BANK0 | 0x10),  /**<DMA Checksum*/
    XJ600_ETXSTATL = (SFR_BANK0 | 0x12), /**<TX Status*/
    XJ600_ETXWIREL = (SFR_BANK0 | 0x14), /**<Transmit Byte Count on Wire (including collision bytes)*/

    XJ600_EHT1L = (SFR_BANK1 | 0x00),    /**<Hash Table Filter 1*/
    XJ600_EHT2L = (SFR_BANK1 | 0x02),    /**<Hash Table Filter 2*/
    XJ600_EHT3L = (SFR_BANK1 | 0x04),    /**<Hash Table Filter 3*/
    XJ600_EHT4L = (SFR_BANK1 | 0x06),    /**<Hash Table Filter 4*/
    XJ600_EPMM1L = (SFR_BANK1 | 0x08),   /**<Pattern Match Filter Mask 1*/
    XJ600_EPMM2L = (SFR_BANK1 | 0x0A),   /**<Pattern Match Filter Mask 2*/
    XJ600_EPMM3L = (SFR_BANK1 | 0x0C),   /**<Pattern Match Filter Mask 3*/
    XJ600_EPMM4L = (SFR_BANK1 | 0x0E),   /**<Pattern Match Filter Mask 4*/
    XJ600_EPMCSL = (SFR_BANK1 | 0x10),   /**<Pattern Match Filter Checksum*/
    XJ600_EPMOL = (SFR_BANK1 | 0x12),    /**<Pattern Match Filter Offset*/
    XJ600_ERXFCONL = (SFR_BANK1 | 0x14), /**<Ethernet RX Filter Control Register*/

    XJ600_MACON1L = (SFR_BANK2 | 0x00),   /**<MAC Control Register 1*/
    XJ600_MACON2L = (SFR_BANK2 | 0x02),   /**<MAC Control Register 2*/
    XJ600_MABBIPGL = (SFR_BANK2 | 0x04),  /**<MAC Back-to-Back Inter-Packet Gap*/
    XJ600_MAIPGL = (SFR_BANK2 | 0x06),    /**<MAC Inter-Packet Gap*/
    XJ600_MACLCONL = (SFR_BANK2 | 0x08),  /**<MAC Colision Control Register*/
    XJ600_MAMXFLL = (SFR_BANK2 | 0x0A),   /**<MAC Maximum Frame Length*/
    XJ600_MICMDL = (SFR_BANK2 | 0x12),    /**<MII Management Command*/
    XJ600_MIREGADRL = (SFR_BANK2 | 0x14), /**<MII Management Adresses*/

    XJ600_MAADR3L = (SFR_BANK3 | 0x00), /**<MAC Address Byte 5*/
    XJ600_MAADR3H = (SFR_BANK3 | 0x01), /**<MAC Address Byte 6*/
    XJ600_MAADR2L = (SFR_BANK3 | 0x02), /**<MAC Address Byte 3*/
    XJ600_MAADR2H = (SFR_BANK3 | 0x03), /**<MAC Address Byte 4*/
    XJ600_MAADR1L = (SFR_BANK3 | 0x04), /**<MAC Address Byte 1*/
    XJ600_MAADR1H = (SFR_BANK3 | 0x05), /**<MAC Address Byte 2*/
    XJ600_MIWRL = (SFR_BANK3 | 0x06),   /**<MII Management Write Data*/
    XJ600_MIRDL = (SFR_BANK3 | 0x08),   /**<MII Management Read Data*/
    XJ600_MISTATL = (SFR_BANK3 | 0x0A), /**<MII Management Status Register*/
    XJ600_EPAUSL = (SFR_BANK3 | 0x0C),  /**<Pause Timer Value*/
    XJ600_ECON2L = (SFR_BANK3 | 0x0E),  /**<Ethernet Control Register 2*/
    XJ600_ERXWML = (SFR_BANK3 | 0x10),  /**<Receive Watermark*/
    XJ600_EIEL = (SFR_BANK3 | 0x12),    /**<Ethernet Interrupt Enable Register*/
    XJ600_EIDLEDL = (SFR_BANK3 | 0x14), /**<Ethernet ID Status/LED Control Register*/

    XJ600_EGPDATAL = (UNBANKED | 0x00),  /**<Use the SRAM Instructions*/
    XJ600_ERXDATAL = (UNBANKED | 0x02),  /**<Use the SRAM Instructions*/
    XJ600_EUDADATAL = (UNBANKED | 0x04), /**<Use the SRAM Instructions*/
    XJ600_EGPRDPTL = (UNBANKED | 0x06),  /**<General Purpose Data Window Register*/
    XJ600_EGPWRPTL = (UNBANKED | 0x08),  /**<General Purpose Window Write Pointer*/
    XJ600_ERXRDPTL = (UNBANKED | 0x0A),  /**<RX Window Read Pointer*/
    XJ600_ERXWRPTL = (UNBANKED | 0x0C),  /**<RX Window Write Pointer*/
    XJ600_EUDARDPTL = (UNBANKED | 0x0E), /**<UDA Window Read Pointer*/
    XJ600_EUDAWRPTL = (UNBANKED | 0x10), /**<UDA Window Write Pointer*/

    XJ600_EUDASTL = (SFR_COMMON | 0x16), /**<User-Defined Area Start Pointer*/
    XJ600_EUDANDL = (SFR_COMMON | 0x18), /**<User-Defined Area End Pointer*/
    XJ600_ESTATL = (SFR_COMMON | 0x1A),  /**<Ethernet Status Register*/
    XJ600_EIRL = (SFR_COMMON | 0x1C),    /**<Ethernet Interrupt Flag Register 1*/
    XJ600_ECON1L = (SFR_COMMON | 0x1E)   /**<Ethernet Control Register 1*/
} encX24J600_registers_t;

/**
 * @ingroup encx24j600
 * @enum encX24J600_1_byte_instructions_t
 * @brief SPI single byte instructions.
 */
typedef enum
{
    b0sel_inst = 0b11000000,      /**<Bank 0 Select*/
    b1sel_inst = 0b11000010,      /**<Bank 1 Select*/
    b2sel_inst = 0b11000100,      /**<Bank 2 Select*/
    b3sel_inst = 0b11000110,      /**<Bank 3 Select*/
    setethrst_inst = 0b11001010,  /**<System Reset*/
    fcdisable_inst = 0b11100000,  /**<Flow Control Disable*/
    fcsingle_inst = 0b11100010,   /**<Flow Control Single*/
    fcmultiple_inst = 0b11100100, /**<Flow Control Multiple*/
    fcclear_inst = 0b11100110,    /**<Flow Control Clear*/
    setpktdec_inst = 0b11001100,  /**<Decrement Packet Counter*/
    dmastop_inst = 0b11010010,    /**<DMA Stop*/
    dmacksum_inst = 0b11011000,   /**<DMA Start Checksum*/
    dmacksums_inst = 0b11011010,  /**<DMA Start Checksum with Seed */
    dmacopy_inst = 0b11011100,    /**<DMA Start Copy*/
    dmacopys_inst = 0b11011110,   /**<DMA Start Copy and Checksum with Seed*/
    settxrts_inst = 0b11010100,   /**<Request Packet Transmission*/
    enablerx_inst = 0b11101000,   /**<Enable RX*/
    disablerx_inst = 0b11101010,  /**<Disable RX*/
    seteie_inst = 0b11101100,     /**<Enable Interrupts*/
    clreie_inst = 0b11101110      /**<Disable Interrupts*/
} encX24J600_1_byte_instructions_t;

/**
 * @ingroup encx24j600
 * @enum encX24J600_2_byte_instructions_t
 * @brief SPI two byte instructions.
 */
typedef enum
{
    rbsel_inst = 0b11001000 /**<Read Bank Select*/
} encX24J600_2_byte_instructions_t;

/**
 * @ingroup encx24j600
 * @enum encX24J600_3_byte_instructions_t
 * @brief SPI three byte instructions.
 */
typedef enum
{
    rgprdpt_inst = 0b01100010,  /**<Read EGPRDPT*/
    rrxrdpt_inst = 0b01100110,  /**<Read ERXRDPT*/
    rgpwrpt_inst = 0b01101110,  /**<Read EGPWRPT*/
    rrxwrpt_inst = 0b01110010,  /**<Read ERXRDPT*/
    rudawrpt_inst = 0b01110110, /**<Read EUDAWRPT*/
    wgprdpt_inst = 0b01100000,  /**<Write EGPRDPT*/
    wrxrdpt_inst = 0b01100100,  /**<Write ERXRDPT*/
    wgpwrpt_inst = 0b01101100,  /**<Write EGPWRPT*/
    wrxwrpt_inst = 0b01110000,  /**<Write ERXWRPT*/
    wudawrpt_inst = 0b01110100  /**<Write EUDAWRPT*/
} encX24J600_3_byte_instructions_t;

/**
 * @ingroup encx24j600
 * @enum encX24J600_N_byte_instructions_t
 * @brief SPI N byte instructions.
 */
typedef enum
{
    rcr_inst = 0b00000000, /**<Banked register read*/
    wcr_inst = 0b01000000, /**<Banked register write*/

    bfs_inst = 0b10000000, /**<Banked bit set*/
    bfc_inst = 0b10100000, /**<Banked bit clear*/

    rcru_inst = 0b00100000, /**<Unbanked SFR operation*/
    wcru_inst = 0b00100010, /**<Unbanked SFR writes*/
    bfsu_inst = 0b00100100, /**<Unbanked bit set*/
    bfcu_inst = 0b00100110, /**<Unbanked bit clear*/

    rgpdata_inst = 0b00101000,  /**<SRAM data EGPDATA read*/
    rrxdata_inst = 0b00101100,  /**<SRAM data RXDATA read*/
    rudadata_inst = 0b00110000, /**<SRAM data UDADATA read*/
    wgpdata_inst = 0b00101010,  /**<SRAM writes*/
    wrxdata_inst = 0b00101110,  /**<SRAM data write from ERXDATA*/
    wudadata_inst = 0b00110010  /**<SRAM data write from EUDADATA*/
} encX24J600_N_byte_instructions_t;

/**
 * @ingroup encx24j600
 * @union receiveStatusVector_t
 * @brief Receive status vector.
 */
typedef union
{
    uint8_t v[6];
    struct
    {
        uint16_t byteCount;                 /**<Received Byte Count*/
        unsigned PreviouslyIgnored : 1;     /**<Packet Previously Ignored*/
        unsigned RXDCPreviouslySeen : 1;    /**<RXDC Previously Seen*/
        unsigned CarrierPreviouslySeen : 1; /**<Carrier Event Previously Seen*/
        unsigned CodeViolation : 1;         /**<Code Violation*/
        unsigned CRCError : 1;              /**<CRC Error*/
        unsigned LengthCheckError : 1;      /**<Length Check Error*/
        unsigned LengthOutOfRange : 1;      /**<Length Out of Range*/
        unsigned ReceiveOk : 1;             /**<Received Ok*/

        unsigned Multicast : 1;         /**<Receive Multicast Packet*/
        unsigned Broadcast : 1;         /**<Receive Broadcast Packet*/
        unsigned DribbleNibble : 1;     /**<Dribble Nibble*/
        unsigned ControlFrame : 1;      /**<Receive Control Frame*/
        unsigned PauseControlFrame : 1; /**<Receive Pause Control Frame*/
        unsigned UnsupportedOpcode : 1; /**<Receive Unknown Opcode*/
        unsigned VLANType : 1;          /**<Receive VLAN Type Detected*/
        unsigned RuntMatch : 1;         /**<Runt Filter Match*/

        unsigned filler : 1;           /**<Not-Me Filter Match*/
        unsigned HashMatch : 1;        /**<Hash Filter Match*/
        unsigned MagicPacketMatch : 1; /**<Magic Packet(TM) Filter Match*/
        unsigned PatternMatch : 1;     /**<Pattern Match Filter Match*/
        unsigned UnicastMatch : 1;     /**<Unicast Filter Match*/
        unsigned BroadcastMatch : 1;   /**<Broadcast Filter Match*/
        unsigned MulticastMatch : 1;   /**<Multicast Filter Match*/
        unsigned ZeroH : 1;            /**<Zero*/

        unsigned char Zero : 8;
    };
} receiveStatusVector_t;

// Variables

extern receiveStatusVector_t rxPacketStatusVector;

extern uint16_t TXPacketSize;

// Function Prototypes

/**
 * @ingroup encx24j600
 * @brief Reads from Special Function Register (SFR).
 * @param a register
 * @return Two bytes of SFR value.
 */
uint16_t ENCx24_Read(encX24J600_registers_t a);

/**
 * @ingroup encx24j600
 * @brief Reads the 8-bit data from the ENCx24J600 register (work around for Errata).
 * @param a register
 * @return One byte of data.
 */
uint8_t ETH_EdataRead(encX24J600_registers_t a);

/**
 * @ingroup encx24j600
 * @brief Writes to SFRs.
 * @param a register
 * @param data
 * @return None.
 */
void ENCx24_Write(encX24J600_registers_t a, uint16_t data);

/**
 * @ingroup encx24j600
 * @brief Sets the SFR bit field.
 * @param a register
 * @param bitMask
 * @return None.
 */
void ENCx24_BFS(encX24J600_registers_t a, uint16_t bitMask);

/**
 * @ingroup encx24j600
 * @brief Clears the SFR bit field.
 * @param a register
 * @param bitMask
 * @return None.
 */
void ENCx24_BFC(encX24J600_registers_t a, uint16_t bitMask);

/**
 * @ingroup encx24j600
 * @brief Reads the PHY register.
 * @param a register
 * @return Two bytes of data.
 */
uint16_t ENCx24_PhyRead(encX24J600_phy_registers_t a);

/**
 * @ingroup encx24j600
 * @brief Writes the PHY register.
 * @param a register
 * @param data
 * @return None.
 */
void ENCx24_PHYWrite(encX24J600_phy_registers_t a, uint16_t data);

/**
 * @ingroup encx24j600
 * @brief Reads the 8-bit MAC-related data from ENCx24J600 registers.
 * @param a register
 * @return One byte of data.
 */
uint8_t ETH_MACRead8(encX24J600_registers_t a);

/**
 * @ingroup encx24j600
 * @brief Writes the 8-bit MAC-related data to ENCx24J600 registers.
 * @param a register
 * @param data
 * @return None.
 */
void ETH_MACWrite8(encX24J600_registers_t a, uint8_t data);

/**
 * @ingroup encx24j600
 * @brief Decrements packet count.
 * @param None.
 * @return None.
 */
void ETH_SetPktDec(void);

/**
 * @ingroup encx24j600
 * @brief RX packet read pointer.
 * @param address
 * @return None.
 */
void ETH_SetRXptr(uint16_t address);

/**
 * @ingroup encx24j600
 * @brief Packet transmission request.
 * @param None.
 * @return None.
 */
void ETH_Set_TXRTS(void);

#endif /* ENCX24J600_SPI_H */