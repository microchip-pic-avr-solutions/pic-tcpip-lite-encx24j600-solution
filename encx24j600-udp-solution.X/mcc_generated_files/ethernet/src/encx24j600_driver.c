/**
 * ENCx24J600 Ethernet Driver source file
 *
 * @file encx24j600_driver.c
 *
 * @ingroup encx24j600
 *
 * @brief This file provides the Ethernet driver API implementation for the ENCx24J600 family devices.
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

// Include Headers

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "../../system/system.h"
#include "../physical_layer_interface.h"
#include "../encx24j600_spi.h"
#include "../../spi/spi_interface.h"
#include "../../timer/delay.h"

// Macros


/**
 * @ingroup encx24j600
 * @def ETH_WRITE_IN_PROGRESS
 * @brief Packet write in progress, not ready for transmit.
 */
#define ETH_WRITE_IN_PROGRESS (0x0001 << 0)

/**
 * @ingroup encx24j600
 * @def ETH_TX_QUEUED
 * @brief Packet in queue for transmit.
 */
#define ETH_TX_QUEUED (0x0001 << 1)

/**
 * @ingroup encx24j600
 * @def ETH_ALLOCATED
 * @brief Flag for pool management. Set to either free or allocated.
 */
#define ETH_ALLOCATED (0x0001 << 2)

// Adjust these parameters for the MAC
#define MAX_TX_PACKET_SIZE (1518)
#define MIN_TX_PACKET_SIZE (64)
#define MAX_TX_PACKETS (20)

#define MIN_TX_PACKET (MIN_TX_PACKET_SIZE)
#define TX_BUFFER_SIZE (TXEND - TXSTART) // 5631

#define TX_BUFFER_MID ((TXSTART + TXEND) >> 1) // 2815

#define SetBit(bitField,bitMask) do{bitField = bitField | bitMask; } while(0)
#define ClearBit(bitField,bitMask) do{bitField = bitField & (~bitMask); } while(0)
#define CheckBit(bitField,bitMask) (bool)(bitField & bitMask)

// MAC Address

#define MAC_ADDRESS {0x02, 0x00, 0x00, 0x00, 0x00, 0x01}
#define BROADCAST_MAC {0xff,0xff,0xff,0xff,0xff,0xff}

// Variables

mac48Address_t macAddress = {MAC_ADDRESS};
const mac48Address_t broadcastMAC = {BROADCAST_MAC};
mac48Address_t ethMAC;

volatile ethernetDriver_t ethData;

// Define a temporary register for passing data to inline assembly
// Work around the 110110 LSB errata and to control RDPTR WRPTR update counts

uint8_t ethListSize;

static txPacket_t txData[MAX_TX_PACKETS];

static txPacket_t *pHead;
static txPacket_t *pTail;

static uint16_t nextPacketPointer;

// Function Prototypes

/**
 * @ingroup encx24j600
 * @brief Enqueues the latest written packet and starts the transmission of a queued packet.
 * @param None.
 * @retval PHY_SUCCESS
 * @retval PHY_BUFFER_BUSY
 */
phy_error_msg ETH_SendQueued(void);

/**
 * @ingroup encx24j600
 * @brief Copies all the queued packets to the TX Buffer start address using the DMA setup.
 * @param None.
 * @retval PHY_SUCCESS
 * @retval PHY_DMA_TIMEOUT
 */
phy_error_msg ETH_Shift_Tx_Packets(void);

/**
 * @ingroup encx24j600
 * @brief Resets the Ethernet packet list.
 * @param None.
 * @return None.
 */
void ETH_PacketListReset(void);

/**
 * @ingroup encx24j600
 * @brief Allocates a new packet element and links it to the chained list.
 * @param None.
 * @return Packet address.
 */
txPacket_t *ETH_NewPacket(void);

/**
 * @ingroup encx24j600
 * @brief Deallocates a packet element and unlinks it from the chained list.
 * @param *pPacket packetHandle
 * @return None.
 */
void ETH_RemovePacket(txPacket_t *pPacket);

/**
 * @ingroup encx24j600
 * @brief Calculates the checksum.
 * @param rxPtr RX pointer
 * @param len length
 * @param seed
 * @return Checksum.
 */
static uint16_t ETH_DMAComputeChecksum(uint16_t rxPtr, uint16_t len, uint16_t seed);

/**
 * @ingroup encx24j600
 * @brief Releases the Serial Peripheral Interface (SPI) bus.
 * @param None.
 * @return None.
 */
static void ETH_CloseSPI(void);

/**
 * @ingroup encx24j600
 * @brief Connects the SPI bus.
 * @param None.
 * @return None.
 */
static void ETH_OpenSPI(void);

/**
 * @ingroup encx24j600
 * @brief Waits for the DMA.
 * @param None.
 * @return None.
 */
static inline void waitForDMA(void);

/**
 * @ingroup encx24j600
 * @brief Checks for the link status.
 * @param None.
 * @retval PHY_SUCCESS  link found
 * @retval PHY_LINK_NOT_FOUND link is not present
 */
static inline uint32_t ETH_readLinkStatus(void);

// Functions

const mac48Address_t *MAC_getAddress(void)
{
    return &macAddress;
}

void ETH_SendSystemReset(void)
{
    do
    {
        // Waits until the function gets response from device
        do
        {
            ENCx24_Write(XJ600_EUDASTL, 0x1234);
        } while (0x1234 != ENCx24_Read(XJ600_EUDASTL));

        // Reset to assure that device is in a fresh state
        ENCx24_BFS(XJ600_ECON2L, ECON2_ETHRST);

        while((ENCx24_Read(XJ600_ESTATL) & (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY)) != (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY));
        DELAY_milliseconds(1);

    } while (0x0 != ENCx24_Read(XJ600_EUDASTL));

    DELAY_milliseconds(1);

    // Device is ready
}

phy_error_msg ETH_SendQueued(void)
{
    uint16_t econ1;

    if (pHead->flags & ETH_TX_QUEUED)
    {
        // Close the latest written packet and enqueue it
        ClearBit(pHead->flags, ETH_TX_QUEUED); // txQueued = false

        // Start transmitting from the tails. The packet first written.
        ENCx24_Write(XJ600_ETXSTL, pTail->packetStart);

        NOP();
        NOP();
        econ1 = ENCx24_Read(XJ600_ECON1L);
        ENCx24_Write(XJ600_ECON1L, (econ1 | 0x02)); // Start sending ECON1bits.TXRTS = 1
        ETH_RemovePacket(pTail);
        return PHY_SUCCESS;
    }
    else
    {
        return PHY_BUFFER_BUSY;
    }
}

phy_error_msg ETH_Shift_Tx_Packets(void)
{
    uint16_t timer;
    uint16_t len = pHead->packetEnd - pTail->packetStart;

    timer = 2 * len;
    while ((ENCx24_Read(XJ600_ECON1L) & ECON1_DMAST) != 0 && --timer)
        NOP(); // Waits until the DMA is free  ECON1bits.DMAST!=0
    if ((ENCx24_Read(XJ600_ECON1L) & ECON1_DMAST) == 0)
    {
        ENCx24_Write(XJ600_EDMADSTL, TXSTART); // Setup the destination start pointer

        ENCx24_Write(XJ600_EDMASTL, pTail->packetStart); // Setup the source pointer from the current read pointer
        ENCx24_Write(XJ600_EDMALENL, len);

        // Clear DMANOCS to select a checksum operation
        ENCx24_BFC(XJ600_ECON1L, 0x0004);
        // Set DMACPY to select a copy operation
        // Set DMACSSD to use a custom seed
        // Set DMAST to start the DMA operation
        ENCx24_BFS(XJ600_ECON1L, 0b0000000000111000);

        // It might take longer to complete if there is heavy network traffic
        timer = 40 * len;
        while ((ENCx24_Read(XJ600_ECON1L) & ECON1_DMAST) != 0 && --timer)
            NOP();
        if ((ENCx24_Read(XJ600_ECON1L) & ECON1_DMAST) == 0)
        {
            // Updates the start and end addresses of each packet
            txPacket_t *pElem = pHead;
            uint16_t shiftOffset = pTail->packetStart;

            while (pElem != NULL)
            {
                pElem->packetStart = pElem->packetStart - shiftOffset;
                pElem->packetEnd = pElem->packetEnd - shiftOffset;
                pElem = pElem->nextPacket;
            }

            // Updates the EWRPT
            ENCx24_Write(XJ600_EGPWRPTL, (TXSTART + len));
            return PHY_SUCCESS;
        }
    }

    // DMA timed out
    RESET(); // Reboot device
    return PHY_DMA_TIMEOUT;
}

void ETH_PacketListReset(void)
{
    uint16_t index = 0;
    uint8_t *ptr = (uint8_t *)txData;
    ethListSize = 0;

    pHead = NULL;
    pTail = NULL;

    while (index < (MAX_TX_PACKETS * sizeof(txPacket_t)))
    {
        ptr[index] = 0;
        index++;
    }
}

txPacket_t *ETH_NewPacket(void)
{
    uint8_t index = 0;

    if (ethListSize == MAX_TX_PACKETS)
    {
        return NULL;
    }

    while (index < MAX_TX_PACKETS)
    {
        if (CheckBit(txData[index].flags, ETH_ALLOCATED) == false)
        {
            txData[index].flags = 0;                    // Reset all flags
            SetBit(txData[index].flags, ETH_ALLOCATED); // Mark the handle as allocated (Allocated = true)

            txData[index].packetEnd = TXEND;

            txData[index].prevPacket = NULL;
            txData[index].nextPacket = pHead;

            if (pHead != NULL)
            {
                pHead->prevPacket = &txData[index];
                txData[index].packetStart = pHead->packetEnd;

                // Try to keep a 2 byte alignment
                if (txData[index].packetStart & 0x0001)
                {
                    // Make sure the end of the packet is odd, so the beginning of the next one is even
                    ++txData[index].packetStart;
                }
            }
            else
            {
                txData[index].packetStart = TXSTART;
                pTail = (txPacket_t *)&txData[index];
            }

            pHead = (txPacket_t *)&txData[index];

            ethListSize++;
            return &txData[index];
        }
        index++;
    }
    return NULL;
}

void ETH_RemovePacket(txPacket_t *pPacket)
{
    #ifdef VALIDATE_ALLOCATED_PTR
    uint8_t index = 0;
    #endif // VALIDATE_ALLOCATED_PTR

    if ((pPacket == NULL) || (ethListSize == 0))
    {
        return;
    }

    #ifdef VALIDATE_ALLOCATED_PTR
    while (index < MAX_TX_PACKETS)
    {
        if ((pPacket == &txData[index]) && (txData[index].allocated == true))
        {
            break;
        }
        index++;
    }
    if (index == MAX_TX_PACKETS)
    {
        return;
    }
    #endif // VALIDATE_ALLOCATED_PTR

    // Unlinks from the chained list
    if (pPacket->nextPacket == NULL)
    {
        pTail = pPacket->prevPacket;
        if (pTail != NULL)
        {
            pTail->nextPacket = NULL;
        }
    }

    if (pPacket->prevPacket == NULL)
    {
        pHead = pPacket->nextPacket;
        if (pHead != NULL)
        {
            pHead->prevPacket = NULL;
        }
    }

    // Deallocate
    pPacket->flags = 0;
    pPacket->prevPacket = NULL;
    pPacket->nextPacket = NULL;

    ethListSize--;

    return;
}

static void ETH_CloseSPI(void)
{
    SPI1_Close();
}

static void ETH_OpenSPI(void)
{
    while (!SPI1_Open(MAC));
}

void ETH_Init(void)
{
    uint16_t phcon1_val;

    ETH_OpenSPI();
    ETH_NCS_HIGH();

    ethData.error = false; // No error
    ethData.up = false;    // No link
    ethData.linkChange = false;

    ETH_PacketListReset();

    ethData.saveRDPT = 0;

    ETH_SendSystemReset();

    // Initializes RX tracking variables and other control state flags
    nextPacketPointer = RXSTART;

    // Sets up TX/RX buffer addresses
    ENCx24_Write(XJ600_ETXSTL, TXSTART);
    ENCx24_Write(XJ600_ERXSTL, RXSTART);
    ENCx24_Write(XJ600_ERXTAILL, RXEND);
    ENCx24_Write(XJ600_EUDASTL, TXEND + 1);
    ENCx24_Write(XJ600_EUDANDL, TXEND + 1);
    ENCx24_Write(XJ600_EGPWRPTL, TXSTART);

    phcon1_val = ENCx24_PhyRead(PHCON1);
    phcon1_val &= PHCON1_PWAKE;
    ENCx24_PHYWrite(PHCON1, phcon1_val);

    // Set PHY Auto-negotiation to support 10BaseT Half duplex, 10BaseT Full duplex,
    // 100BaseTX Half Duplex, 100BaseTX Full Duplex, and symmetric PAUSE capability
    // ADPAUS=1;AD10FD=1;AD10=1;AD100FD=1;AD100=1;ADIEEE=1;
    // PHANA=0b 0000 0101 1110 0001
    ENCx24_PHYWrite(PHANA, 0x05E1);

    // Wait until internal clock is ready and PHY link established
    while ((ENCx24_Read(XJ600_ESTATL) & 0X1100) != 0x1100)
    {
        DELAY_microseconds(256);
    }

    // Configure the filter
    // UCEN   = 1;  // Allow unicast   (enables IP & ICMP)
    // BCEN   = 1;  // Allow broadcast (enables the ARP responses)
    // RUNTEN = 1;  // Disallow runts
    // CRCEN  = 1;  // Disallow bad CRC
    // ERXFCON = 0b 0000 0000 0101 1011
    // MCEN   = 1;  // Allow multicast
    // ENCx24_Write(XJ600_ERXFCONL, 0x005B);

    // MCEN   = 0;  // Filter out multicast
    ENCx24_Write(XJ600_ERXFCONL, 0x0059);

    // Enable packet receive interrupts
    // INTIE = 1;    // Host interrupt flag
    // PKTIE = 1;    // Receive packet interrupts
    // LNKIE = 1;    // Link changed interrupts
    // TXIE  = 1;    // Sent packet interrupts
    // PCFULIE = 1;  // Packet counter full interrupt
    // RXABTIE = 1;  // Receive abort enable interrupt
    // EIE = 0b 1000 1000 0100 1011
    ENCx24_Write(XJ600_EIEL, 0x884B);

    // Enables RX packet reception
    ENCx24_BFS(XJ600_ECON1L, ECON1_RXEN);

    // Chooses Bank 0
    ETH_WRITE8(b0sel_inst);

    ETH_GetMAC((uint8_t *)&ethMAC);

    // Checks for a preexisting link
    ETH_CheckLinkUp();
}

static inline uint32_t ETH_readLinkStatus(void)
{
    return (ENCx24_Read(XJ600_ESTATL));
}

bool ETH_CheckLinkUp(void)
{
    uint32_t value;
    bool ret = false;
    // Check for a preexisting link
    value = ETH_readLinkStatus();
    if (value & 0x0100)
    {
        ethData.up = true;
        ret = true;
    }
    return ret;
}

void ETH_EventHandler(void)
{
    uint16_t eirVal, estatVal, mabbipgVal, macon2Val;

    estatVal = ENCx24_Read(XJ600_ESTATL);
    // MAC is sending an interrupt
    eirVal = ENCx24_Read(XJ600_EIRL);

    if (eirVal != 0)
    {
        if (eirVal & EIR_LINKIF) // Something about the link changed. Update the link parameters.
        {
            ethData.linkChange = true;
            macon2Val = ENCx24_Read(XJ600_MACON2L);

            ethData.up = false;
            if (estatVal & ESTAT_PHYLINK)
            {
                ethData.up = true;
            }

            // Updates MAC duplex settings to match PHY duplex setting
            if (estatVal & ESTAT_PHYDPX)
            {
                // Switching to full duplex
                mabbipgVal = 0x15;
                macon2Val |= MACON2_FULDPX_ON;
            }
            else
            {
                // Switching to half duplex
                mabbipgVal = 0x12;
                macon2Val |= MACON2_FULDPX_OFF;
            }
            ENCx24_Write(XJ600_MABBIPGL, mabbipgVal);
            ENCx24_Write(XJ600_MACON2L, macon2Val);
        }
        if (eirVal & EIR_TXIF) // Finished sending a packet
        {
            if (ethListSize > 0)
            {
                if (ENCx24_Read(XJ600_EGPWRPTL) > TX_BUFFER_MID)
                {
                    // Shifts all the queued packets to the start of the TX Buffer
                    ETH_Shift_Tx_Packets();
                }
                // Sends the next queued packet
                ETH_SendQueued();
            }
        }
        if ((eirVal & EIR_RXABTIF) || (eirVal & EIR_PCFULIF)) // Buffer overflow
        {
            ETH_ResetReceiver();
        }
        // Checks if there are any received frames
        if (estatVal & ESTAT_PKTCNT || eirVal & EIR_PKTIF) // Received a packet
        {
            if (ethData.pktReady == false)
            {
                ethData.pktReady = true;
            }
        }
        ENCx24_BFC(XJ600_EIRL, eirVal); // Write the EIR value back to clear any of the interrupts
    }
}

void ETH_NextPacketUpdate(void)
{
    ETH_SetRXptr(nextPacketPointer);

    ETH_NCS_LOW();
    ETH_WRITE8(rrxdata_inst);
    ((char *)&nextPacketPointer)[0] = ETH_READ8();
    ((char *)&nextPacketPointer)[1] = ETH_READ8();
    ((char *)&rxPacketStatusVector)[0] = ETH_READ8();
    ((char *)&rxPacketStatusVector)[1] = ETH_READ8();
    ((char *)&rxPacketStatusVector)[2] = ETH_READ8();
    ((char *)&rxPacketStatusVector)[3] = ETH_READ8();
    ((char *)&rxPacketStatusVector)[4] = ETH_READ8();
    ((char *)&rxPacketStatusVector)[5] = ETH_READ8();

    ETH_NCS_HIGH();

    rxPacketStatusVector.byteCount -= 4; // Frame checksum can be ignored at the end
                                         // The checksum is 4 bytes. So the payload is the byte count less 4.
}

void ETH_ResetReceiver(void)
{
    ENCx24_BFS(XJ600_ECON2L, ECON2_RXRST);
    ENCx24_BFC(XJ600_ECON2L, ECON2_RXRST);

    ENCx24_BFC(XJ600_ECON1L, ECON1_RXEN);

    // Initializes RX tracking variables and other control state flags
    nextPacketPointer = RXSTART;

    // Sets up RX buffer addresses
    ENCx24_Write(XJ600_ERXSTL, RXSTART);
    ENCx24_Write(XJ600_ERXTAILL, RXEND);

    // Enables RX packet reception
    ENCx24_BFS(XJ600_ECON1L, ECON1_RXEN);
}

void ETH_Dump(uint16_t length)
{
    uint16_t newRXTail;
    length = (rxPacketStatusVector.byteCount <= length) ? rxPacketStatusVector.byteCount : length;
    if (length)
    {
        newRXTail = ENCx24_Read(XJ600_ERXRDPTL);
        newRXTail += length;
        // Writes new RX tail
        ENCx24_Write(XJ600_ERXRDPTL, newRXTail);
    }
    rxPacketStatusVector.byteCount -= length;
}

void ETH_Flush(void)
{
    uint16_t newRXTail = nextPacketPointer - 2;

    // Special situation if nextPacketPointer is exactly RXSTART
    if (nextPacketPointer == RXSTART)
        newRXTail = RXEND;

    // Packet Decrement
    ETH_SetPktDec();
    ethData.pktReady = false; // IRQ line should remain low if PKTCNT is != 0

    // Writes new RX tail
    ENCx24_Write(XJ600_ERXTAILL, newRXTail);
}

// Ethernet Write Functions
// The Ethernet system needs a variety of ways to write to the MAC
// By making separate functions at the driver level, the upper layers get smaller

phy_error_msg ETH_WriteStart(const mac48Address_t *dest_mac, uint16_t type)
{
    txPacket_t *ethPacket = NULL;

    TXPacketSize = 0;

    if (ENCx24_Read(XJ600_EGPWRPTL) > TX_BUFFER_MID)
    {
        // Need to shift all the queued packets to the start of the TX Buffer

        // Checks for TX in progress
        if ((ENCx24_Read(XJ600_ECON1L) & 0x02))
        {
            return PHY_TX_LOGIC_NOT_IDLE;
        }
        // Try to move the queued packets
        ETH_Shift_Tx_Packets();

        // Verify if shift
        if (ENCx24_Read(XJ600_EGPWRPTL) > TX_BUFFER_MID)
        {
            return PHY_BUFFER_BUSY;
        }
    }

    // Creates new packet and queue it in the TX Buffer

    // Initialize a new packet handler. It is automatically placed in the queue
    ethPacket = (txPacket_t *)ETH_NewPacket();

    if (ethPacket == NULL)
    {
        // No more available packets
        return PHY_BUFFER_BUSY;
    }

    SetBit(ethPacket->flags, ETH_WRITE_IN_PROGRESS); // writeInProgress = true;

    ENCx24_Write(XJ600_EGPWRPTL, ethPacket->packetStart);

    ETH_ResetByteCount(); // Check if necessary

    // TXPacketSize = 0;
    ETH_WriteBlock((char *)dest_mac, 6); // destination
    ETH_WriteBlock((char *)&ethMAC, 6);  // source
    ETH_Write16(type);

    return PHY_SUCCESS;
}

phy_error_msg ETH_Send(void)
{
    uint16_t packetEnd = (ENCx24_Read(XJ600_EGPWRPTL)) - 1;

    // Waits for any transmissions to finish
    ENCx24_Write(XJ600_ETXLENL, TXPacketSize);
    if (!ethData.up)
    {
        return PHY_LINK_NOT_FOUND;
    }

    if (ethListSize == 0)
    {
        return PHY_BUFFER_BUSY; // This is a false message
    }

    ClearBit(pHead->flags, ETH_WRITE_IN_PROGRESS); // WriteInProgress = false
    pHead->packetEnd = packetEnd;
    SetBit(pHead->flags, ETH_TX_QUEUED); // txQueued = true
    // The packet is prepared to be sent is queued

    if ((ENCx24_Read(XJ600_ECON1L) & 0x02) || (ethListSize > 1)) // Checking for (ECON1bits.TXRTS)
    {
        return PHY_TX_QUEUED;
    }

    return ETH_SendQueued();
}

phy_error_msg ETH_Copy(uint16_t len)
{
    uint16_t txBufferAddress;

    // Setup DMA to copy length bytes from RX buffer to TX buffer
    // This is used for ICMP ECHO to eliminate the need to extract the arbitrary payload
    // Wait until module is idle
    waitForDMA();

    if ((ENCx24_Read(XJ600_ECON1L) & 0x20) == 0) // ECON1bits.DMAST == 0
    {
        txBufferAddress = ENCx24_Read(XJ600_EGPWRPTL); // Current Window Write Pointer (the TX Buffer)
        ENCx24_Write(XJ600_EDMASTL, ethData.saveRDPT);
        ENCx24_Write(XJ600_EDMALENL, len);
        ENCx24_Write(XJ600_EDMADSTL, txBufferAddress);

        // Clear DMANOCS to select a checksum operation
        ENCx24_BFC(XJ600_ECON1L, 0x0004);
        // Set DMACPY to select a copy operation
        // Set DMACSSD to use a custom seed
        // Set DMAST to start the DMA operation
        ENCx24_BFS(XJ600_ECON1L, 0b0000000000111000);
        waitForDMA();

        // Clean up the source and destination window pointers
        txBufferAddress += len;

        ENCx24_Write(XJ600_EGPWRPTL, txBufferAddress);

        TXPacketSize += len; // fix the packet length

        return PHY_SUCCESS;
    }
    RESET();
    return PHY_DMA_TIMEOUT;
}

void ETH_GetMAC(uint8_t *macAddr)
{
    // Get MAC address
    *macAddr++ = ETH_MACRead8(XJ600_MAADR1L);
    *macAddr++ = ETH_MACRead8(XJ600_MAADR1H);
    *macAddr++ = ETH_MACRead8(XJ600_MAADR2L);
    *macAddr++ = ETH_MACRead8(XJ600_MAADR2H);
    *macAddr++ = ETH_MACRead8(XJ600_MAADR3L);
    *macAddr++ = ETH_MACRead8(XJ600_MAADR3H);
}

void ETH_SetMAC(uint8_t *macAddr)
{
    ETH_MACWrite8(XJ600_MAADR1L, *macAddr++);
    ETH_MACWrite8(XJ600_MAADR1H, *macAddr++);
    ETH_MACWrite8(XJ600_MAADR2L, *macAddr++);
    ETH_MACWrite8(XJ600_MAADR2H, *macAddr++);
    ETH_MACWrite8(XJ600_MAADR3L, *macAddr++);
    ETH_MACWrite8(XJ600_MAADR3H, *macAddr++);
}

inline static void waitForDMA(void)
{
    uint16_t econ1_val;
    do
    {
        econ1_val = ENCx24_Read(XJ600_ECON1L);
    } while (econ1_val & ECON1_DMAST);
}

static uint16_t ETH_DMAComputeChecksum(uint16_t rxPtr, uint16_t len, uint16_t seed)
{
    uint16_t cksm = 0;
    uint16_t econ1Val;

    // Waits until module is idle
    waitForDMA();

    // Clears DMACPY to prevent a copy operation
    // Clears DMANOCS to select a checksum operation
    // Clears DMACSSD to use the default seed of 0000h
    econ1Val = 0;
    // econ1.DMACPY = 1; econ1.DMANOCS = 1; econ1.DMACSSD = 1;
    // 0x001C
    econ1Val = 0x001C;
    ENCx24_BFC(XJ600_ECON1L, econ1Val);
    // Sets EDMAST to source address
    ENCx24_Write(XJ600_EDMASTL, rxPtr);
    // Sets EDMALEN to length
    ENCx24_Write(XJ600_EDMALENL, len);

    econ1Val = ECON1_DMAST;
    if (seed)
    {
        econ1Val |= ECON1_DMACSSD;
        seed = ~(seed);
        seed = htons(seed);
        ENCx24_Write(XJ600_EDMACSL, seed);
    }

    // Initiates operation
    ENCx24_BFS(XJ600_ECON1L, econ1Val);
    // Waits until done
    waitForDMA();
    cksm = ENCx24_Read(XJ600_EDMACSL);

    ENCx24_Write(XJ600_ERXRDPTL, rxPtr);

    return cksm;
}

uint16_t ETH_TxComputeChecksum(uint16_t position, uint16_t length, uint16_t seed)
{
    uint16_t econ1Val;
    uint16_t cksm;
    // Waits until module is idle
    waitForDMA();

    // Clears DMACPY to prevent a copy operation
    // Clears DMANOCS to select a checksum operation
    // Clears DMACSSD to use the default seed of 0000h
    econ1Val = 0;
    // econ1.DMACPY = 1; econ1.DMANOCS = 1; econ1.DMACSSD = 1;
    // 0x001C
    econ1Val = 0x001C;
    ENCx24_BFC(XJ600_ECON1L, econ1Val);

    // Sets EDMAST to source address
    ENCx24_Write(XJ600_EDMASTL, position);
    // Sets EDMALEN to length
    ENCx24_Write(XJ600_EDMALENL, length);

    econ1Val = ECON1_DMAST;
    // If there is a seed, Update SFR address XJ600_EDMACSL.
    if (seed)
    {
        econ1Val |= ECON1_DMACSSD;
        seed = ~(seed);
        seed = htons(seed);
        ENCx24_Write(XJ600_EDMACSL, seed);
    }
    // Initiates operation
    ENCx24_BFS(XJ600_ECON1L, econ1Val);
    // Waits until done
    waitForDMA();
    cksm = ENCx24_Read(XJ600_EDMACSL);
    return cksm;
}

uint16_t ETH_RxComputeChecksum(uint16_t len, uint16_t seed)
{
    uint16_t cksm;
    uint16_t rxPtr;
    uint16_t length;

    rxPtr = ENCx24_Read(XJ600_ERXRDPTL);
    if (rxPtr > (RXEND - len))
    {
        length = RXEND - rxPtr;
        seed = ETH_DMAComputeChecksum(rxPtr, length, seed);
        len = length - len;
        rxPtr = RXSTART;
    }

    cksm = ETH_DMAComputeChecksum(rxPtr, len, seed);
    return (uint16_t)cksm;
}

void ETH_SaveRDPT(void)
{
    ethData.saveRDPT = ENCx24_Read(XJ600_ERXRDPTL);
}

void ETH_ResetByteCount(void)
{
    ethData.saveWRPT = ENCx24_Read(XJ600_EGPWRPTL);
}

uint16_t ETH_GetByteCount(void)
{
    uint16_t wPtr;

    wPtr = ENCx24_Read(XJ600_EGPWRPTL);

    return (wPtr - ethData.saveWRPT);
}

uint16_t ETH_GetReadPtr(void)
{
    return ENCx24_Read(XJ600_ERXRDPTL);
}

void ETH_SetReadPtr(uint16_t rdptr)
{
    ENCx24_Write(XJ600_ERXRDPTL, rdptr);
}

void ETH_MoveBackReadPtr(uint16_t offset)
{
    uint16_t rdptr;
    rdptr = ENCx24_Read(XJ600_ERXRDPTL);
    ENCx24_Write(XJ600_ERXRDPTL, rdptr - offset);
    ETH_SetRxByteCount(offset);
}

void ETH_ResetReadPtr(void)
{
    ENCx24_Write(XJ600_ERXRDPTL, RXSTART);
}

uint16_t ETH_GetWritePtr(void)
{
    return ENCx24_Read(XJ600_EGPWRPTL);
}

uint16_t ETH_GetRxByteCount(void)
{
    return (rxPacketStatusVector.byteCount);
}

void ETH_SetRxByteCount(uint16_t count)
{
    rxPacketStatusVector.byteCount += count;
}

void ETH_SaveWRPT(void)
{
    ethData.saveWRPT = ENCx24_Read(XJ600_EGPWRPTL);
}

uint16_t ETH_ReadSavedWRPT(void)
{
    return ethData.saveWRPT;
}

uint16_t ETH_GetStatusVectorByteCount(void)
{
    return (rxPacketStatusVector.byteCount);
}

void ETH_SetStatusVectorByteCount(uint16_t bc)
{
    rxPacketStatusVector.byteCount = bc;
}

void ETH_TxReset(void)
{
    uint16_t econ2;
    econ2 = ENCx24_Read(XJ600_ECON2L);

    ENCx24_Write(XJ600_ECON2L, (econ2 | 0x0040));

    ETH_ResetByteCount();
    ENCx24_Write(XJ600_ETXSTL, TXSTART);
    ENCx24_Write(XJ600_EGPWRPTL, TXSTART);

    ETH_PacketListReset();
}
