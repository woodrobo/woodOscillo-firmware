/*******************************************************************************
  UART5 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart5.c

  Summary:
    UART5 PLIB Implementation File

  Description:
    None

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#include "device.h"
#include "plib_uart5.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART5 Implementation
// *****************************************************************************
// *****************************************************************************

UART_RING_BUFFER_OBJECT uart5Obj;

#define UART5_READ_BUFFER_SIZE      128
#define UART5_READ_BUFFER_SIZE_9BIT (128 >> 1)
#define UART5_RX_INT_DISABLE()      IEC5CLR = _IEC5_U5RXIE_MASK;
#define UART5_RX_INT_ENABLE()       IEC5SET = _IEC5_U5RXIE_MASK;

static uint8_t UART5_ReadBuffer[UART5_READ_BUFFER_SIZE];

#define UART5_WRITE_BUFFER_SIZE     128
#define UART5_WRITE_BUFFER_SIZE_9BIT       (128 >> 1)
#define UART5_TX_INT_DISABLE()      IEC5CLR = _IEC5_U5TXIE_MASK;
#define UART5_TX_INT_ENABLE()       IEC5SET = _IEC5_U5TXIE_MASK;

static uint8_t UART5_WriteBuffer[UART5_WRITE_BUFFER_SIZE];

#define UART5_IS_9BIT_MODE_ENABLED()    ( U5MODE & (_U5MODE_PDSEL0_MASK | _U5MODE_PDSEL1_MASK)) == (_U5MODE_PDSEL0_MASK | _U5MODE_PDSEL1_MASK) ? true:false

void static UART5_ErrorClear( void )
{
    UART_ERROR errors = UART_ERROR_NONE;
    uint8_t dummyData = 0u;

    errors = (UART_ERROR)(U5STA & (_U5STA_OERR_MASK | _U5STA_FERR_MASK | _U5STA_PERR_MASK));

    if(errors != UART_ERROR_NONE)
    {
        /* If it's a overrun error then clear it to flush FIFO */
        if(U5STA & _U5STA_OERR_MASK)
        {
            U5STACLR = _U5STA_OERR_MASK;
        }

        /* Read existing error bytes from FIFO to clear parity and framing error flags */
        while(U5STA & _U5STA_URXDA_MASK)
        {
            dummyData = U5RXREG;
        }

        /* Clear error interrupt flag */
        IFS5CLR = _IFS5_U5EIF_MASK;

        /* Clear up the receive interrupt flag so that RX interrupt is not
         * triggered for error bytes */
        IFS5CLR = _IFS5_U5RXIF_MASK;

    }

    // Ignore the warning
    (void)dummyData;
}

void UART5_Initialize( void )
{
    /* Set up UxMODE bits */
    /* STSEL  = 0 */
    /* PDSEL = 0 */

    U5MODE = 0x8;

    /* Enable UART5 Receiver and Transmitter */
    U5STASET = (_U5STA_UTXEN_MASK | _U5STA_URXEN_MASK | _U5STA_UTXISEL1_MASK );

    /* BAUD Rate register Setup */
    U5BRG = 216;

    /* Disable Interrupts */
    IEC5CLR = _IEC5_U5EIE_MASK;

    IEC5CLR = _IEC5_U5RXIE_MASK;

    IEC5CLR = _IEC5_U5TXIE_MASK;

    /* Initialize instance object */
    uart5Obj.rdCallback = NULL;
    uart5Obj.rdInIndex = 0;
    uart5Obj.rdOutIndex = 0;
    uart5Obj.isRdNotificationEnabled = false;
    uart5Obj.isRdNotifyPersistently = false;
    uart5Obj.rdThreshold = 0;

    uart5Obj.wrCallback = NULL;
    uart5Obj.wrInIndex = 0;
    uart5Obj.wrOutIndex = 0;
    uart5Obj.isWrNotificationEnabled = false;
    uart5Obj.isWrNotifyPersistently = false;
    uart5Obj.wrThreshold = 0;

    uart5Obj.errors = UART_ERROR_NONE;

    if (UART5_IS_9BIT_MODE_ENABLED())
    {
        uart5Obj.rdBufferSize = UART5_READ_BUFFER_SIZE_9BIT;
        uart5Obj.wrBufferSize = UART5_WRITE_BUFFER_SIZE_9BIT;
    }
    else
    {
        uart5Obj.rdBufferSize = UART5_READ_BUFFER_SIZE;
        uart5Obj.wrBufferSize = UART5_WRITE_BUFFER_SIZE;
    }


    /* Turn ON UART5 */
    U5MODESET = _U5MODE_ON_MASK;

    /* Enable UART5_FAULT Interrupt */
    IEC5SET = _IEC5_U5EIE_MASK;

    /* Enable UART5_RX Interrupt */
    IEC5SET = _IEC5_U5RXIE_MASK;
}

bool UART5_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    int32_t brgValHigh = 0;
    int32_t brgValLow = 0;
    uint32_t brgVal = 0;
    uint32_t uartMode;

    if (setup != NULL)
    {
        baud = setup->baudRate;

        if (baud == 0)
        {
            return status;
        }

        if(srcClkFreq == 0)
        {
            srcClkFreq = UART5_FrequencyGet();
        }

        /* Calculate BRG value */
        brgValLow = (((srcClkFreq >> 4) + (baud >> 1)) / baud ) - 1;
        brgValHigh = (((srcClkFreq >> 2) + (baud >> 1)) / baud ) - 1;

        /* Check if the baud value can be set with low baud settings */
        if((brgValLow >= 0) && (brgValLow <= UINT16_MAX))
        {
            brgVal =  brgValLow;
            U5MODECLR = _U5MODE_BRGH_MASK;
        }
        else if ((brgValHigh >= 0) && (brgValHigh <= UINT16_MAX))
        {
            brgVal = brgValHigh;
            U5MODESET = _U5MODE_BRGH_MASK;
        }
        else
        {
            return status;
        }

        if(setup->dataWidth == UART_DATA_9_BIT)
        {
            if(setup->parity != UART_PARITY_NONE)
            {
               return status;
            }
            else
            {
               /* Configure UART5 mode */
               uartMode = U5MODE;
               uartMode &= ~_U5MODE_PDSEL_MASK;
               U5MODE = uartMode | setup->dataWidth;
            }
        }
        else
        {
            /* Configure UART5 mode */
            uartMode = U5MODE;
            uartMode &= ~_U5MODE_PDSEL_MASK;
            U5MODE = uartMode | setup->parity ;
        }

        /* Configure UART5 mode */
        uartMode = U5MODE;
        uartMode &= ~_U5MODE_STSEL_MASK;
        U5MODE = uartMode | setup->stopBits ;

        /* Configure UART5 Baud Rate */
        U5BRG = brgVal;

        if (UART5_IS_9BIT_MODE_ENABLED())
        {
            uart5Obj.rdBufferSize = UART5_READ_BUFFER_SIZE_9BIT;
            uart5Obj.wrBufferSize = UART5_WRITE_BUFFER_SIZE_9BIT;
        }
        else
        {
            uart5Obj.rdBufferSize = UART5_READ_BUFFER_SIZE;
            uart5Obj.wrBufferSize = UART5_WRITE_BUFFER_SIZE;
        }

        status = true;
    }

    return status;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static inline bool UART5_RxPushByte(uint16_t rdByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    tempInIndex = uart5Obj.rdInIndex + 1;

    if (tempInIndex >= uart5Obj.rdBufferSize)
    {
        tempInIndex = 0;
    }

    if (tempInIndex == uart5Obj.rdOutIndex)
    {
        /* Queue is full - Report it to the application. Application gets a chance to free up space by reading data out from the RX ring buffer */
        if(uart5Obj.rdCallback != NULL)
        {
            uart5Obj.rdCallback(UART_EVENT_READ_BUFFER_FULL, uart5Obj.rdContext);

            /* Read the indices again in case application has freed up space in RX ring buffer */
            tempInIndex = uart5Obj.rdInIndex + 1;

            if (tempInIndex >= uart5Obj.rdBufferSize)
            {
                tempInIndex = 0;
            }
        }
    }

    /* Attempt to push the data into the ring buffer */
    if (tempInIndex != uart5Obj.rdOutIndex)
    {
        if (UART5_IS_9BIT_MODE_ENABLED())
        {
            ((uint16_t*)&UART5_ReadBuffer)[uart5Obj.rdInIndex] = rdByte;
        }
        else
        {
            UART5_ReadBuffer[uart5Obj.rdInIndex] = (uint8_t)rdByte;
        }

        uart5Obj.rdInIndex = tempInIndex;

        isSuccess = true;
    }
    else
    {
        /* Queue is full. Data will be lost. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART5_ReadNotificationSend(void)
{
    uint32_t nUnreadBytesAvailable;

    if (uart5Obj.isRdNotificationEnabled == true)
    {
        nUnreadBytesAvailable = UART5_ReadCountGet();

        if(uart5Obj.rdCallback != NULL)
        {
            if (uart5Obj.isRdNotifyPersistently == true)
            {
                if (nUnreadBytesAvailable >= uart5Obj.rdThreshold)
                {
                    uart5Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart5Obj.rdContext);
                }
            }
            else
            {
                if (nUnreadBytesAvailable == uart5Obj.rdThreshold)
                {
                    uart5Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart5Obj.rdContext);
                }
            }
        }
    }
}

size_t UART5_Read(uint8_t* pRdBuffer, const size_t size)
{
    size_t nBytesRead = 0;
    uint32_t rdOutIndex = 0;
    uint32_t rdInIndex = 0;

    /* Take a snapshot of indices to avoid creation of critical section */
    rdOutIndex = uart5Obj.rdOutIndex;
    rdInIndex = uart5Obj.rdInIndex;

    while (nBytesRead < size)
    {
        if (rdOutIndex != rdInIndex)
        {
            if (UART5_IS_9BIT_MODE_ENABLED())
            {
                ((uint16_t*)pRdBuffer)[nBytesRead++] = ((uint16_t*)&UART5_ReadBuffer)[rdOutIndex++];
            }
            else
            {
                pRdBuffer[nBytesRead++] = UART5_ReadBuffer[rdOutIndex++];
            }

            if (rdOutIndex >= uart5Obj.rdBufferSize)
            {
                rdOutIndex = 0;
            }
        }
        else
        {
            /* No more data available in the RX buffer */
            break;
        }
    }

    uart5Obj.rdOutIndex = rdOutIndex;

    return nBytesRead;
}

size_t UART5_ReadCountGet(void)
{
    size_t nUnreadBytesAvailable;
    uint32_t rdInIndex;
    uint32_t rdOutIndex;

    /* Take a snapshot of indices to avoid processing in critical section */
    rdInIndex = uart5Obj.rdInIndex;
    rdOutIndex = uart5Obj.rdOutIndex;

    if ( rdInIndex >=  rdOutIndex)
    {
        nUnreadBytesAvailable =  rdInIndex -  rdOutIndex;
    }
    else
    {
        nUnreadBytesAvailable =  (uart5Obj.rdBufferSize -  rdOutIndex) + rdInIndex;
    }

    return nUnreadBytesAvailable;
}

size_t UART5_ReadFreeBufferCountGet(void)
{
    return (uart5Obj.rdBufferSize - 1) - UART5_ReadCountGet();
}

size_t UART5_ReadBufferSizeGet(void)
{
    return (uart5Obj.rdBufferSize - 1);
}

bool UART5_ReadNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart5Obj.isRdNotificationEnabled;

    uart5Obj.isRdNotificationEnabled = isEnabled;

    uart5Obj.isRdNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART5_ReadThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0)
    {
        uart5Obj.rdThreshold = nBytesThreshold;
    }
}

void UART5_ReadCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart5Obj.rdCallback = callback;

    uart5Obj.rdContext = context;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static bool UART5_TxPullByte(uint16_t* pWrByte)
{
    bool isSuccess = false;
    uint32_t wrOutIndex = uart5Obj.wrOutIndex;
    uint32_t wrInIndex = uart5Obj.wrInIndex;

    if (wrOutIndex != wrInIndex)
    {
        if (UART5_IS_9BIT_MODE_ENABLED())
        {
            *pWrByte = ((uint16_t*)&UART5_WriteBuffer)[wrOutIndex++];
        }
        else
        {
            *pWrByte = UART5_WriteBuffer[wrOutIndex++];
        }

        if (wrOutIndex >= uart5Obj.wrBufferSize)
        {
            wrOutIndex = 0;
        }

        uart5Obj.wrOutIndex = wrOutIndex;

        isSuccess = true;
    }

    return isSuccess;
}

static inline bool UART5_TxPushByte(uint16_t wrByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    uint32_t wrOutIndex = uart5Obj.wrOutIndex;
    uint32_t wrInIndex = uart5Obj.wrInIndex;

    tempInIndex = wrInIndex + 1;

    if (tempInIndex >= uart5Obj.wrBufferSize)
    {
        tempInIndex = 0;
    }
    if (tempInIndex != wrOutIndex)
    {
        if (UART5_IS_9BIT_MODE_ENABLED())
        {
            ((uint16_t*)&UART5_WriteBuffer)[wrInIndex] = wrByte;
        }
        else
        {
            UART5_WriteBuffer[wrInIndex] = (uint8_t)wrByte;
        }

        uart5Obj.wrInIndex = tempInIndex;

        isSuccess = true;
    }
    else
    {
        /* Queue is full. Report Error. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART5_WriteNotificationSend(void)
{
    uint32_t nFreeWrBufferCount;

    if (uart5Obj.isWrNotificationEnabled == true)
    {
        nFreeWrBufferCount = UART5_WriteFreeBufferCountGet();

        if(uart5Obj.wrCallback != NULL)
        {
            if (uart5Obj.isWrNotifyPersistently == true)
            {
                if (nFreeWrBufferCount >= uart5Obj.wrThreshold)
                {
                    uart5Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart5Obj.wrContext);
                }
            }
            else
            {
                if (nFreeWrBufferCount == uart5Obj.wrThreshold)
                {
                    uart5Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart5Obj.wrContext);
                }
            }
        }
    }
}

static size_t UART5_WritePendingBytesGet(void)
{
    size_t nPendingTxBytes;

    /* Take a snapshot of indices to avoid processing in critical section */

    uint32_t wrOutIndex = uart5Obj.wrOutIndex;
    uint32_t wrInIndex = uart5Obj.wrInIndex;

    if ( wrInIndex >=  wrOutIndex)
    {
        nPendingTxBytes =  wrInIndex - wrOutIndex;
    }
    else
    {
        nPendingTxBytes =  (uart5Obj.wrBufferSize -  wrOutIndex) + wrInIndex;
    }

    return nPendingTxBytes;
}

size_t UART5_WriteCountGet(void)
{
    size_t nPendingTxBytes;

    nPendingTxBytes = UART5_WritePendingBytesGet();

    return nPendingTxBytes;
}

size_t UART5_Write(uint8_t* pWrBuffer, const size_t size )
{
    size_t nBytesWritten  = 0;

    while (nBytesWritten < size)
    {
        if (UART5_IS_9BIT_MODE_ENABLED())
        {
            if (UART5_TxPushByte(((uint16_t*)pWrBuffer)[nBytesWritten]) == true)
            {
                nBytesWritten++;
            }
            else
            {
                /* Queue is full, exit the loop */
                break;
            }
        }
        else
        {
            if (UART5_TxPushByte(pWrBuffer[nBytesWritten]) == true)
            {
                nBytesWritten++;
            }
            else
            {
                /* Queue is full, exit the loop */
                break;
            }
        }

    }

    /* Check if any data is pending for transmission */
    if (UART5_WritePendingBytesGet() > 0)
    {
        /* Enable TX interrupt as data is pending for transmission */
        UART5_TX_INT_ENABLE();
    }

    return nBytesWritten;
}

size_t UART5_WriteFreeBufferCountGet(void)
{
    return (uart5Obj.wrBufferSize - 1) - UART5_WriteCountGet();
}

size_t UART5_WriteBufferSizeGet(void)
{
    return (uart5Obj.wrBufferSize - 1);
}

bool UART5_WriteNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart5Obj.isWrNotificationEnabled;

    uart5Obj.isWrNotificationEnabled = isEnabled;

    uart5Obj.isWrNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART5_WriteThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0)
    {
        uart5Obj.wrThreshold = nBytesThreshold;
    }
}

void UART5_WriteCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart5Obj.wrCallback = callback;

    uart5Obj.wrContext = context;
}

UART_ERROR UART5_ErrorGet( void )
{
    UART_ERROR errors = uart5Obj.errors;

    uart5Obj.errors = UART_ERROR_NONE;

    /* All errors are cleared, but send the previous error state */
    return errors;
}

bool UART5_AutoBaudQuery( void )
{
    if(U5MODE & _U5MODE_ABAUD_MASK)
        return true;
    else
        return false;
}

void UART5_AutoBaudSet( bool enable )
{
    if( enable == true )
    {
        U5MODESET = _U5MODE_ABAUD_MASK;
    }

    /* Turning off ABAUD if it was on can lead to unpredictable behavior, so that
       direction of control is not allowed in this function.                      */
}

void UART5_FAULT_InterruptHandler (void)
{
    /* Save the error to be reported later */
    uart5Obj.errors = (UART_ERROR)(U5STA & (_U5STA_OERR_MASK | _U5STA_FERR_MASK | _U5STA_PERR_MASK));

    UART5_ErrorClear();

    /* Client must call UARTx_ErrorGet() function to clear the errors */
    if( uart5Obj.rdCallback != NULL )
    {
        uart5Obj.rdCallback(UART_EVENT_READ_ERROR, uart5Obj.rdContext);
    }
}

void UART5_RX_InterruptHandler (void)
{
    /* Keep reading until there is a character availabe in the RX FIFO */
    while((U5STA & _U5STA_URXDA_MASK) == _U5STA_URXDA_MASK)
    {
        if (UART5_RxPushByte( (uint16_t )(U5RXREG) ) == true)
        {
            UART5_ReadNotificationSend();
        }
        else
        {
            /* UART RX buffer is full */
        }
    }

    /* Clear UART5 RX Interrupt flag */
    IFS5CLR = _IFS5_U5RXIF_MASK;
}

void UART5_TX_InterruptHandler (void)
{
    uint16_t wrByte;

    /* Check if any data is pending for transmission */
    if (UART5_WritePendingBytesGet() > 0)
    {
        /* Keep writing to the TX FIFO as long as there is space */
        while(!(U5STA & _U5STA_UTXBF_MASK))
        {
            if (UART5_TxPullByte(&wrByte) == true)
            {
                if (UART5_IS_9BIT_MODE_ENABLED())
                {
                    U5TXREG = wrByte;
                }
                else
                {
                    U5TXREG = (uint8_t)wrByte;
                }

                /* Send notification */
                UART5_WriteNotificationSend();
            }
            else
            {
                /* Nothing to transmit. Disable the data register empty interrupt. */
                UART5_TX_INT_DISABLE();
                break;
            }
        }

        /* Clear UART5TX Interrupt flag */
        IFS5CLR = _IFS5_U5TXIF_MASK;
    }
    else
    {
        /* Nothing to transmit. Disable the data register empty interrupt. */
        UART5_TX_INT_DISABLE();

        /* Clear UART5TX Interrupt flag */
        IFS5CLR = _IFS5_U5TXIF_MASK;
    }
}

