/*
 * extFlash.c - External QSPI Flash Memory Driver Implementation
 *
 * This module implements a state machine-based driver for the MT25QL512ABB
 * QSPI flash memory device. It provides asynchronous operations for:
 * - Reading data from flash
 * - Writing (page programming) data to flash
 * - Erasing flash blocks/sectors
 *
 * The driver uses interrupt-driven callbacks to monitor operation completion
 * and manages state transitions through a simple state machine.
 *
 * Key Design:
 * - State machine pattern for async operations
 * - Callback-driven progress monitoring
 * - QPI (Quad SPI) mode support for faster transfers
 * - 256-byte page size matching device specifications
 *
 * Created on: Mar 3, 2025
 * Author: deploy
 *
 * @file extFlash.c
 * @brief External QSPI Flash Memory Driver
 */

#include "globVar.h"
#include "localDef.h"
#include "localEnum.h"
#include "localVar.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "mt25ql01gbb.h"
#include "extFlash.h"

/* ============================================================================
   Global Variables
   ============================================================================ */

/** @brief QSPI peripheral handle (defined by STM32 HAL) */
extern QSPI_HandleTypeDef hqspi;

/** @brief QSPI command configuration structure */
static QSPI_CommandTypeDef sCommand;

/** @brief External flash device state and control structure */
static extFlash mtFlash;

/* ============================================================================
   Public Functions
   ============================================================================ */

/**
 * @brief Initialize external flash memory device
 *
 * Performs complete initialization of the MT25QL512ABB QSPI flash device:
 * 1. Release hardware reset
 * 2. Configure QSPI command structure with timing parameters
 * 3. Read device ID in SPI mode (verify communication)
 * 4. Read status and configuration registers
 * 5. Enter QPI (Quad SPI) mode for faster operations
 * 6. Switch to 4-byte address mode for larger memory space
 * 7. Verify successful mode switching
 * 8. Initialize state machine to ready state
 *
 * @retval void
 *
 * @warning QSPI peripheral must be pre-initialized by HAL_QSPI_Init()
 * @warning GPIO reset pin must be configured as output
 * @warning Function may take 100+ ms due to mode transitions
 *
 * @note Should be called once at system startup
 * @note TODO: Add return status and error handling for verification failures
 * @note TODO: Log device ID to verify correct device is present
 */
void extFlashInit(void)
{
    /* Release device from reset */
    HAL_GPIO_WritePin(QSPI_A_RST_GPIO_Port, QSPI_A_RST_Pin, GPIO_PIN_SET);

    /* Initialize state machine to initialization state */
    mtFlash.exFlashStage = exFlashInit;

    /* Configure QSPI command settings for this device */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;      /* 1-line instruction mode */
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;         /* 3-byte address initially */
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;    /* No alternate bytes */
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;        /* Single data rate mode */
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;    /* DDR timing (unused) */
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;     /* Send instruction every command */

    /* === SPI Mode Communication === */
    /* Read device ID in SPI mode to verify communication */
    MT25QL512ABB_ReadID(&hqspi, MT25QL512ABB_SPI_MODE, mtFlash.exFlashID, QSPI_DUALFLASH_DISABLE);
    
    /* Read status register to check device state */
    MT25QL512ABB_ReadStatusRegister(&hqspi, MT25QL512ABB_SPI_MODE, 
                                     QSPI_DUALFLASH_DISABLE, mtFlash.extFlashStatFlReg);
    
    /* Read enhanced volatile configuration register */
    MT25QL512ABB_ReadEnhancedVolCfgRegister(&hqspi, MT25QL512ABB_SPI_MODE, 
                                             QSPI_DUALFLASH_DISABLE, mtFlash.extFlashEnhVolReg);

    /* === Transition to QPI Mode === */
    /* Switch device from SPI mode to QPI (Quad SPI) mode for faster transfers */
    MT25QL512ABB_EnterQPIMode(&hqspi);

    /* Verify mode switch - read status register in QPI mode */
    MT25QL512ABB_ReadStatusRegister(&hqspi, MT25QL512ABB_QPI_MODE, 
                                     QSPI_DUALFLASH_DISABLE, mtFlash.extFlashStatFlReg);
    
    /* Verify enhanced config in QPI mode */
    MT25QL512ABB_ReadEnhancedVolCfgRegister(&hqspi, MT25QL512ABB_QPI_MODE, 
                                             QSPI_DUALFLASH_DISABLE, mtFlash.extFlashEnhVolReg);

    /* === Switch to 4-Byte Addressing === */
    /* Clear ID array before re-reading in QPI mode */
    mtFlash.exFlashID[0] = 0;
    mtFlash.exFlashID[1] = 0;
    mtFlash.exFlashID[2] = 0;

    /* Re-read device ID in QPI mode to confirm successful mode switch */
    MT25QL512ABB_ReadID(&hqspi, MT25QL512ABB_QPI_MODE, mtFlash.exFlashID, QSPI_DUALFLASH_DISABLE);

    /* Enable 4-byte address mode for devices with address space > 16MB */
    MT25QL512ABB_Enter4BytesAddressMode(&hqspi, MT25QL512ABB_QPI_MODE);

    /* Verify 4-byte mode is active - read status register */
    MT25QL512ABB_ReadStatusRegister(&hqspi, MT25QL512ABB_QPI_MODE, 
                                     QSPI_DUALFLASH_DISABLE, mtFlash.extFlashStatFlReg);
    
    /* Verify enhanced config with 4-byte addressing */
    MT25QL512ABB_ReadEnhancedVolCfgRegister(&hqspi, MT25QL512ABB_QPI_MODE, 
                                             QSPI_DUALFLASH_DISABLE, mtFlash.extFlashEnhVolReg);

    /* === Initialization Complete === */
    /* Clear status match flag and set state to ready */
    mtFlash.exFlashStatusMatchFlg = 0;
    mtFlash.exFlashStage = exFlashReady;
}

/**
 * @brief Process external flash state machine
 *
 * This is the main state machine handler that should be called periodically
 * to progress flash operations. It manages state transitions based on:
 * - Hardware callback flags (command complete, receive complete, etc.)
 * - Current operation status
 * - Automatic polling results
 *
 * State Flow Summary:
 * - **Erase:** Init → Erase → EraseWait → EraseEnd → Ready
 * - **Read:**  Init → Read → ReadEnd → Ready
 * - **Write:** Init → Write → WriteWait → WriteEnd → Ready
 *
 * @retval void
 *
 * @warning Must be called frequently (10-100 Hz recommended)
 * @warning No timeout protection - system can hang if flash doesn't respond
 * @warning Large switch statement (130+ lines) - consider refactoring
 *
 * @note Non-blocking function; returns immediately
 * @note Callback flags are set by interrupt handlers
 * @note State transitions occur based on flag conditions
 */
void extFlashFunc(void)
{
    switch(mtFlash.exFlashStage)
    {
        /* ====== ERASE OPERATION SEQUENCE ====== */
        case exFlashErase:
        {
            /* Clear command completion flag for this operation */
            mtFlash.exFlashCmdCpltFlg = 0;

            /* Enable write operations on the device */
            MT25QL512ABB_WriteEnable(&hqspi, MT25QL512ABB_QPI_MODE, QSPI_DUALFLASH_DISABLE);
            
            /* Read status register to confirm write enable */
            MT25QL512ABB_ReadStatusRegister(&hqspi, MT25QL512ABB_QPI_MODE, 
                                             QSPI_DUALFLASH_DISABLE, mtFlash.extFlashStatFlReg);

            /* Issue erase command based on erase size selection */
            if(mtFlash.exFlashErsSize == exFlashAll) {
                /* Chip erase - erases entire device */
                MT25QL512ABB_ChipErase(&hqspi, MT25QL512ABB_QPI_MODE, MT25QL512ABB_4BYTES_SIZE);
            } else {
                /* Block erase - erases 4K/32K/64K block at specified address */
                MT25QL512ABB_BlockErase(&hqspi, MT25QL512ABB_QPI_MODE, MT25QL512ABB_4BYTES_SIZE, 
                                        mtFlash.exFlashAddress, mtFlash.exFlashErsSize);
            }

            /* TODO: Check HAL return status and handle errors */
            /* TODO: Add timeout protection */

            /* Transition to wait state for erase completion */
            mtFlash.exFlashStage = exFlashEraseWait;
            break;
        }

        case exFlashEraseWait:
        {
            /* Wait for erase command to complete on device */
            if(mtFlash.exFlashCmdCpltFlg != 0)
            {
                /* Clear flags for next phase */
                mtFlash.exFlashCmdCpltFlg = 0;
                mtFlash.exFlashStatusMatchFlg = 0;

                /* Start automatic polling to wait for device to finish erasing */
                MT25QL512ABB_AutoPollingMemReady(&hqspi, MT25QL512ABB_QPI_MODE, QSPI_DUALFLASH_DISABLE);
                
                /* Transition to end state */
                mtFlash.exFlashStage = exFlashEraseEnd;
            }
            
            /* Continue polling status register while waiting */
            MT25QL512ABB_ReadStatusRegister(&hqspi, MT25QL512ABB_QPI_MODE, 
                                             QSPI_DUALFLASH_DISABLE, mtFlash.extFlashStatFlReg);
            break;
        }

        case exFlashEraseEnd:
        {
            /* Check if polling detected device is ready (erase complete) */
            if(mtFlash.exFlashStatusMatchFlg != 0)
            {
                /* Clear flags and mark operation complete */
                mtFlash.exFlashStatusMatchFlg = 0;
                mtFlash.exFlashTxCpltFlg = 0;
                mtFlash.exFlashStage = exFlashReady;
                mtFlash.exFlashCmplt = exFlErase;  /* Record that erase completed */
            }
            
            /* Continue checking status until ready */
            MT25QL512ABB_ReadStatusRegister(&hqspi, MT25QL512ABB_QPI_MODE, 
                                             QSPI_DUALFLASH_DISABLE, mtFlash.extFlashStatFlReg);
            break;
        }

        /* ====== READ OPERATION SEQUENCE ====== */
        case exFlashRead:
        {
            /* Clear receive completion flag */
            mtFlash.exFlashRxCpltFlg = 0;
            
            /* Issue read command (DTR = Double Transfer Rate mode) */
            MT25QL512ABB_ReadDTR(&hqspi, MT25QL512ABB_QPI_MODE, MT25QL512ABB_4BYTES_SIZE,
                                 mtFlash.exFlashRxData, mtFlash.exFlashAddress, mtFlash.exFlashDataSize);

            /* TODO: Add error handling for read failures */

            /* Transition to wait for completion state */
            mtFlash.exFlashStage = exFlashReadEnd;
            break;
        }

        case exFlashReadEnd:
        {
            /* Wait for read data to be received via DMA */
            if(mtFlash.exFlashRxCpltFlg != 0)
            {
                /* Clear flag and mark operation complete */
                mtFlash.exFlashRxCpltFlg = 0;
                mtFlash.exFlashStage = exFlashReady;
                mtFlash.exFlashCmplt = exFlRead;  /* Record that read completed */
                
                /* Data is now available in mtFlash.exFlashRxData[256] */
            }
            break;
        }

        /* ====== WRITE OPERATION SEQUENCE ====== */
        case exFlashWrite:
        {
            /* Enable write operations before page program */
            MT25QL512ABB_WriteEnable(&hqspi, MT25QL512ABB_QPI_MODE, QSPI_DUALFLASH_DISABLE);
            
            /* Clear transmit completion flag */
            mtFlash.exFlashTxCpltFlg = 0;
            
            /* Issue page program command to write data to flash */
            MT25QL512ABB_PageProgram(&hqspi, MT25QL512ABB_QPI_MODE, MT25QL512ABB_4BYTES_SIZE,
                                     mtFlash.exFlashTxData, mtFlash.exFlashAddress, mtFlash.exFlashDataSize);

            /* TODO: Add error handling for write failures */

            /* Transition to wait for write completion state */
            mtFlash.exFlashStage = exFlashWriteWait;
            break;
        }

        case exFlashWriteWait:
        {
            /* Wait for page program command to transmit */
            if(mtFlash.exFlashTxCpltFlg != 0)
            {
                /* Clear transmit buffer after successful transmission */
                memset(&mtFlash.exFlashTxData, 0, EXT_FL_PG_SIZE);
                
                /* Clear flag for next phase */
                mtFlash.exFlashTxCpltFlg = 0;
                
                /* Start automatic polling to wait for device to finish writing */
                MT25QL512ABB_AutoPollingMemReady(&hqspi, MT25QL512ABB_QPI_MODE, QSPI_DUALFLASH_DISABLE);
                
                /* Transition to end state */
                mtFlash.exFlashStage = exFlashWriteEnd;
            }
            break;
        }

        case exFlashWriteEnd:
        {
            /* Check if polling detected device is ready (write complete) */
            if(mtFlash.exFlashStatusMatchFlg != 0)
            {
                /* Clear flag and mark operation complete */
                mtFlash.exFlashStatusMatchFlg = 0;
                mtFlash.exFlashStage = exFlashReady;
                mtFlash.exFlashCmplt = exFlWrite;  /* Record that write completed */
            }
            break;
        }

        /* ====== IDLE AND ERROR STATES ====== */
        case exFlashReady:
        {
            /* Device is ready for next command; nothing to do */
            break;
        }

        case exFlashError:
        {
            /* Error state - recovery requires manual intervention */
            /* TODO: Implement error recovery mechanism */
            break;
        }

        default:
        {
            /* Unexpected state - should never reach here */
            break;
        }
    }
}

/**
 * @brief Get current state of flash state machine
 *
 * Non-blocking query to determine if flash device is busy or ready.
 * Useful for checking operation completion before issuing new commands.
 *
 * @retval eFlSt Current state (exFlashReady, exFlashErase, exFlashWrite, etc.)
 *
 * @note Thread-safe read of volatile state variable
 * @note Can be called from anywhere without side effects
 *
 * Example:
 * @code
 * // Initiate read operation
 * extFlashCmd(256, 0x1000, exFlRead);
 *
 * // Poll until completion
 * while(extFlashStatGet() != exFlashReady) {
 *     HAL_Delay(10);
 * }
 *
 * // Data now available in mtFlash.exFlashRxData
 * @endcode
 */
eFlSt extFlashStatGet(void)
{
    return (eFlSt)mtFlash.exFlashStage;
}

/**
 * @brief Get the most recently completed flash command
 *
 * Returns which command (if any) was completed since last query.
 * Use after extFlashStatGet() returns exFlashReady to verify
 * the expected operation succeeded.
 *
 * @retval eFlCmd Type of completed command (exFlRead, exFlWrite, exFlErase, etc.)
 *
 * @note Value is NOT automatically cleared; reflects last command only
 * @note Should be checked immediately after state transitions to Ready
 *
 * Example:
 * @code
 * if(extFlashStatGet() == exFlashReady) {
 *     eFlCmd lastCmd = extFlashCmdGet();
 *     if(lastCmd == exFlRead) {
 *         // Process data from exFlashRxData
 *         processImageData(mtFlash.exFlashRxData, 256);
 *     }
 * }
 * @endcode
 */
eFlCmd extFlashCmdGet(void)
{
    return mtFlash.exFlashCmplt;
}

/**
 * @brief Initiate a flash operation (erase, read, or write)
 *
 * Issues a command to the flash state machine to perform the specified
 * operation. The operation executes asynchronously; monitor progress with
 * extFlashStatGet() and extFlashCmdGet().
 *
 * @param[in] eFlSize Size of data to read/write in bytes
 *                    (Ignored for erase operations)
 * @param[in] eFlAddr Starting address in flash for the operation
 *                    (Ignored for chip erase)
 * @param[in] FlCmd Command type: exFlEraseAll, exFlErase, exFlRead, or exFlWrite
 *
 * @retval void
 *
 * Command Descriptions:
 * - **exFlEraseAll**: Erase entire device (mass erase)
 *   - Parameters: eFlAddr, eFlSize ignored
 *   - Time: ~30 seconds
 * - **exFlErase**: Erase 4KB sector at specified address
 *   - Parameters: eFlAddr required, eFlSize ignored
 *   - Time: ~100 ms
 * - **exFlRead**: Read up to 256 bytes from specified address
 *   - Parameters: eFlAddr, eFlSize required
 *   - Time: ~5 ms
 *   - Result: Data in mtFlash.exFlashRxData[]
 * - **exFlWrite**: Write up to 256 bytes to specified address
 *   - Parameters: eFlAddr, eFlSize required
 *   - Data source: mtFlash.exFlashTxData[]
 *   - Time: ~5 ms
 *
 * @warning State machine must be in exFlashReady before calling
 * @warning For write operations, load mtFlash.exFlashTxData before calling
 * @warning eFlSize must not exceed EXT_FL_PG_SIZE (256 bytes)
 * @warning eFlAddr must be aligned to block size for erase operations
 *
 * Example Usage:
 * @code
 * // Read 256 bytes from address 0x1000
 * if(extFlashStatGet() == exFlashReady) {
 *     extFlashCmd(256, 0x1000, exFlRead);
 * }
 *
 * // Write 256 bytes at address 0x2000
 * if(extFlashStatGet() == exFlashReady) {
 *     memcpy(mtFlash.exFlashTxData, myData, 256);
 *     extFlashCmd(256, 0x2000, exFlWrite);
 * }
 *
 * // Erase 4KB sector at 0x3000
 * if(extFlashStatGet() == exFlashReady) {
 *     extFlashCmd(0, 0x3000, exFlErase);
 * }
 * @endcode
 */
void extFlashCmd(uint32_t eFlSize, uint32_t eFlAddr, eFlCmd FlCmd)
{
    switch(FlCmd)
    {
        case exFlEraseAll:
        {
            /* Prepare for chip erase (entire device) */
            mtFlash.exFlashErsSize = exFlashAll;
            mtFlash.exFlashStage = exFlashErase;
            break;
        }

        case exFlErase:
        {
            /* Prepare for block erase (4KB default) */
            mtFlash.exFlashErsSize = exFlash4K;
            mtFlash.exFlashAddress = eFlAddr;
            mtFlash.exFlashStage = exFlashErase;
            break;
        }

        case exFlRead:
        {
            /* Prepare for read operation */
            /* Clear receive buffer to ensure clean data */
            memset(mtFlash.exFlashRxData, 0, sizeof(mtFlash.exFlashRxData));
            
            /* Set operation parameters */
            mtFlash.exFlashDataSize = eFlSize;
            mtFlash.exFlashAddress = eFlAddr;
            mtFlash.exFlashStage = exFlashRead;
            break;
        }

        case exFlWrite:
        {
            /* Prepare for write operation */
            /* Set operation parameters */
            mtFlash.exFlashDataSize = eFlSize;
            mtFlash.exFlashAddress = eFlAddr;
            mtFlash.exFlashStage = exFlashWrite;
            
            /* Note: Data must already be in mtFlash.exFlashTxData[] */
            break;
        }

        default:
        {
            /* Invalid command - do nothing */
            break;
        }
    }
}

/**
 * @brief Format flash read data as debug/diagnostic message
 *
 * Packages the most recent flash read data into an ASCII-formatted message
 * suitable for debug output or transmission over the debug interface.
 *
 * Message Format:
 * @code
 * "<address> <error_name> <net_addr> <object_name> <object_num> is <raw_256_bytes>\r\n"
 * @endcode
 *
 * @param[in,out] debugParse Pointer to debug structure containing:
 *                           - txDumpBuffDbg: Output buffer
 *                           - txDataAsciiLen: Current buffer size
 * @param[in] systemObj Pointer to system state object:
 *                      - communicationData: Network addressing info
 *                      - intRequestCnt: Request counter
 * @param[in] constName Pointer to constant name lookup table:
 *                      - errorName[]: Error string names
 *                      - objectName[]: Object type string names
 *
 * @retval void
 *
 * @warning Assumes sufficient buffer space in debugParse->txDumpBuffDbg
 * @warning ISSUE: Uses strlen() on potentially uninitialized buffer (BUG!)
 * @warning Should validate all pointers before dereferencing
 * @note This function couples flash driver to debug protocol (consider separating)
 * @note Should calculate buffer size explicitly instead of using strlen()
 */
void extFlashRead(void *debugParse, void *systemObj, void *constName)
{
    /* Cast opaque pointers to actual types */
    /* Note: These casts are unsafe without proper null checks */
    dbg *pDebug = (dbg *)debugParse;
    syst *pSystem = (syst *)systemObj;
    globCnst *pConst = (globCnst *)constName;

    /* Initialize output buffer */
    pDebug->txDataAsciiLen = 0;
    
    /* FIXME: BUG - strlen() on uninitialized buffer is undefined behavior */
    /* Should use sizeof(txDumpBuffDbg) instead */
    memset((char*)pDebug->txDumpBuffDbg, 0, strlen((char*)pDebug->txDumpBuffDbg));

    /* Format debug message with addressing information */
    sprintf((char*)pDebug->txDumpBuffDbg, "%d %s %d %s %d is ",
            MANAGEMENT_NET_ADDR_NUMBER,
            pConst->errorName[dbgParseErrResponse],
            pSystem->communicationData[pSystem->intRequestCnt % DBG_REQUEST_SEQ_MAX].netAddr,
            pConst->objectName[pSystem->communicationData[pSystem->intRequestCnt % DBG_REQUEST_SEQ_MAX].objectNumName - 1],
            pSystem->communicationData[pSystem->intRequestCnt % DBG_REQUEST_SEQ_MAX].numberOfObj);

    /* Calculate length of ASCII prefix */
    pDebug->txDataAsciiLen = (uint16_t)(strlen((char*)pDebug->txDumpBuffDbg));

    /* Append raw flash data (256 bytes) */
    memcpy(&pDebug->txDumpBuffDbg[pDebug->txDataAsciiLen], &mtFlash.exFlashRxData, EXT_FL_PG_SIZE);
    pDebug->txDataAsciiLen += EXT_FL_PG_SIZE;

    /* Append CRLF line terminator */
    pDebug->txDumpBuffDbg[pDebug->txDataAsciiLen] = '\r';
    pDebug->txDataAsciiLen++;
    pDebug->txDumpBuffDbg[pDebug->txDataAsciiLen] = '\n';
    pDebug->txDataAsciiLen++;
}

/**
 * @brief Copy flash read data into image data structure
 *
 * Helper function that copies the most recent flash read buffer
 * (exFlashRxData) into an image data structure. Typically called
 * after an exFlRead command completes to process the received data.
 *
 * @param[in,out] imgPDat Pointer to image data structure
 *                        (Will be cast to imgData internally)
 *
 * @retval void
 *
 * @warning ISSUE: Null pointer check is wrong: if(&ptr->member != NULL)
 * @warning Should check if(imgPDat != NULL) before dereferencing
 * @warning Blindly copies EXT_FL_PG_SIZE (256 bytes) - verify buffer size
 * @warning No validation that pointer is valid imgData structure
 *
 * @note Used by image transfer module to get flash data
 * @note Should add proper error handling
 */
void extFlashReadGui(uint32_t *imgPDat)
{
    /* Cast pointer to image data structure */
    imgData *tpmDat = (imgData*)imgPDat;

    /* ISSUE: This NULL check is incorrect!
     * &tpmDat->imgLineBuff is address of member, never NULL if tpmDat is valid
     * Should check: if(tpmDat != NULL) instead */
    if(&tpmDat->imgLineBuff != NULL)
    {
        /* Copy 256 bytes from flash read buffer to image buffer */
        memcpy(&tpmDat->imgLineBuff, &mtFlash.exFlashRxData, EXT_FL_PG_SIZE);
    }
}

/**
 * @brief Copy image data to flash write buffer and initiate write
 *
 * Helper function that:
 * 1. Validates image data pointer
 * 2. Clears flash write buffer
 * 3. Copies image data to flash write buffer
 * 4. Updates flash write address from image structure
 * 5. Initiates page program (write) operation
 *
 * Typically called by image transfer module to write a page of image data.
 *
 * @param[in,out] imgPDat Pointer to image data structure containing:
 *                        - imgLineBuff: Data to write (256 bytes)
 *                        - imgAddr: Flash destination address
 *
 * @retval void
 *
 * @warning ISSUE: Null pointer check is wrong: if(&ptr->member != NULL)
 * @warning Should check if(imgPDat != NULL) before dereferencing
 * @warning Does not validate flash is ready before initiating write
 * @warning No error handling if write command fails
 *
 * @note Should check extFlashStatGet() == exFlashReady before calling
 * @note Write data must be in imgLineBuff before calling
 */
void extFlashWriteGui(uint32_t *imgPDat)
{
    /* Cast pointer to image data structure */
    imgData *tpmDat = (imgData*)imgPDat;

    /* ISSUE: This NULL check is incorrect!
     * &tpmDat->imgLineBuff is address of member, never NULL if tpmDat is valid
     * Should check: if(tpmDat != NULL) instead */
    if(&tpmDat->imgLineBuff != NULL)
    {
        /* Clear flash write buffer before loading new data */
        memset(&mtFlash.exFlashTxData, 0, EXT_FL_PG_SIZE);
        
        /* Copy image data to flash write buffer */
        memcpy(&mtFlash.exFlashTxData, &tpmDat->imgLineBuff, EXT_FL_PG_SIZE);
        
        /* Update write address from image structure */
        mtFlash.exFlashAddress = tpmDat->imgAddr;
        
        /* Initiate page program (write) operation */
        extFlashCmd(EXT_FL_PG_SIZE, mtFlash.exFlashAddress, exFlWrite);
    }
}

/* ============================================================================
   Interrupt Callback Functions
   
   These functions are called by the QSPI HAL during interrupt service.
   They set flags that are monitored by extFlashFunc() to drive state
   machine progression.
   ============================================================================ */

/**
 * @brief QSPI command completion callback
 *
 * Called by HAL_QSPI_CmdCpltCallback when a QSPI command
 * (write enable, status register read, etc.) completes.
 * Sets flag for state machine to proceed to next phase.
 *
 * @param[in] hqspi QSPI peripheral handle
 *                  (Required by HAL callback signature, unused here)
 *
 * @retval void
 *
 * @warning Called from interrupt context - keep very brief
 * @warning ISSUE: Flag incremented without overflow protection
 * @warning Should use atomic operation or set to 1 instead of increment
 * @note Flag is cleared by state machine when processed
 * @note Called frequently during erase/write sequences
 */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* ISSUE: Incrementing flag without overflow protection
     * If callbacks occur faster than state machine processes,
     * flag could overflow (uint8_t max = 255)
     * FIX: Should set to 1 instead of increment */
    mtFlash.exFlashCmdCpltFlg++;
}

/**
 * @brief QSPI receive complete callback
 *
 * Called by HAL_QSPI_RxCpltCallback when read data reception
 * completes. Sets flag for state machine to transition from
 * exFlashReadEnd state back to exFlashReady.
 *
 * @param[in] hqspi QSPI peripheral handle (unused)
 *
 * @retval void
 *
 * @warning Called from interrupt context - keep very brief
 * @warning ISSUE: Flag incremented without overflow protection
 * @warning Data is already in exFlashRxData[] at this point
 * @note This callback signals DMA reception is complete
 */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* ISSUE: Flag overflow risk - increment without protection */
    mtFlash.exFlashRxCpltFlg++;
}

/**
 * @brief QSPI transmit complete callback
 *
 * Called by HAL_QSPI_TxCpltCallback when transmit (write data)
 * completes. Sets flag for state machine to start monitoring
 * device ready status via automatic polling.
 *
 * @param[in] hqspi QSPI peripheral handle (unused)
 *
 * @retval void
 *
 * @warning Called from interrupt context - keep very brief
 * @warning ISSUE: Flag incremented without overflow protection
 * @note Device may still be busy writing; polling checks actual status
 * @note Only indicates DMA transmission is complete, not flash write
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* ISSUE: Flag overflow risk - increment without protection */
    mtFlash.exFlashTxCpltFlg++;
}

/**
 * @brief QSPI automatic polling status match callback
 *
 * Called by HAL_QSPI_StatusMatchCallback when automatic polling
 * detects the device status register matches the expected value
 * (indicating operation completion). Signals to state machine that
 * erase/write operation is fully complete and device is ready.
 *
 * @param[in] hqspi QSPI peripheral handle (unused)
 *
 * @retval void
 *
 * @warning Called from interrupt context - keep very brief
 * @warning ISSUE: Flag incremented without overflow protection
 * @note Automatic polling mode continuously checks status until match
 * @note Indicates device is ready for next command
 * @note Called after exFlash.exFlashStatusMatchFlg == 0 initially
 */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
    /* ISSUE: Flag overflow risk - increment without protection */
    mtFlash.exFlashStatusMatchFlg++;
}

