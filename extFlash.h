/*
 * extFlash.h - External QSPI Flash Memory Driver
 *
 * This module provides an interface to the MT25QL512ABB QSPI flash memory device.
 * It implements a state machine-based approach for asynchronous flash operations
 * (read, write, erase) with support for both SPI and QPI modes.
 *
 * Key Features:
 * - State machine-based async operations
 * - Support for QPI (Quad SPI) mode for faster transfers
 * - 256-byte page size and multi-block erase operations
 * - Interrupt-driven callbacks for operation completion
 *
 * Usage:
 *   1. Call extFlashInit() at startup
 *   2. Issue commands via extFlashCmd()
 *   3. Monitor state with extFlashStatGet()
 *   4. Poll status or wait for callbacks
 *   5. Retrieve results via extFlashCmdGet()
 *
 * Created on: Mar 3, 2025
 * Author: deploy
 */

#ifndef INC_EXTFLASH_H_
#define INC_EXTFLASH_H_

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
   Configuration Constants
   ============================================================================ */

/** @brief Page size of external flash memory (must match device datasheet) */
#define EXT_FL_PG_SIZE      256UL

/** @brief Device ID size (Manufacturer ID + Device Type + Capacity) */
#define EXT_FL_ID_SIZE      3

/* ============================================================================
   Enumeration Types
   ============================================================================ */

/**
 * @enum exflashState
 * @brief State enumeration for external flash state machine
 * 
 * The flash driver operates as a state machine with the following flow:
 * - Erase: exFlashInit → exFlashErase → exFlashEraseWait → exFlashEraseEnd → exFlashReady
 * - Read:  exFlashInit → exFlashRead → exFlashReadEnd → exFlashReady
 * - Write: exFlashInit → exFlashWrite → exFlashWriteWait → exFlashWriteEnd → exFlashReady
 */
typedef enum exflashState {
    exFlashInit = 0,        /**< Initialization state (transient) */
    exFlashErase,           /**< Erase operation started */
    exFlashEraseWait,       /**< Waiting for erase command completion */
    exFlashEraseEnd,        /**< Waiting for memory to be ready after erase */
    exFlashRead,            /**< Read operation started */
    exFlashReadEnd,         /**< Waiting for read data reception */
    exFlashWrite,           /**< Write operation started */
    exFlashWriteWait,       /**< Waiting for write command completion */
    exFlashWriteEnd,        /**< Waiting for memory to be ready after write */
    exFlashReady,           /**< Idle, ready for next command */
    exFlashError,           /**< Error state (recovery required) */
} eFlSt;

/**
 * @enum exflashCmd
 * @brief Command types for external flash operations
 * 
 * These commands are issued via extFlashCmd() to initiate flash operations.
 * The status can be monitored via extFlashStatGet() and extFlashCmdGet().
 */
typedef enum exflashCmd {
    exFlEraseAll = 0,   /**< Erase entire device (mass erase) */
    exFlErase,          /**< Erase a 4KB block at specified address */
    exFlRead,           /**< Read data from specified address */
    exFlWrite,          /**< Write data to specified address (page program) */
} eFlCmd;

/**
 * @enum exFlashEraseSize
 * @brief Erase block size options
 * 
 * Defines the granularity of erase operations. Different block sizes may
 * be required depending on application needs and memory layout.
 */
enum exFlashEraseSize {
    exFlash4K = 0,      /**< 4 KB sector erase */
    exFlash32K,         /**< 32 KB block erase */
    exFlash64K,         /**< 64 KB block erase */
    exFlashAll,         /**< Entire chip erase (mass erase) */
};

/* ============================================================================
   Data Structures
   ============================================================================ */

/**
 * @struct exflsh
 * @brief External flash device state and control structure
 * 
 * This structure maintains all state information for the flash device,
 * including operation status, data buffers, and configuration.
 * 
 * @note This structure should be treated as internal to the driver;
 *       external modules should use the public API functions instead.
 */
typedef struct exflsh {
    /* Callback Flags - Set by interrupt handlers, cleared by state machine */
    volatile uint8_t exFlashRxCpltFlg;       /**< RX complete flag (interrupt set) */
    volatile uint8_t exFlashTxCpltFlg;       /**< TX complete flag (interrupt set) */
    volatile uint8_t exFlashCmdCpltFlg;      /**< Command complete flag (interrupt set) */
    volatile uint8_t exFlashStatusMatchFlg;  /**< Status match flag (polling match) */
    
    /* Register Read Values */
    uint8_t extFlashStatFlReg[2];            /**< Status register values [Flag, Status] */
    uint8_t extFlashEnhVolReg[2];            /**< Enhanced volatile config register */
    
    /* State Machine */
    uint8_t exFlashStage;                    /**< Current state (eFlSt enum value) */
    volatile eFlCmd exFlashCmplt;            /**< Last completed command type */
    
    /* Operation Parameters */
    uint32_t exFlashErsSize;                 /**< Erase block size for current operation */
    uint32_t exFlashDataSize;                /**< Data size for current operation (bytes) */
    uint32_t exFlashAddress;                 /**< Flash address for current operation */
    
    /* Device Identification */
    uint8_t exFlashID[EXT_FL_ID_SIZE];       /**< Device ID: [ManufID, DevType, Capacity] */
    
    /* Data Buffers */
    uint8_t exFlashTxData[EXT_FL_PG_SIZE];   /**< Transmit buffer (write data) */
    uint8_t exFlashRxData[EXT_FL_PG_SIZE];   /**< Receive buffer (read data) */
} extFlash;

/* ============================================================================
   Public Function Declarations
   ============================================================================ */

/**
 * @brief Initialize external flash memory and configure QSPI interface
 * 
 * Performs the following initialization sequence:
 * 1. Release reset pin to device
 * 2. Configure QSPI command structure with timing parameters
 * 3. Read device ID (verify communication)
 * 4. Read status and configuration registers
 * 5. Switch device from SPI mode to QPI (Quad SPI) mode
 * 6. Enable 4-byte address mode for larger memory address space
 * 7. Initialize state machine to ready state
 * 
 * @warning Must be called before any other flash operations
 * @warning Assumes QSPI peripheral is pre-configured by HAL
 * @warning GPIO reset pin must be configured for output
 * 
 * @retval void
 * 
 * @note May take 100+ ms due to mode switching and register reads
 * @note Does not validate device presence (should add ID verification)
 */
void extFlashInit(void);

/**
 * @brief Process external flash state machine
 * 
 * This is the main state machine function that should be called periodically
 * (typically in main loop or timer ISR) to progress flash operations.
 * 
 * Handles all state transitions and manages:
 * - Command sequencing (write enable, erase, read, write, status polling)
 * - Waiting for operation completion via callback flags
 * - Transitioning to ready state when operations complete
 * 
 * @retval void
 * 
 * @note Non-blocking; will not wait for operations to complete
 * @note Should be called frequently (10-100 Hz recommended)
 * @note Callback flags are set by hardware interrupt handlers
 * 
 * @warning Large switch statement - consider refactoring into modular functions
 * @warning No timeout protection - system could hang if flash doesn't respond
 */
void extFlashFunc(void);

/**
 * @brief Get current flash state machine state
 * 
 * Non-blocking query to determine the current state of the flash
 * state machine. Useful for checking if a previous operation has completed.
 * 
 * @retval eFlSt Current state value (see exflashState enum)
 * 
 * Example usage:
 * @code
 * extFlashCmd(256, 0x1000, exFlRead);  // Initiate read
 * while(extFlashStatGet() != exFlashReady) {
 *     // Wait for operation
 *     HAL_Delay(10);
 * }
 * // Data now available in mtFlash.exFlashRxData
 * @endcode
 * 
 * @note Thread-safe read of volatile state
 */
eFlSt extFlashStatGet(void);

/**
 * @brief Get the completion status of the last flash command
 * 
 * Returns which command (if any) was most recently completed.
 * Use this after extFlashStatGet() returns exFlashReady to determine
 * if the expected operation completed successfully.
 * 
 * @retval eFlCmd Type of command that just completed (see exflashCmd enum)
 * 
 * Example usage:
 * @code
 * if(extFlashStatGet() == exFlashReady) {
 *     eFlCmd lastCmd = extFlashCmdGet();
 *     if(lastCmd == exFlRead) {
 *         // Process data from exFlashRxData
 *     }
 * }
 * @endcode
 * 
 * @note Value is not cleared automatically; will reflect last command
 * @note Compare against exFlCmd enum values
 */
eFlCmd extFlashCmdGet(void);

/**
 * @brief Initiate a flash operation (erase, read, or write)
 * 
 * Issues a command to the flash state machine to perform the specified
 * operation. The operation is asynchronous; use extFlashStatGet() to
 * monitor progress.
 * 
 * @param[in] eFlSize Size of data to read/write (bytes), or unused for erase
 * @param[in] eFlAddr Starting address in flash for the operation
 * @param[in] FlCmd Command type (erase, read, write, or erase all)
 * 
 * @retval void
 * 
 * Behavior by command type:
 * - exFlEraseAll: Erases entire device (eFlAddr and eFlSize ignored)
 * - exFlErase: Erases 4KB block at eFlAddr (eFlSize ignored)
 * - exFlRead: Reads eFlSize bytes starting at eFlAddr into exFlashRxData[]
 * - exFlWrite: Writes eFlSize bytes from exFlashTxData[] starting at eFlAddr
 * 
 * @warning Must not be called while extFlashFunc() has state != exFlashReady
 * @warning Write data must be loaded into exFlashTxData before calling
 * @warning eFlSize must not exceed EXT_FL_PG_SIZE (256 bytes)
 * @warning eFlAddr must be aligned to appropriate block size for erases
 * 
 * Example:
 * @code
 * // Read 256 bytes from address 0x1000
 * extFlashCmd(256, 0x1000, exFlRead);
 * 
 * // Write 256 bytes at address 0x2000 (data in exFlashTxData)
 * memcpy(mtFlash.exFlashTxData, myData, 256);
 * extFlashCmd(256, 0x2000, exFlWrite);
 * 
 * // Erase 4KB sector at 0x3000
 * extFlashCmd(0, 0x3000, exFlErase);
 * @endcode
 * 
 * @note State machine must be in exFlashReady state before calling
 * @note Parameters are stored in mtFlash struct for use by state machine
 */
void extFlashCmd(uint32_t eFlSize, uint32_t eFlAddr, eFlCmd FlCmd);

/**
 * @brief Format flash read data as debug/diagnostic message
 * 
 * Packages flash data read via exFlRead command into a human-readable
 * ASCII format suitable for debugging or transmission over debug interface.
 * 
 * The formatted message includes:
 * - Management address identifier
 * - Error response indicator
 * - Network address of source
 * - Object name and number
 * - Raw page data (256 bytes)
 * - CRLF line terminator
 * 
 * @param[in,out] debugParse Pointer to debug message structure
 *                           (updated with formatted output)
 * @param[in] systemObj Pointer to system state (for addressing info)
 * @param[in] constName Pointer to constant name lookup table
 * 
 * @retval void
 * 
 * @warning Assumes sufficient buffer space in debugParse->txDumpBuffDbg
 * @warning Uses strlen() on potentially uninitialized buffer (BUG)
 * @note This function couples flash driver to debug protocol (consider separating)
 */
void extFlashRead(void *debugParse, void *systemObj, void *constName);

/**
 * @brief Copy flash read data into image data structure
 * 
 * Helper function that copies data from the flash read buffer
 * (exFlashRxData) into an image data structure for processing.
 * Typically called after an exFlRead command completes.
 * 
 * @param[in,out] imgPDat Pointer to image data structure
 *                        (cast to imgData internally)
 * 
 * @retval void
 * 
 * @warning Performs no null pointer validation (NULL check always true)
 * @warning Blindly copies EXT_FL_PG_SIZE (256 bytes) - verify buffer size
 * @note Should validate imgPDat before dereferencing
 */
void extFlashReadGui(uint32_t *imgPDat);

/**
 * @brief Copy image data to flash write buffer and initiate write
 * 
 * Helper function that:
 * 1. Copies data from image data structure to flash write buffer
 * 2. Updates flash write address from image structure
 * 3. Initiates page program (write) operation
 * 
 * Typically called to write image pages during firmware upload.
 * 
 * @param[in,out] imgPDat Pointer to image data structure
 *                        (contains imgLineBuff and imgAddr fields)
 * 
 * @retval void
 * 
 * @warning Performs no null pointer validation (NULL check always true)
 * @warning Clears write buffer before each operation
 * @warning Does not check if flash is ready before initiating write
 * @note Should validate imgPDat and check extFlashStatGet() == exFlashReady
 */
void extFlashWriteGui(uint32_t *imgPDat);

/* ============================================================================
   Interrupt Callback Functions
   
   These functions are called by the QSPI HAL during interrupt service routines.
   They set flags that are monitored by the state machine to drive operations.
   ============================================================================ */

/**
 * @brief QSPI command completion callback
 * 
 * Called by HAL_QSPI_CmdCpltCallback when a command (like write enable
 * or status register read) completes. Sets flag for state machine.
 * 
 * @param[in] hqspi QSPI peripheral handle (unused but required by HAL)
 * 
 * @retval void
 * 
 * @warning Called from interrupt context - keep very brief
 * @warning Increments flag without overflow protection
 * @note Flag cleared by state machine when processed
 */
void HAL_QSPI_CmdCpltCallback(void *hqspi);

/**
 * @brief QSPI receive complete callback
 * 
 * Called when read data reception completes. Sets flag for state machine
 * to transition from exFlashReadEnd state back to exFlashReady.
 * 
 * @param[in] hqspi QSPI peripheral handle (unused but required by HAL)
 * 
 * @retval void
 * 
 * @warning Called from interrupt context - keep very brief
 * @warning Increments flag without overflow protection
 * @note Data is already in exFlashRxData[] at this point
 */
void HAL_QSPI_RxCpltCallback(void *hqspi);

/**
 * @brief QSPI transmit complete callback
 * 
 * Called when write data transmission (page program) completes.
 * Sets flag for state machine to monitor memory ready status.
 * 
 * @param[in] hqspi QSPI peripheral handle (unused but required by HAL)
 * 
 * @retval void
 * 
 * @warning Called from interrupt context - keep very brief
 * @warning Increments flag without overflow protection
 * @note Memory may still be busy writing; subsequent polling checks status
 */
void HAL_QSPI_TxCpltCallback(void *hqspi);

/**
 * @brief QSPI status match callback
 * 
 * Called by automatic polling when the status register matches the
 * expected value (indicating operation completion). Signals to state
 * machine that erase/write operation is fully complete.
 * 
 * @param[in] hqspi QSPI peripheral handle (unused but required by HAL)
 * 
 * @retval void
 * 
 * @warning Called from interrupt context - keep very brief
 * @warning Increments flag without overflow protection
 * @note Polling mode automatically checks status repeatedly until match
 */
void HAL_QSPI_StatusMatchCallback(void *hqspi);

#endif /* INC_EXTFLASH_H_ */
