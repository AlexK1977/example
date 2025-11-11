/*
 * imager.h - Firmware Image Transfer and Management Module
 *
 * This module handles the transfer, verification, and installation of firmware
 * images to external flash memory. It implements a state machine that:
 * 
 * 1. Receives image data packets via communication interface
 * 2. Validates data integrity using CRC32 checksum
 * 3. Writes image pages to external flash in sequence
 * 4. Updates image metadata tables (Table A and Table B) for boot selection
 * 5. Provides mechanisms for device firmware updates (OTA-like functionality)
 *
 * Image Memory Layout:
 * - Multiple image slots for different firmware types (MNG, QV, BAT, OH, SP)
 * - Golden (factory) and Release (active) copies of each image
 * - Metadata tables at fixed addresses for boot loader reference
 * - Each image offset is 256KB (IMG_EXT_IMG_OFFSET)
 * - Each page is 256 bytes (IMG_EXT_PAGE_OFFSET)
 *
 * State Machine Flow (Simplified):
 * imStReady → imStTrnsferStart → imStTrnsfer → imStTrnsferFinish
 *          → imStTblAUpd → imStTblBUpd → imStInit
 *
 * Created on: Oct 5, 2025
 * Author: deploy
 */

#ifndef INC_IMAGER_H_
#define INC_IMAGER_H_

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
   Memory Layout and Address Definitions
   ============================================================================ */

/**
 * @defgroup img_addresses Image Memory Addresses
 * @brief Physical addresses in external flash for different image types
 * @{
 */

/* Factory/Golden Images - Read-only reference copies */
#define IMG_MNG_GOLD_ADDR       0x00000000UL    /**< Management golden image */
#define IMG_QV_GOLD_ADDR        0x00040000UL    /**< Quad Vision golden image */
#define IMG_BAT_GOLD_ADDR       0x00080000UL    /**< Battery Manager golden image */
#define IMG_OH_GOLD_ADDR        0x000C0000UL    /**< Oil/Hydraulic golden image */
#define IMG_SP_GOLD_ADDR        0x00100000UL    /**< Special Purpose golden image */
#define IMG_BOOT_MNG_GOLD_ADDR  0x00140000UL    /**< Management boot golden image */
#define IMG_BOOT_QV_GOLD_ADDR   0x00180000UL    /**< QV boot golden image */
#define IMG_BOOT_BAT_GOLD_ADDR  0x001C0000UL    /**< Battery boot golden image */
#define IMG_BOOT_OH_GOLD_ADDR   0x00200000UL    /**< OH boot golden image */
#define IMG_BOOT_SP_GOLD_ADDR   0x00240000UL    /**< Special boot golden image */

/* Release/Active Images - User-updatable copies */
#define IMG_MNG_RLS_ADDR        0x00280000UL    /**< Management release image */
#define IMG_QV_RLS_ADDR         0x002C0000UL    /**< QV release image */
#define IMG_BAT_RLS_ADDR        0x00300000UL    /**< Battery Manager release image */
#define IMG_OH_RLS_ADDR         0x00340000UL    /**< OH release image */
#define IMG_SP_RLS_ADDR         0x00380000UL    /**< Special Purpose release image */
#define IMG_BOOT_MNG_RLS_ADDR   0x003C0000UL    /**< Management boot release image */
#define IMG_BOOT_QV_RLS_ADDR    0x00400000UL    /**< QV boot release image */
#define IMG_BOOT_BAT_RLS_ADDR   0x00440000UL    /**< Battery boot release image */
#define IMG_BOOT_OH_RLS_ADDR    0x00480000UL    /**< OH boot release image */
#define IMG_BOOT_SP_RLS_ADDR    0x004C0000UL    /**< Special boot release image */

/* Metadata Tables - Image descriptors and CRC values */
#define IMG_TABLE_A             0x00500000UL    /**< Primary metadata table */
#define IMG_TABLE_B             0x00501000UL    /**< Backup metadata table */

/** @} */

/* ============================================================================
   Image Transfer Control Constants
   ============================================================================ */

/** @brief No image packet received in this transfer cycle */
#define IMG_NO_PACKET           0

/** @brief New image packet received and ready for processing */
#define IMG_NEW_PACKET          1

/** @brief Distance between consecutive image slots (256 KB) */
#define IMG_EXT_IMG_OFFSET      0x00040000UL

/** @brief Page size for image transfers (256 bytes) */
#define IMG_EXT_PAGE_OFFSET     0x00000100UL

/* ============================================================================
   Image Data Verification Offsets
   
   These offsets define where critical image information is stored within
   a page buffer for validation purposes. Used by imgVerify() function.
   ============================================================================ */

/** @brief Offset of signature/identification word in image page */
#define IMG_SIGNWRD_OFFSET      252

/** @brief Offset of image size field (4 bytes) in image page */
#define IMG_SZ_CHCK_OFFSET      244

/** @brief Offset of CRC32 checksum field (4 bytes) in image page */
#define IMG_CRC_CHCK_OFFSET     248

/** @brief Offset of image type identifier (1 byte) in image page */
#define IMG_TYP_CHCK_OFFSET     243

/* ============================================================================
   Enumeration Types
   ============================================================================ */

/**
 * @enum imgState
 * @brief State machine states for image transfer process
 * 
 * Represents all possible states in the image transfer state machine.
 * Transitions occur based on flash operation completion and packet availability.
 */
typedef enum imgState {
    imStInit = 0,              /**< Initialization state - clears all variables */
    imStReady,                 /**< Idle state - waiting for first packet */
    imStTrnsferStart,          /**< Starting image transfer - setup phase */
    imStTrnsfer,               /**< Transferring image pages - main loop */
    imStTrnsferFinish,         /**< Transfer complete - prepare table update */
    imStTblAUpd,               /**< Updating primary metadata table */
    imStTblBUpd,               /**< Updating backup metadata table */
    imStMount,                 /**< Mount/activate image (reserved) */
    imStError,                 /**< Error state - reset to ready */
} imgState_t;

/**
 * @enum tableUpdateStage
 * @brief Sub-states for metadata table update operations
 * 
 * Table updates require multiple flash operations in sequence:
 * read current table, update entries, erase, write back.
 */
typedef enum tableUpdateStage {
    tblRead = 0,               /**< Stage 0: Read current table from flash */
    tblUpd,                    /**< Stage 1: Update table entries in RAM */
    tblErs,                    /**< Stage 2: Erase old table block from flash */
    tblWr,                     /**< Stage 3: Write updated table back to flash */
} tblStg_t;

/* ============================================================================
   Image Metadata Structure
   
   This structure would be defined elsewhere, but represents the format
   of entries in the metadata tables.
   ============================================================================ */

/**
 * @struct imgMetadata
 * @brief Entry in image metadata table
 * 
 * Each table (A and B) contains multiple entries describing available images.
 * This allows the bootloader to select which image to execute.
 */
typedef struct imgMetadata {
    uint8_t  imgType;                 /**< Image type identifier */
    uint32_t imgSize;                 /**< Total size of image in bytes */
    uint32_t imgCRC;                  /**< CRC32 checksum of image */
} imgMetadata_t;

/* ============================================================================
   Public Function Declarations
   ============================================================================ */

/**
 * @brief Initialize the image transfer module
 * 
 * Sets up the initial state for image operations:
 * - Clears the no-packet flag
 * - Sets state machine to ready
 * - Note: Does NOT read existing image tables from flash
 *
 * @retval void
 * 
 * @note Should be called at system startup before imagerFunc()
 * @note Consider reading and validating existing tables on init
 */
void imagerInit(void);

/**
 * @brief Process image transfer state machine
 * 
 * Main state machine handler that manages the firmware image transfer process.
 * This function orchestrates:
 * 1. Receiving image data from communication interface
 * 2. Verifying data integrity via CRC
 * 3. Writing pages to external flash via extFlash module
 * 4. Managing image termination
 * 5. Updating metadata tables
 * 6. Error handling and recovery
 *
 * State transitions are triggered by:
 * - Flash operation completion (extFlashStatGet() == exFlashReady)
 * - New packet availability (imgNewPcktFl == IMG_NEW_PACKET)
 * - CRC verification results
 * - Special terminator packet receipt
 *
 * @retval void
 * 
 * @warning Large switch statement with nested switches - difficult to follow
 * @warning No timeout protection on state waiting conditions
 * @warning State transitions not validated (could corrupt state)
 * 
 * @note Should be called frequently (main loop or timer ISR)
 * @note Non-blocking; returns immediately without waiting
 * 
 * Implementation Notes:
 * - Transfer protocol expects image packets + terminator
 * - Terminator packet contains final size and CRC32
 * - Two tables (A/B) provide redundancy for metadata
 * - CRC calculated per-page and accumulated across transfer
 */
void imagerFunc(void);

/**
 * @brief Parse and validate incoming image payload packet
 * 
 * Wrapper function that:
 * 1. Calls imgPayloadCheck() to validate packet structure and CRC
 * 2. Extracts image type from payload
 * 3. Sets the new-packet flag
 * 4. Triggers state transition if in ready state
 *
 * Typically called by communication handler when a complete message arrives.
 *
 * @param[in,out] debugParse Pointer to debug/comm structure containing:
 *                           - rxParseBuffDbg: Raw received data
 *                           - communicationData: Parsed header info
 *                           - currCntParse: Current position in buffer
 *
 * @retval uint8_t 1 if payload valid and processed, 0 if error
 *
 * @warning Assumes debugParse is valid pointer
 * @warning Multiple pointer dereferences without null checks
 *
 * @note Extracted image type is stored in imageDat.imgType
 * @note Sets imageDat.imgNewPcktFl = IMG_NEW_PACKET on success
 */
uint8_t imgParsePayld(void *debugParse);

/**
 * @brief Validate image payload structure, CRC, and extract data
 * 
 * Performs comprehensive validation of incoming image data:
 * 1. Extracts address (for direct writes) if indicated
 * 2. Copies image data into line buffer
 * 3. Calculates checksum (XOR-based, NOT CRC32)
 * 4. Validates checksum matches transmitted value
 * 5. Verifies error status is acceptable
 *
 * Buffer extraction uses a circular buffer with wraparound counter.
 *
 * Packet Format:
 * ```
 * [Optional Address (4 bytes)]  <- if numberOfObj == imgNoImg
 * [Image Data (IMAGE_LINE_MAX bytes)]
 * [Checksum (1 byte)]           <- low byte of computed checksum
 * ```
 *
 * @param[in,out] debugParse Pointer to communication structure:
 *                Input:  rxParseBuffDbg[], currCntParse, communicationData
 *                Output: imageDat fields populated
 *
 * @retval uint8_t 1 if valid payload extracted, 0 if checksum/format error
 *
 * @warning No bounds checking on array indices
 * @warning Checksum algorithm differs from CRC32 used in imgGetPgCRC()
 * @warning Circular buffer logic could be error-prone
 * @warning Magic numbers: 0xFFFF, 0x00FF - no explanation
 *
 * @note Checksum: ~(sum of all data bytes) + 1 (two's complement)
 * @note Buffer size assumptions: IMAGE_LINE_MAX + IMAGE_ADDR_MAX
 * @note currCntParse wrapped with modulo operator frequently
 */
uint8_t imgPayloadCheck(void *debugParse);

#endif /* INC_IMAGER_H_ */
