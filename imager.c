/*
 * imager.c - Firmware Image Transfer and Management Implementation
 *
 * This module implements firmware image transfer, verification, and installation
 * functionality for embedded systems. It manages the process of:
 * 
 * 1. Receiving image data from communication interface in packets
 * 2. Validating packet integrity via checksum
 * 3. Writing validated data to external flash memory via extFlash module
 * 4. Accumulating CRC32 checksum across all pages
 * 5. Verifying complete image against final CRC and size
 * 6. Updating redundant metadata tables (Table A & B) for boot loader
 * 7. Supporting multiple image types and fallback mechanisms
 *
 * Image Transfer Protocol:
 * - Host sends image data in fixed-size packets (typically 256 bytes)
 * - Each packet includes optional address and a checksum byte
 * - Transfer ends with terminator packet containing final size and CRC
 * - Module validates and writes each packet to appropriate flash location
 * - After transfer, updates both metadata tables for failsafe operation
 *
 * Memory Layout:
 * - Golden images: Read-only reference copies (factory default)
 * - Release images: User-updatable active copies
 * - Metadata tables: Store image descriptors (type, size, CRC)
 * - Each image offset: 256KB (IMG_EXT_IMG_OFFSET)
 * - Each page: 256 bytes (IMG_EXT_PAGE_OFFSET)
 *
 * Created on: Oct 5, 2025
 * Author: deploy
 *
 * @file imager.c
 * @brief Firmware Image Transfer and Management
 */

#include "globVar.h"
#include "localDef.h"
#include "localEnum.h"
#include "localVar.h"
#include "imager.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "extFlash.h"

/* ============================================================================
   External References
   ============================================================================ */

/** @brief CRC peripheral handle for image data verification */
extern CRC_HandleTypeDef hcrc;

/* ============================================================================
   Public Functions - Module Initialization
   ============================================================================ */

/**
 * @brief Initialize image transfer module
 *
 * Sets up the initial state for image operations:
 * - Clears the packet ready flag
 * - Sets state machine to ready state
 *
 * @retval void
 *
 * @warning Does NOT read existing image tables from flash
 * @warning Does NOT validate previously stored images
 * @note Should be called once at system startup
 * @note Consider reading and validating tables on init for fallback support
 * @note TODO: Add table verification and fallback selection logic
 */
void imagerInit(void)
{
    /* Initialize image packet flag - no packet waiting */
    imageDat.imgNewPcktFl = IMG_NO_PACKET;
    
    /* Set state machine to ready state */
    imageDat.imgStateFlag = imStReady;
}

/* ============================================================================
   Public Functions - Image Processing
   ============================================================================ */

/**
 * @brief Parse and process incoming image payload packet
 *
 * Wrapper function that:
 * 1. Validates packet structure and checksum
 * 2. Extracts image type from payload header
 * 3. Sets the new-packet flag
 * 4. Triggers state transition if in ready state
 *
 * Typically called by communication handler when a complete packet arrives.
 *
 * @param[in,out] debugParse Pointer to communication/debug structure containing:
 *                           - rxParseBuffDbg[]: Raw received data buffer
 *                           - communicationData: Parsed header information
 *                           - currCntParse: Current position in circular buffer
 *                           - erorrCode: Error status flags
 *
 * @retval uint8_t 1 if payload valid and processed, 0 if checksum/format error
 *
 * @warning Assumes debugParse pointer is valid
 * @warning Multiple unchecked pointer dereferences
 * @warning Should add null pointer validation
 *
 * @note Extracted image type stored in imageDat.imgType
 * @note Sets imageDat.imgNewPcktFl = IMG_NEW_PACKET on success
 * @note Initiates imStTrnsferStart state if currently in imStReady
 */
uint8_t imgParsePayld(void *debugParse)
{
    /* Cast opaque pointer to debug structure */
    dbg *pDebug = (dbg *)debugParse;

    /* Validate payload format and checksum */
    if(imgPayloadCheck(pDebug))
    {
        /* Extract image type from communication data */
        imageDat.imgType = pDebug->communicationData.numberOfObj;
        
        /* Set flag indicating new packet is available */
        imageDat.imgNewPcktFl = IMG_NEW_PACKET;
        
        /* If idle, trigger transfer start */
        if(imageDat.imgStateFlag == imStReady)
        {
            imageDat.imgStateFlag = imStTrnsferStart;
        }

        return 1;  /* Success */
    }
    else
    {
        return 0;  /* Error - packet validation failed */
    }
}

/**
 * @brief Validate image payload structure, checksum, and extract data
 *
 * Performs comprehensive validation of incoming image packet:
 * 1. Extracts optional address (for direct writes to flash)
 * 2. Copies image/command data into line buffer
 * 3. Calculates checksum using XOR (simple method, not CRC32)
 * 4. Compares calculated checksum against transmitted value
 * 5. Verifies error status is within acceptable range
 *
 * Packet Format:
 * @code
 * [Optional Address (4 bytes)]  <- Only if numberOfObj == imgNoImg
 * [Image Data (IMAGE_LINE_MAX bytes)]
 * [Checksum Byte (1 byte)]      <- Low byte of checksum
 * @endcode
 *
 * Checksum Calculation:
 * @code
 * checksum = (sum of all data bytes) XOR 0xFFFF
 * checksum = checksum + 1 (two's complement)
 * transmitted = checksum & 0x00FF (low byte only)
 * @endcode
 *
 * Data Buffer Management:
 * Uses circular buffer with wraparound. Buffer position (currCntParse)
 * automatically wraps around using modulo operator.
 *
 * @param[in,out] debugParse Pointer to debug/communication structure:
 *                Input:
 *                  - rxParseBuffDbg[]: Received data (circular)
 *                  - currCntParse: Current read position
 *                  - communicationData.numberOfObj: Packet type
 *                Output:
 *                  - imageDat fields populated:
 *                    - imgAddr: Destination address (if direct write)
 *                    - imgLineBuff[]: Extracted payload data
 *                  - currCntParse: Advanced to next packet start
 *
 * @retval uint8_t 1 if valid payload extracted, 0 if validation failed
 *
 * @warning ISSUE: No bounds checking on array access
 * @warning ISSUE: IMAGE_LINE_MAX + IMAGE_ADDR_MAX buffer size assumed
 * @warning ISSUE: No validation that indices stay within buffer bounds
 * @warning ISSUE: Checksum algorithm differs from CRC32 in imgGetPgCRC()
 * @warning ISSUE: Circular buffer logic repeated (no helper function)
 * @warning Magic numbers: 0xFFFF, 0x00FF (need explanation)
 *
 * @note Checksum: Simple XOR-based, suitable for basic integrity checking
 * @note Not cryptographically secure - cannot detect deliberate tampering
 * @note Full image integrity checked by imgGetPgCRC() per-page accumulation
 * @note TODO: Extract circular buffer logic to helper function
 * @note TODO: Add array bounds validation
 * @note TODO: Unify checksum methods or clearly document difference
 */
uint8_t imgPayloadCheck(void *debugParse)
{
    /* Cast opaque pointer to debug structure */
    dbg *pDebug = (dbg *)debugParse;
    
    uint16_t imgCnt;           /* Loop counter for byte extraction */
    uint16_t checkCRC = 0;     /* Accumulated checksum */

    /* Clear image line buffer before extraction */
    memset(imageDat.imgLineBuff, 0, IMAGE_LINE_MAX + IMAGE_ADDR_MAX);

    /* Extract address if this is a direct write command (not image transfer) */
    if(pDebug->communicationData.numberOfObj == imgNoImg)
    {
        /* Extract 4-byte address from buffer in big-endian format */
        for(imgCnt = 0; imgCnt < IMAGE_ADDR_MAX; imgCnt++)
        {
            /* Get byte from circular buffer at current position */
            imageDat.imgAddr |= pDebug->rxParseBuffDbg[pDebug->currCntParse];
            
            /* Shift left if not the last byte (to build 32-bit address) */
            if(imgCnt < (IMAGE_ADDR_MAX - 1))
            {
                imageDat.imgAddr <<= 8;
            }
            
            /* Add byte to checksum */
            checkCRC += pDebug->rxParseBuffDbg[pDebug->currCntParse];
            
            /* Advance to next byte in circular buffer */
            pDebug->currCntParse++;
            pDebug->currCntParse %= DBG_RX_PARSE_BUFF_MAX_LEN;
        }
    }

    /* Extract image/command data (typically 256 bytes per page) */
    for(imgCnt = 0; imgCnt < IMAGE_LINE_MAX; imgCnt++)
    {
        /* Copy byte from buffer to image data */
        imageDat.imgLineBuff[imgCnt] = pDebug->rxParseBuffDbg[pDebug->currCntParse];
        
        /* Add to accumulated checksum */
        checkCRC += imageDat.imgLineBuff[imgCnt];
        
        /* Advance circular buffer position */
        pDebug->currCntParse++;
        pDebug->currCntParse %= DBG_RX_PARSE_BUFF_MAX_LEN;
    }

    /* Calculate final checksum: two's complement (invert and add 1) */
    checkCRC ^= 0xFFFF;        /* Invert all bits */
    checkCRC++;                /* Add 1 for two's complement */

    /* Verify transmitted checksum matches calculated value */
    if((checkCRC & 0x00FF) != pDebug->rxParseBuffDbg[pDebug->currCntParse])
    {
        /* Checksum mismatch - packet corrupted */
        return 0;
    }

    /* Advance past checksum byte */
    pDebug->currCntParse++;
    pDebug->currCntParse %= DBG_RX_PARSE_BUFF_MAX_LEN;

    /* Verify no error condition and packet not empty */
    if((!imgCnt) && (pDebug->erorrCode <= dbgParseErrOnGoing))
    {
        /* Empty packet or significant error */
        return 0;
    }

    /* All validations passed */
    return 1;
}

/* ============================================================================
   Public Functions - Main State Machine
   ============================================================================ */

/**
 * @brief Process image transfer state machine
 *
 * Main state machine that orchestrates the complete firmware image transfer,
 * validation, and installation process:
 *
 * 1. **Receives** image data packets
 * 2. **Validates** packet integrity
 * 3. **Writes** pages to external flash
 * 4. **Accumulates** CRC32 checksum across pages
 * 5. **Verifies** final image against CRC and size
 * 6. **Updates** metadata tables for boot selection
 *
 * State Sequence:
 * @code
 * imStInit
 *   ↓
 * imStReady (idle, waiting for packet)
 *   ↓
 * imStTrnsferStart (setup transfer)
 *   ↓
 * imStTrnsfer (loop for each page)
 *   ↓
 * imStTrnsferFinish (verify and prepare table update)
 *   ↓
 * imStTblAUpd (update primary metadata table)
 *   ↓
 * imStTblBUpd (update backup metadata table)
 *   ↓
 * imStInit → imStReady (ready for next transfer)
 * @endcode
 *
 * @retval void
 *
 * @warning ISSUE: Large nested switch statements (200+ lines) - difficult to follow
 * @warning ISSUE: No timeout protection on waiting states
 * @warning ISSUE: State transitions not validated (could corrupt state)
 * @warning ISSUE: Complex state interactions hard to debug
 * @warning ISSUE: Array index bounds not checked in table update
 *
 * @note Should be called frequently (10-100 Hz recommended)
 * @note Non-blocking; returns immediately
 * @note State transitions driven by flash operation completion
 * @note CRC accumulated per-page using imgGetPgCRC()
 * @note Final CRC XORed with 0xFFFFFFFF per CRC32 algorithm
 * @note TODO: Modularize into smaller state handler functions
 * @note TODO: Add timeout protection for waiting states
 * @note TODO: Add error recovery mechanisms
 */
void imagerFunc(void)
{
    switch(imageDat.imgStateFlag)
    {
        /* ====== INITIALIZATION STATE ====== */
        case imStInit:
        {
            /* Clear all transfer-related variables */
            imageDat.imgAddr = 0;
            imageDat.imgSz = 0;
            imageDat.imgCRC_IN = 0;
            imageDat.imgCRC_OUT = 0;
            imageDat.imgTypeTransfr = 0;
            imageDat.imgType = 0;
            imageDat.imgNewPcktFl = IMG_NO_PACKET;
            
            /* Transition to ready state */
            imageDat.imgStateFlag = imStReady;
            break;
        }

        /* ====== READY STATE ====== */
        case imStReady:
        {
            /* Idle state - waiting for first packet */
            /* No action needed; state transition occurs in imgParsePayld() */
            break;
        }

        /* ====== TRANSFER START STATE ====== */
        case imStTrnsferStart:
        {
            /* Proceed only when flash is ready and new packet available */
            if((extFlashStatGet() == exFlashReady) && (imageDat.imgNewPcktFl == IMG_NEW_PACKET))
            {
                if(imageDat.imgType == imgNoImg)
                {
                    /* Direct write to flash (address in payload) */
                    extFlashWriteGui((uint32_t*)&imageDat);
                    
                    /* Return to ready state after direct write */
                    imageDat.imgStateFlag = imStReady;
                }
                else
                {
                    /* Begin image transfer sequence */
                    
                    /* Set image type for transfer tracking */
                    imageDat.imgTypeTransfr = imageDat.imgType;
                    
                    /* Calculate starting address: (type-1) * offset per type */
                    imageDat.imgAddr = (imageDat.imgType - 1) * IMG_EXT_IMG_OFFSET;
                    
                    /* Initialize CRC for this image (all 1s per CRC32) */
                    imageDat.imgCRC_OUT = imgGetPgCRC(&imageDat, 0xFFFFFFFF);
                    imageDat.imgCRC_IN = imageDat.imgCRC_OUT;
                    
                    /* Initialize size (first page) */
                    imageDat.imgSz = IMG_EXT_PAGE_OFFSET;
                    
                    /* Write first page */
                    extFlashWriteGui((uint32_t*)&imageDat);
                    
                    /* Transition to page transfer loop */
                    imageDat.imgStateFlag = imStTrnsfer;
                }
                
                /* Clear packet flag for next iteration */
                imageDat.imgNewPcktFl = IMG_NO_PACKET;
            }
            break;
        }

        /* ====== TRANSFER STATE (Page Loop) ====== */
        case imStTrnsfer:
        {
            /* Process next packet in transfer sequence */
            if((extFlashStatGet() == exFlashReady) && (imageDat.imgNewPcktFl == IMG_NEW_PACKET))
            {
                if(imageDat.imgType == ImgTerminator)
                {
                    /* Terminator packet received - end of transfer */
                    /* Terminator contains final size and CRC for verification */

                    /* Final CRC computation: apply standard CRC32 final XOR */
                    imageDat.imgCRC_IN ^= 0xFFFFFFFF;
                    imageDat.imgCRC_OUT = imageDat.imgCRC_IN;

                    /* Verify image integrity */
                    if(imgVerify(&imageDat))
                    {
                        /* Verification successful - proceed to table update */
                        imageDat.imgStateFlag = imStTrnsferFinish;
                    }
                    else
                    {
                        /* Verification failed - corrupted image received */
                        imageDat.imgStateFlag = imStError;
                        break;
                    }
                }
                else
                {
                    /* Regular data page - accumulate CRC and increment address */

                    /* Update address for next page */
                    imageDat.imgAddr += IMG_EXT_PAGE_OFFSET;
                    
                    /* Update accumulated size */
                    imageDat.imgSz += IMG_EXT_PAGE_OFFSET;
                    
                    /* Calculate CRC for this page (accumulative) */
                    imageDat.imgCRC_OUT = imgGetPgCRC(&imageDat, imageDat.imgCRC_IN);
                    imageDat.imgCRC_IN = imageDat.imgCRC_OUT;
                    
                    /* Write page to flash */
                    extFlashWriteGui((uint32_t*)&imageDat);
                }

                /* Clear packet flag */
                imageDat.imgNewPcktFl = IMG_NO_PACKET;
            }
            break;
        }

        /* ====== TRANSFER FINISHED STATE ====== */
        case imStTrnsferFinish:
        {
            /* Wait for flash to be ready, then read and update Table A */
            if(extFlashStatGet() == exFlashReady)
            {
                /* Initiate read of primary metadata table */
                extFlashCmd(EXT_FL_PG_SIZE, IMG_TABLE_A, exFlRead);
                
                /* Initialize table update sub-state machine */
                imageDat.updTblStg = tblRead;
                imageDat.imgStateFlag = imStTblAUpd;
            }
            break;
        }

        /* ====== TABLE A UPDATE STATE ====== */
        case imStTblAUpd:
        {
            /* Update primary metadata table */
            if(extFlashStatGet() == exFlashReady)
            {
                switch(imageDat.updTblStg)
                {
                    case tblRead:
                    {
                        /* Set address for Table A read */
                        imageDat.imgAddr = IMG_TABLE_A;
                        
                        /* Copy read data to line buffer */
                        extFlashReadGui((uint32_t *)&imageDat);
                        
                        /* Update table entry with new image info */
                        imgTableUpdt(&imageDat, IMG_TABLE_A);
                        
                        /* Transition to erase stage */
                        imageDat.updTblStg++;
                        break;
                    }

                    case tblUpd:
                    {
                        /* Erase Table A block */
                        extFlashCmd(EXT_FL_PG_SIZE, IMG_TABLE_A, exFlErase);
                        
                        /* Transition to write stage after erase completes */
                        imageDat.updTblStg++;
                        break;
                    }

                    case tblErs:
                    {
                        /* Wait for erase to complete */
                        if(extFlashCmdGet() == exFlErase)
                        {
                            /* Set address for write-back */
                            imageDat.imgAddr = IMG_TABLE_A;
                            
                            /* Write updated table back to flash */
                            extFlashWriteGui((uint32_t*)&imageDat);
                            
                            /* Transition to write completion stage */
                            imageDat.updTblStg++;
                        }
                        break;
                    }

                    case tblWr:
                    {
                        /* Wait for write to complete */
                        if(extFlashCmdGet() == exFlWrite)
                        {
                            /* Table A updated successfully - move to Table B */
                            imageDat.imgStateFlag = imStTblBUpd;
                            imageDat.updTblStg = tblRead;
                        }
                        break;
                    }

                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }

        /* ====== TABLE B UPDATE STATE ====== */
        case imStTblBUpd:
        {
            /* Update backup metadata table (same sequence as Table A) */
            if(extFlashStatGet() == exFlashReady)
            {
                switch(imageDat.updTblStg)
                {
                    case tblRead:
                    {
                        /* Initiate read of Table B */
                        imageDat.imgAddr = IMG_TABLE_B;
                        extFlashCmd(EXT_FL_PG_SIZE, IMG_TABLE_B, exFlRead);
                        
                        /* Transition to update stage */
                        imageDat.updTblStg++;
                        break;
                    }

                    case tblUpd:
                    {
                        /* Copy read data and update table */
                        imageDat.imgAddr = IMG_TABLE_B;
                        extFlashReadGui((uint32_t *)&imageDat);
                        imgTableUpdt(&imageDat, IMG_TABLE_B);
                        
                        /* Initiate erase */
                        extFlashCmd(EXT_FL_PG_SIZE, IMG_TABLE_B, exFlErase);
                        
                        /* Transition to erase completion */
                        imageDat.updTblStg++;
                        break;
                    }

                    case tblErs:
                    {
                        /* Wait for erase completion */
                        if(extFlashCmdGet() == exFlErase)
                        {
                            /* Set address for write-back */
                            imageDat.imgAddr = IMG_TABLE_B;
                            
                            /* Write updated table */
                            extFlashWriteGui((uint32_t*)&imageDat);
                            
                            /* Transition to write completion */
                            imageDat.updTblStg++;
                        }
                        break;
                    }

                    case tblWr:
                    {
                        /* Wait for write completion */
                        if(extFlashCmdGet() == exFlWrite)
                        {
                            /* Both tables updated - transfer complete */
                            imageDat.imgStateFlag = imStInit;
                            imageDat.updTblStg = tblRead;
                        }
                        break;
                    }

                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }

        /* ====== MOUNT STATE ====== */
        case imStMount:
        {
            /* Reserved for future mount/activation logic */
            break;
        }

        /* ====== ERROR STATE ====== */
        case imStError:
        {
            /* Clear all state on error */
            imageDat.imgAddr = 0;
            imageDat.imgSz = 0;
            imageDat.imgCRC_IN = 0;
            imageDat.imgCRC_OUT = 0;
            imageDat.imgTypeTransfr = 0;
            imageDat.imgType = 0;
            imageDat.imgNewPcktFl = IMG_NO_PACKET;
            
            /* Return to ready state */
            imageDat.imgStateFlag = imStReady;
            break;
        }

        default:
        {
            break;
        }
    }
}

/* ============================================================================
   Utility Functions
   ============================================================================ */

/**
 * @brief Update metadata table entry with image information
 *
 * Updates a table entry with the current image's metadata (type, size, CRC).
 * Each metadata table contains entries for different image types, allowing
 * the boot loader to select which image to execute.
 *
 * @param[in,out] imgDat Pointer to image data structure containing:
 *                       - imgTypeTransfr: Image type (1-based index)
 *                       - imgSz: Total image size in bytes
 *                       - imgCRC_OUT: Calculated CRC32 of image
 *                       - imgLineBuff: Current table data
 *                Output: imgLineBuff updated with new entry
 * @param[in] tbl Table selector: IMG_TABLE_A or IMG_TABLE_B
 *
 * @retval void
 *
 * @warning ISSUE: No array bounds validation on imgTypeTransfr
 * @warning ISSUE: Assumes imgTypeTransfr-1 is valid array index
 * @warning Could cause buffer overflow if imgTypeTransfr exceeds table size
 * @warning Code duplication: TABLE_A and TABLE_B cases are identical
 *
 * @note Intended to be called from imagerFunc state machine
 * @note Table entries use 1-based type numbering (subtract 1 for index)
 * @note TODO: Add bounds checking for array access
 * @note TODO: Extract common logic to helper function
 * @note TODO: Return error status instead of void
 */
static void imgTableUpdt(imgData *imgDat, uint32_t tbl)
{
    /* Get table size (size of entire table array) */
    uint32_t tbSz = sizeof(imgDat->imgTableA);

    /* TODO: ISSUE - Should validate:
     * if(imgDat->imgTypeTransfr == 0 || imgDat->imgTypeTransfr > NUM_TABLE_ENTRIES)
     *     return -1;  // Out of bounds */

    switch(tbl)
    {
        case IMG_TABLE_A:
        {
            /* Read current table from buffer into local copy */
            memcpy(imgDat->imgTableA, imgDat->imgLineBuff, tbSz);

            /* Update entry with new image info (1-based type -> 0-based index) */
            imgDat->imgTableA[imgDat->imgTypeTransfr - 1].imgType = imgDat->imgTypeTransfr;
            imgDat->imgTableA[imgDat->imgTypeTransfr - 1].imgSize = imgDat->imgSz;
            imgDat->imgTableA[imgDat->imgTypeTransfr - 1].imgCRC = imgDat->imgCRC_OUT;

            /* Copy updated table back to buffer for write */
            memcpy(imgDat->imgLineBuff, imgDat->imgTableA, tbSz);
            break;
        }

        case IMG_TABLE_B:
        {
            /* Identical logic for Table B backup */
            memcpy(imgDat->imgTableB, imgDat->imgLineBuff, tbSz);

            imgDat->imgTableB[imgDat->imgTypeTransfr - 1].imgType = imgDat->imgTypeTransfr;
            imgDat->imgTableB[imgDat->imgTypeTransfr - 1].imgSize = imgDat->imgSz;
            imgDat->imgTableB[imgDat->imgTypeTransfr - 1].imgCRC = imgDat->imgCRC_OUT;

            memcpy(imgDat->imgLineBuff, imgDat->imgTableB, tbSz);
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * @brief Verify image integrity using size and CRC32 checksum
 *
 * Compares the received image's size and CRC against expected values
 * embedded in the image data itself (terminator packet). This provides
 * verification that the complete image transferred correctly.
 *
 * Expected values are stored at fixed offsets within image page:
 * - IMG_TYP_CHCK_OFFSET: Image type identifier
 * - IMG_SZ_CHCK_OFFSET: Total image size (32-bit)
 * - IMG_CRC_CHCK_OFFSET: CRC32 checksum (32-bit)
 *
 * @param[in] imgDat Pointer to image data structure containing:
 *                   - imgTypeTransfr: Expected image type
 *                   - imgSz: Accumulated image size
 *                   - imgCRC_OUT: Calculated CRC32
 *                   - imgLineBuff: Buffer containing verification data
 *
 * @retval uint8_t 1 if verification passed, 0 if failed
 *
 * @warning Relies on correct offset definitions in header
 * @warning Assumes imgLineBuff contains terminator packet data
 * @note Critical for security - prevents partial/corrupted images
 * @note Verification must occur before installing image
 */
static uint8_t imgVerify(imgData *imgDat)
{
    /* Extract expected values from image terminator packet */
    uint32_t *verCRC = (uint32_t*)&imgDat->imgLineBuff[IMG_CRC_CHCK_OFFSET];
    uint32_t *verSZ = (uint32_t*)&imgDat->imgLineBuff[IMG_SZ_CHCK_OFFSET];

    /* Verify image type matches */
    if(imgDat->imgLineBuff[IMG_TYP_CHCK_OFFSET] == imgDat->imgTypeTransfr)
    {
        /* Verify size and CRC both match */
        if((*verSZ == imgDat->imgSz) && (*verCRC == imgDat->imgCRC_OUT))
        {
            return 1;  /* All verification checks passed */
        }
        return 0;      /* Size or CRC mismatch */
    }

    /* Type mismatch */
    return 0;
}

/**
 * @brief Reverse all bits in a 32-bit value
 *
 * Utility function that reverses the bit order of a 32-bit integer.
 * Used for CRC32 verification where certain algorithms require
 * bit-reversed initial/final values.
 *
 * Example: 0x12345678 becomes 0x1E6A2C48
 *
 * @param[in] n 32-bit value to reverse
 *
 * @retval uint32_t Value with all bits reversed
 *
 * @note Generic implementation works for any size integer
 * @note Performance: O(32) - could optimize with lookup table
 * @note Correct algorithm - properly handles all bit positions
 *
 * Algorithm:
 * @code
 * For each bit position i in input:
 *   If bit i is set in input:
 *     Set bit (31-i) in output
 * @endcode
 */
static uint32_t reverseBits(uint32_t n)
{
    uint32_t reversed_n = 0;
    uint8_t num_bits = sizeof(n) * 8;  /* Bits in integer type */

    /* Process each bit */
    for(uint8_t i = 0; i < num_bits; i++)
    {
        /* Check if bit i is set */
        if((n >> i) & 1)
        {
            /* Set bit (num_bits-1-i) in result */
            reversed_n |= (1 << (num_bits - 1 - i));
        }
    }

    return reversed_n;
}

/**
 * @brief Calculate CRC32 for image page (cumulative)
 *
 * Calculates CRC32 checksum for current image page and accumulates
 * across multiple pages for complete image verification.
 *
 * CRC Calculation:
 * 1. Reverse bits of previous CRC value (lastCRC)
 * 2. Use as initial CRC value
 * 3. Calculate CRC for this page
 * 4. Result is carried forward to next page
 * 5. After final page, XOR result with 0xFFFFFFFF
 *
 * @param[in] imgDat Pointer to image data containing imgLineBuff
 * @param[in] lastCRC Previous CRC value from last page
 *                    (0xFFFFFFFF for first page)
 *
 * @retval uint32_t CRC32 value for this page (to feed to next page)
 *
 * @note Cumulative CRC: Each page's result feeds into next page
 * @note First call: lastCRC = 0xFFFFFFFF (CRC32 standard)
 * @note Final call: Result XORed with 0xFFFFFFFF per CRC32 spec
 * @note Bit reversal required by specific CRC32 polynomial variant
 * @warning Assumes hcrc (CRC peripheral) is initialized
 * @note TODO: Document CRC polynomial and initial value used
 */
static uint32_t imgGetPgCRC(imgData *imgDat, uint32_t lastCRC)
{
    uint32_t newCRC;
    uint32_t invnewCRC;

    /* Reverse bits of input CRC (required for this CRC polynomial) */
    invnewCRC = reverseBits(lastCRC);

    /* Reset CRC peripheral to clean state */
    __HAL_CRC_DR_RESET(&hcrc);

    /* Set CRC initial value (usually 0xFFFFFFFF) */
    __HAL_CRC_INITIALCRCVALUE_CONFIG(&hcrc, invnewCRC);

    /* Calculate CRC for this page (first 256 bytes minus metadata) */
    newCRC = HAL_CRC_Calculate(&hcrc, (uint32_t *)imgDat->imgLineBuff, IMG_EXT_PAGE_OFFSET);

    /* Restore CRC to default initial value for next page */
    __HAL_CRC_INITIALCRCVALUE_CONFIG(&hcrc, 0xFFFFFFFF);

    return newCRC;
}

