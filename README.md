# Flash Memory Manager & Image Transfer System

**Comprehensive Code Review with Fully Documented Improved Source Files**

> A complete embedded systems project featuring QSPI flash memory management and firmware image transfer with extensive analysis and properly documented implementations.

---

## 📋 Project Overview

This project implements a **Flash Memory Manager** and **Image Transfer System** for embedded microcontrollers (STM32 HAL-based). It provides:

- **External Flash Driver** - MT25QL512ABB QSPI flash memory management with state machine-based async operations
- **Image Transfer Handler** - Firmware image upload, verification, and installation with CRC32 validation
- **Redundant Metadata** - Dual metadata tables (Table A & B) for failsafe boot selection
- **Comprehensive Documentation** - Full code review with improved documented versions

## 🎯 Key Features

### Flash Operations
✅ Read data from external flash (256-byte pages)
✅ Write (page program) data with automatic polling
✅ Erase operations (4KB blocks, 32KB, 64KB, or full chip)
✅ QPI (Quad SPI) mode for faster transfers
✅ 4-byte address mode for large memory spaces
✅ Interrupt-driven callbacks for async operation completion

### Image Transfer
✅ Receive firmware images via serial/network interface
✅ Validate packet integrity with checksum
✅ Accumulate CRC32 across multiple pages
✅ Verify complete image before installation
✅ Update redundant metadata tables
✅ Support multiple image types and versions

### Memory Organization
```
External Flash (512MB)
├── Golden Images (256KB each)        - Read-only factory defaults
├── Release Images (256KB each)       - User-updatable active versions
├── Boot Images                        - Bootloader code
└── Metadata Tables (4KB each)        - Image descriptors (Table A & B)
```

---

## 📁 File Structure

### Original Source Code
```
extFlash.c          - QSPI flash driver (308 lines)
extFlash.h          - Flash driver header (67 lines)
imager.c            - Image transfer module (297 lines)
imager.h            - Image handler header (65 lines)
README.md           - This file
```
