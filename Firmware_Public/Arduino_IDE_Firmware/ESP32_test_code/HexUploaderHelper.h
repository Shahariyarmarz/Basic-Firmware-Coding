#ifndef Hex_Uploader_Helper_h
#define Hex_Uploader_Helper_h

#define K_BYTE 1024
#define NO_FUSE 0xFF

typedef enum {
  MSG_NO_SD_CARD,
  MSG_CANNOT_OPEN_FILE,
  MSG_LINE_TOO_LONG,
  MSG_LINE_TOO_SHORT,
  MSG_LINE_DOES_NOT_START_WITH_COLON,
  MSG_INVALID_HEX_DIGITS,
  MSG_BAD_SUMCHECK,
  MSG_LINE_NOT_EXPECTED_LENGTH,
  MSG_UNKNOWN_RECORD_TYPE,
  MSG_NO_END_OF_FILE_RECORD,
  MSG_FILE_TOO_LARGE_FOR_FLASH,
  MSG_CANNOT_ENTER_PROGRAMMING_MODE,
  MSG_NO_BOOTLOADER_FUSE,
  MSG_CANNOT_FIND_SIGNATURE,
  MSG_UNRECOGNIZED_SIGNATURE,
  MSG_BAD_START_ADDRESS,
  MSG_VERIFICATION_ERROR,
  MSG_FUSE_PROBLEM,
  MSG_FLASHED_OK,
} msgType;

enum {
  checkFile,
  verifyFlash,
  writeToFlash,
};

enum {
  lowFuse,
  highFuse,
  extFuse,
  lockByte,
  calibrationByte
};

typedef struct {
  byte sig[3];
  const char* desc;
  unsigned long flashSize;
  unsigned int baseBootSize;
  unsigned long pageSize;
  byte fuseWithBootloaderSize;
  byte timedWrites;
} signatureType;

enum {
  progamEnable = 0xAC,
  chipErase = 0x80,
  writeLockByte = 0xE0,
  writeLowFuseByte = 0xA0,
  writeHighFuseByte = 0xA8,
  writeExtendedFuseByte = 0xA4,
  pollReady = 0xF0,
  programAcknowledge = 0x53,
  readSignatureByte = 0x30,
  readCalibrationByte = 0x38,
  readLowFuseByte = 0x50,
  readLowFuseByteArg2 = 0x00,
  readExtendedFuseByte = 0x50,
  readExtendedFuseByteArg2 = 0x08,
  readHighFuseByte = 0x58,
  readHighFuseByteArg2 = 0x08,
  readLockByte = 0x58,
  readLockByteArg2 = 0x00,
  readProgramMemory = 0x20,
  writeProgramMemory = 0x4C,
  loadExtendedAddressByte = 0x4D,
  loadProgramMemory = 0x40,
};

enum {
  hexDataRecord,
  hexEndOfFile,
  hexExtendedSegmentAddressRecord,
  hexStartSegmentAddressRecord,
  hexExtendedLinearAddressRecord,
  hexStartLinearAddressRecord
};

const signatureType signatures[] PROGMEM = {
  { { 0x1E, 0x91, 0x0B }, "ATtiny24", 2 * K_BYTE, 0, 32, NO_FUSE, false },
  { { 0x1E, 0x92, 0x07 }, "ATtiny44", 4 * K_BYTE, 0, 64, NO_FUSE, false },
  { { 0x1E, 0x93, 0x0C }, "ATtiny84", 8 * K_BYTE, 0, 64, NO_FUSE, false },
  { { 0x1E, 0x91, 0x08 }, "ATtiny25", 2 * K_BYTE, 0, 32, NO_FUSE, false },
  { { 0x1E, 0x92, 0x06 }, "ATtiny45", 4 * K_BYTE, 0, 64, NO_FUSE, false },
  { { 0x1E, 0x93, 0x0B }, "ATtiny85", 8 * K_BYTE, 0, 64, NO_FUSE, false },
  { { 0x1E, 0x92, 0x0A }, "ATmega48PA", 4 * K_BYTE, 0, 64, NO_FUSE, false },
  { { 0x1E, 0x93, 0x0F }, "ATmega88PA", 8 * K_BYTE, 256, 128, extFuse, false },
  { { 0x1E, 0x94, 0x0B }, "ATmega168PA", 16 * K_BYTE, 256, 128, extFuse, false },
  { { 0x1E, 0x95, 0x0F }, "ATmega328P", 32 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x95, 0x16 }, "ATmega328PB", 32 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x94, 0x0A }, "ATmega164P", 16 * K_BYTE, 256, 128, highFuse, false },
  { { 0x1E, 0x95, 0x08 }, "ATmega324P", 32 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x96, 0x0A }, "ATmega644P", 64 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0x96, 0x02 }, "ATmega64", 64 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0x96, 0x08 }, "ATmega640", 64 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0x97, 0x03 }, "ATmega1280", 128 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0x97, 0x04 }, "ATmega1281", 128 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0x98, 0x01 }, "ATmega2560", 256 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0x98, 0x02 }, "ATmega2561", 256 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0x93, 0x82 }, "At90USB82", 8 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x94, 0x82 }, "At90USB162", 16 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x93, 0x89 }, "ATmega8U2", 8 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x94, 0x89 }, "ATmega16U2", 16 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x95, 0x8A }, "ATmega32U2", 32 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x94, 0x88 }, "ATmega16U4", 16 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x95, 0x87 }, "ATmega32U4", 32 * K_BYTE, 512, 128, highFuse, false },
  { { 0x1E, 0x97, 0x05 }, "ATmega1284P", 128 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0x91, 0x0A }, "ATtiny2313A", 2 * K_BYTE, 0, 32, NO_FUSE, false },
  { { 0x1E, 0x92, 0x0D }, "ATtiny4313", 4 * K_BYTE, 0, 64, NO_FUSE, false },
  { { 0x1E, 0x90, 0x07 }, "ATtiny13A", 1 * K_BYTE, 0, 32, NO_FUSE, false },
  { { 0x1E, 0x93, 0x07 }, "ATmega8A", 8 * K_BYTE, 256, 64, highFuse, true },
  { { 0x1E, 0xA6, 0x02 }, "ATmega64rfr2", 256 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0xA7, 0x02 }, "ATmega128rfr2", 256 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
  { { 0x1E, 0xA8, 0x02 }, "ATmega256rfr2", 256 * K_BYTE, 1 * K_BYTE, 256, highFuse, false },
};

#endif