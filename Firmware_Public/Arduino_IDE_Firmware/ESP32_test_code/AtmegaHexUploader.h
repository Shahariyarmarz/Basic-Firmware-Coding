#ifndef Atmega_Hex_Uploader_h
#define Atmega_Hex_Uploader_h

#include "FS.h"
#include "SPIFFS.h"
#include "HexUploaderHelper.h"

#define CROSSROADS_PROGRAMMING_BOARD false
#define NO_ENCODER true
#define HEX_FILE_NAMES true
#define NUMITEMS(arg) ((unsigned int)(sizeof(arg) / sizeof(arg[0])))

#define ENTER_PROGRAMMING_ATTEMPTS 2
#define BB_DELAY_MICROSECONDS 6

#define NO_PAGE 0xFFFFFFFF
#define MAX_FILENAME_SIZE 32

class AtmegaHexUploader {
public:
  AtmegaHexUploader(int _softMisoPin, int _softMosiPin);  // Software: MISO, MOSI
  bool begin(int _softResetPin, int _softSckPin);         // Software: RESET, SCK
  bool upload(char* _fileName);
  bool start();
  void stop();
  bool getSignature();
  void getFuseBytes();
  void showDirectory();

private:
  int softMisoPin;
  int softMosiPin;
  int softSckPin;
  int softResetPin;

  bool gotEndOfFile;
  bool fileSystemReady = false;
  unsigned long extendedAddress;
  unsigned long lowestAddress;
  unsigned long highestAddress;
  unsigned long bytesWritten;
  unsigned long pagesize;
  unsigned long pagemask;
  unsigned long oldPage;
  unsigned int progressBarCount;
  unsigned int lineCount;
  unsigned int errors;
  int foundSig = -1;
  byte lastAddressMSB = 0;
  signatureType currentSignature;
  byte fuses[5];
  char flashFileName[MAX_FILENAME_SIZE] = { 0 };
  const byte fuseCommands[4] = { writeLowFuseByte, writeHighFuseByte, writeExtendedFuseByte, writeLockByte };

  bool _checkFlashFile(char* _fileName);
  bool _writeFlashContents();
  void _showMessage(const byte which);
  byte _softSPITransfer(byte c);
  byte _program(const byte b1, const byte b2 = 0, const byte b3 = 0, const byte b4 = 0);
  byte _readFlash(unsigned long addr);
  void _writeFlash(unsigned long addr, const byte data);
  bool _hexConv(const char*(&pStr), byte& b);
  void _pollUntilReady();
  void _clearPage();
  void _commitPage(unsigned long addr);
  void _writeData(const unsigned long addr, const byte* pData, const int length);
  void _verifyData(const unsigned long addr, const byte* pData, const int length);
  bool _processLine(const char* pLine, const byte action);
  bool _readHexFile(const char* fName, const byte action);
  void _printHex(byte _hex);
  void _writeFuse(const byte newValue, const byte instruction);
  bool _updateFuses(const bool writeIt);
  bool _chooseInputFile();
};

AtmegaHexUploader::AtmegaHexUploader(int _softMisoPin, int _softMosiPin) {
  softMisoPin = _softMisoPin;
  softMosiPin = _softMosiPin;
}

bool AtmegaHexUploader::begin(int _softResetPin, int _softSckPin) {
  softSckPin = _softSckPin;
  softResetPin = _softResetPin;

  if (!fileSystemReady) {
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS Failed");
      delay(1000);
    } else {
      fileSystemReady = true;
    }

    if (!fileSystemReady) {
      if (SPIFFS.begin(true)) {
        Serial.println("SPIFFS Started");
        fileSystemReady = true;
      }
    }

    if (fileSystemReady) {
      showDirectory();
      fileSystemReady = true;
      return fileSystemReady;
    }
  }

  return fileSystemReady;
}

bool AtmegaHexUploader::upload(char* _fileName) {
  int _len = strlen(_fileName);
  if ((_len <= MAX_FILENAME_SIZE) && (_len > 6) && (_fileName[0] == '/')) {  // /*.hex = 6
    if (_checkFlashFile(_fileName)) {
      strcpy(flashFileName, _fileName);

      if (!start()) {
        _showMessage(MSG_CANNOT_ENTER_PROGRAMMING_MODE);
        return false;
      }

      getSignature();
      getFuseBytes();

      if (foundSig == -1) {
        _showMessage(MSG_CANNOT_FIND_SIGNATURE);
        return false;
      }

      bool ok = _writeFlashContents();

      Serial.println();
      Serial.println();

      getFuseBytes();
      stop();

      if (ok) {
        _showMessage(MSG_FLASHED_OK);
        return true;
      }
    }
  }

  return false;
}

bool AtmegaHexUploader::start() {
  byte confirm;
  unsigned int timeout = 0;

  pinMode(softResetPin, OUTPUT);
  digitalWrite(softSckPin, LOW);
  pinMode(softSckPin, OUTPUT);
  pinMode(softMosiPin, OUTPUT);

  do {
    delay(100);
    noInterrupts();
    digitalWrite(softSckPin, LOW);

    digitalWrite(softResetPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(softResetPin, LOW);
    interrupts();

    delay(25);
    noInterrupts();
    _softSPITransfer(progamEnable);
    _softSPITransfer(programAcknowledge);
    confirm = _softSPITransfer(0);
    _softSPITransfer(0);
    interrupts();

    if (confirm != programAcknowledge) {
      if (timeout++ >= ENTER_PROGRAMMING_ATTEMPTS) {
        _showMessage(MSG_CANNOT_ENTER_PROGRAMMING_MODE);
        return false;
      }
    }
  } while (confirm != programAcknowledge);
  return true;
}

void AtmegaHexUploader::stop() {
  digitalWrite(softResetPin, LOW);
  digitalWrite(softSckPin, LOW);
  digitalWrite(softMosiPin, LOW);
  digitalWrite(softMisoPin, LOW);

  pinMode(softResetPin, INPUT);
  pinMode(softSckPin, INPUT);
  pinMode(softMosiPin, INPUT);
  pinMode(softMisoPin, INPUT);
}

bool AtmegaHexUploader::getSignature() {
  foundSig = -1;
  lastAddressMSB = 0;

  byte sig[3];
  for (byte i = 0; i < 3; i++) {
    sig[i] = _program(readSignatureByte, 0, i);
  }

  for (unsigned int j = 0; j < NUMITEMS(signatures); j++) {
    memcpy_P(&currentSignature, &signatures[j], sizeof currentSignature);

    if (memcmp(sig, currentSignature.sig, sizeof sig) == 0) {
      foundSig = j;

      Serial.println();
      Serial.println("Signature Matched!");
      Serial.print("Signature: ");
      _printHex(currentSignature.sig[0]);
      Serial.print(" ");
      _printHex(currentSignature.sig[1]);
      Serial.print(" ");
      _printHex(currentSignature.sig[2]);
      Serial.println();

      Serial.print("Description: ");
      Serial.println(currentSignature.desc);

      Serial.print("Flash Size: ");
      Serial.print(currentSignature.flashSize / 1024.0);
      Serial.println(" kB");

      Serial.print("Base Boot Size: ");
      Serial.print(currentSignature.baseBootSize);
      Serial.println(" B");

      Serial.print("Page Size: ");
      Serial.print(currentSignature.pageSize);
      Serial.println(" B");

      Serial.print("Fuse with Bootloader Size: ");
      _printHex(currentSignature.fuseWithBootloaderSize);
      Serial.println();

      Serial.print("Timed Writes: ");
      _printHex(currentSignature.timedWrites);
      Serial.println();
      Serial.println();

      _program(loadExtendedAddressByte, 0, 0);
      return true;
    }
  }
  _showMessage(MSG_UNRECOGNIZED_SIGNATURE);
  return false;
}

void AtmegaHexUploader::getFuseBytes() {
  fuses[lowFuse] = _program(readLowFuseByte, readLowFuseByteArg2);
  fuses[highFuse] = _program(readHighFuseByte, readHighFuseByteArg2);
  fuses[extFuse] = _program(readExtendedFuseByte, readExtendedFuseByteArg2);
  fuses[lockByte] = _program(readLockByte, readLockByteArg2);
  fuses[calibrationByte] = _program(readCalibrationByte);

  Serial.print("LFuse: ");
  _printHex(fuses[lowFuse]);
  Serial.println();

  Serial.print("HFuse: ");
  _printHex(fuses[highFuse]);
  Serial.println();

  Serial.print("EFuse: ");
  _printHex(fuses[extFuse]);
  Serial.println();

  Serial.print("Lock Byte: ");
  _printHex(fuses[lockByte]);
  Serial.println();

  Serial.print("Clock Calibration: ");
  _printHex(fuses[calibrationByte]);
  Serial.println();
  Serial.println();
}

void AtmegaHexUploader::showDirectory() {
  Serial.println("Listing directory: /");

  File root = SPIFFS.open("/");
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }

  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

bool AtmegaHexUploader::_checkFlashFile(char* _fileName) {
  int _len = strlen(_fileName) - 1;
  char fileRealName[MAX_FILENAME_SIZE] = "";
  memcpy((void*)&fileRealName, (void*)&_fileName[1], _len);
  Serial.print("Find Flash Name: ");
  Serial.println(fileRealName);

  File root = SPIFFS.open("/");
  if (!root) {
    Serial.println("- failed to open directory");
    return false;
  }

  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return false;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
    } else {
      Serial.print("  FILE: ");
      const char* _tempFileName = file.name();
      Serial.print(_tempFileName);
      Serial.print("\tSIZE: ");
      Serial.println(file.size());

      if ((strcasecmp(_tempFileName, fileRealName) == 0) && (strlen(_tempFileName) == strlen(fileRealName))) {
        memcpy((void*)&_fileName[1], (void*)&_tempFileName, _len);  // copy the real name
        Serial.print("True Flash Name With Directory: ");
        Serial.println(_fileName);
        return true;
      }
    }
    file = root.openNextFile();
  }

  return false;
}

bool AtmegaHexUploader::_writeFlashContents() {
  errors = 0;
  Serial.println("Checking Flash File...");
  if (_chooseInputFile())
    return false;

  Serial.println("Start Programmer...");
  if (!start())
    return false;

  Serial.println("Flash Writing...");
  if (_readHexFile(flashFileName, writeToFlash))
    return false;

  Serial.println("Verifying Flash...");
  if (_readHexFile(flashFileName, verifyFlash))
    return false;

  return errors == 0;
}

void AtmegaHexUploader::_showMessage(const byte which) {
  switch (which) {
    case MSG_NO_SD_CARD: Serial.println("Error: No SD card found"); break;
    case MSG_CANNOT_OPEN_FILE: Serial.println("Error: Cannot open file"); break;
    case MSG_LINE_TOO_LONG: Serial.println("Error: Line too long"); break;
    case MSG_LINE_TOO_SHORT: Serial.println("Error: Line too short"); break;
    case MSG_LINE_DOES_NOT_START_WITH_COLON: Serial.println("Error: Line does not start with colon"); break;
    case MSG_INVALID_HEX_DIGITS: Serial.println("Error: Invalid hex digits"); break;
    case MSG_BAD_SUMCHECK: Serial.println("Error: Bad sumcheck"); break;
    case MSG_LINE_NOT_EXPECTED_LENGTH: Serial.println("Error: Line not expected length"); break;
    case MSG_UNKNOWN_RECORD_TYPE: Serial.println("Error: Unknown record type"); break;
    case MSG_NO_END_OF_FILE_RECORD: Serial.println("Error: No end of file record"); break;
    case MSG_FILE_TOO_LARGE_FOR_FLASH: Serial.println("Error: File too large for flash"); break;
    case MSG_CANNOT_ENTER_PROGRAMMING_MODE: Serial.println("Error: Cannot enter programming mode"); break;
    case MSG_NO_BOOTLOADER_FUSE: Serial.println("Error: No bootloader fuse"); break;
    case MSG_CANNOT_FIND_SIGNATURE: Serial.println("Error: Cannot find signature"); break;
    case MSG_UNRECOGNIZED_SIGNATURE: Serial.println("Error: Unrecognized signature"); break;
    case MSG_BAD_START_ADDRESS: Serial.println("Error: Bad start address"); break;
    case MSG_VERIFICATION_ERROR: Serial.println("Error: Verification error"); break;
    case MSG_FUSE_PROBLEM: Serial.println("Error: Fuse problem"); break;
    case MSG_FLASHED_OK: Serial.println("Success: Flashed OK"); break;
    default: Serial.println("Error: Unknown error"); break;
  }
}

byte AtmegaHexUploader::_softSPITransfer(byte c) {
  byte bit;
  for (bit = 0; bit < 8; bit++) {
    if (c & 0x80)
      digitalWrite(softMosiPin, HIGH);
    else
      digitalWrite(softMosiPin, LOW);
    c <<= 1;
    digitalWrite(softSckPin, HIGH);
    delayMicroseconds(BB_DELAY_MICROSECONDS);
    if (digitalRead(softMisoPin))
      c |= 0x01;
    digitalWrite(softSckPin, LOW);
    delayMicroseconds(BB_DELAY_MICROSECONDS);
  }
  return c;
}

byte AtmegaHexUploader::_program(const byte b1, const byte b2, const byte b3, const byte b4) {
  noInterrupts();
  _softSPITransfer(b1);
  _softSPITransfer(b2);
  _softSPITransfer(b3);
  byte b = _softSPITransfer(b4);
  interrupts();
  return b;
}

byte AtmegaHexUploader::_readFlash(unsigned long addr) {
  byte high = (addr & 1) ? 0x08 : 0;
  addr >>= 1;
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB) {
    _program(loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }
  return _program(readProgramMemory | high, highByte(addr), lowByte(addr));
}

void AtmegaHexUploader::_writeFlash(unsigned long addr, const byte data) {
  byte high = (addr & 1) ? 0x08 : 0;
  addr >>= 1;
  _program(loadProgramMemory | high, 0, lowByte(addr), data);
}

bool AtmegaHexUploader::_hexConv(const char*(&pStr), byte& b) {
  if (!isxdigit(pStr[0]) || !isxdigit(pStr[1])) {
    _showMessage(MSG_INVALID_HEX_DIGITS);
    return true;
  }
  b = *pStr++ - '0';
  if (b > 9)
    b -= 7;
  b <<= 4;
  byte b1 = *pStr++ - '0';
  if (b1 > 9)
    b1 -= 7;
  b |= b1;
  return false;
}

void AtmegaHexUploader::_pollUntilReady() {
  if (currentSignature.timedWrites)
    delay(10);
  else {
    while ((_program(pollReady) & 1) == 1) {}
  }
}

void AtmegaHexUploader::_clearPage() {
  unsigned int len = currentSignature.pageSize;
  for (unsigned int i = 0; i < len; i++)
    _writeFlash(i, 0xFF);
}

void AtmegaHexUploader::_commitPage(unsigned long addr) {
  addr >>= 1;
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB) {
    _program(loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }

  Serial.print(".");
  _program(writeProgramMemory, highByte(addr), lowByte(addr));
  _pollUntilReady();
  _clearPage();
}

void AtmegaHexUploader::_writeData(const unsigned long addr, const byte* pData, const int length) {
  for (int i = 0; i < length; i++) {
    unsigned long thisPage = (addr + i) & pagemask;
    if (thisPage != oldPage && oldPage != NO_PAGE)
      _commitPage(oldPage);
    oldPage = thisPage;
    _writeFlash(addr + i, pData[i]);
  }
}

void AtmegaHexUploader::_verifyData(const unsigned long addr, const byte* pData, const int length) {
  for (int i = 0; i < length; i++) {
    unsigned long thisPage = (addr + i) & pagemask;
    if (thisPage != oldPage && oldPage != NO_PAGE)
      Serial.print(".");
    oldPage = thisPage;
    byte found = _readFlash(addr + i);
    byte expected = pData[i];
    if (found != expected) {
      errors++;
    }
  }
}

bool AtmegaHexUploader::_processLine(const char* pLine, const byte action) {
  if (*pLine == 'F') {
    pLine++;
    byte fuseN = 20;
    byte fuseVal = 6;
    switch (*pLine++) {
      case 'L': fuseN = lowFuse; break;
      case 'H': fuseN = highFuse; break;
      case 'E': fuseN = extFuse; break;
      case 'B': fuseN = lockByte; break;
      default:
        _showMessage(MSG_FUSE_PROBLEM);
        return true;
    }

    switch (action) {
      case writeToFlash:
        if (!_hexConv(pLine, fuseVal)) {
          _writeFuse(fuseVal, fuseCommands[fuseN]);
          return false;
        } else {
          _showMessage(MSG_FUSE_PROBLEM);
          return true;
        }
      default: return false;
    }
  }

  if (*pLine++ != ':') {
    _showMessage(MSG_LINE_DOES_NOT_START_WITH_COLON);
    return true;
  }

  const int maxHexData = 40;
  byte hexBuffer[maxHexData];
  int bytesInLine = 0;

  if (action == checkFile)
    if (lineCount++ % 40 == 0)
      Serial.print(".");

  while (isxdigit(*pLine)) {

    if (bytesInLine >= maxHexData) {
      _showMessage(MSG_LINE_TOO_LONG);
      return true;
    }

    if (_hexConv(pLine, hexBuffer[bytesInLine++]))
      return true;
  }

  if (bytesInLine < 5) {
    _showMessage(MSG_LINE_TOO_SHORT);
    return true;
  }

  byte sumCheck = 0;
  for (int i = 0; i < (bytesInLine - 1); i++)
    sumCheck += hexBuffer[i];

  sumCheck = ~sumCheck + 1;

  if (sumCheck != hexBuffer[bytesInLine - 1]) {
    _showMessage(MSG_BAD_SUMCHECK);
    return true;
  }

  byte len = hexBuffer[0];

  if (len != (bytesInLine - 5)) {
    _showMessage(MSG_LINE_NOT_EXPECTED_LENGTH);
    return true;
  }

  unsigned long addrH = hexBuffer[1];
  unsigned long addrL = hexBuffer[2];

  unsigned long addr = addrL | (addrH << 8);

  byte recType = hexBuffer[3];

  switch (recType) {

    case hexDataRecord:
      lowestAddress = min(lowestAddress, addr + extendedAddress);
      highestAddress = max(lowestAddress, addr + extendedAddress + len - 1);
      bytesWritten += len;

      switch (action) {
        case checkFile:
          break;

        case verifyFlash:
          _verifyData(addr + extendedAddress, &hexBuffer[4], len);
          break;

        case writeToFlash:
          _writeData(addr + extendedAddress, &hexBuffer[4], len);
          break;
      }
      break;

    case hexEndOfFile:
      gotEndOfFile = true;
      break;

    case hexExtendedSegmentAddressRecord:
      extendedAddress = ((unsigned long)hexBuffer[4]) << 12;
      break;

    case hexStartSegmentAddressRecord:
    case hexExtendedLinearAddressRecord:
    case hexStartLinearAddressRecord:
      break;

    default:
      _showMessage(MSG_UNKNOWN_RECORD_TYPE);
      return true;
  }
  return false;
}

bool AtmegaHexUploader::_readHexFile(const char* fName, const byte action) {
  const int maxLine = 80;
  char buffer[maxLine];

  File file = SPIFFS.open(fName, "r");
  if (!file) {
    _showMessage(MSG_CANNOT_OPEN_FILE);
    return true;
  }

  int lineNumber = 0;
  gotEndOfFile = false;
  extendedAddress = 0;
  errors = 0;
  lowestAddress = 0xFFFFFFFF;
  highestAddress = 0;
  bytesWritten = 0;
  progressBarCount = 0;

  pagesize = currentSignature.pageSize;
  pagemask = ~(pagesize - 1);
  oldPage = NO_PAGE;

  switch (action) {
    case checkFile:
      break;

    case verifyFlash:
      break;

    case writeToFlash:
      _program(progamEnable, chipErase);
      delay(20);
      _pollUntilReady();
      _clearPage();
      break;
  }

  while (file.available()) {
    int count = file.readBytesUntil('\n', buffer, maxLine);
    buffer[count] = '\0';
    lineNumber++;

    if (count >= maxLine - 1) {
      _showMessage(MSG_LINE_TOO_LONG);
      file.close();
      return true;
    }

    if (count > 1) {
      if (_processLine(buffer, action)) {
        file.close();
        return true;
      }
    }
  }

  if (!gotEndOfFile) {
    _showMessage(MSG_NO_END_OF_FILE_RECORD);
    file.close();
    return true;
  }

  switch (action) {
    case writeToFlash:
      if (oldPage != NO_PAGE)
        _commitPage(oldPage);
      break;

    case verifyFlash:
      if (errors > 0) {
        _showMessage(MSG_VERIFICATION_ERROR);
        file.close();
        return true;
      }
      break;

    case checkFile:
      break;
  }

  file.close();
  return false;
}

void AtmegaHexUploader::_printHex(byte _hex) {
  Serial.print("0x");
  if (_hex < 10) {
    Serial.print('0');
  }
  Serial.print(_hex, HEX);
}

void AtmegaHexUploader::_writeFuse(const byte newValue, const byte instruction) {
  if (newValue == 0) {
    Serial.println("Info: No need to update Fuse");
    return;
  }

  _program(progamEnable, instruction, 0, newValue);
  _pollUntilReady();
}

bool AtmegaHexUploader::_updateFuses(const bool writeIt) {
  unsigned long addr;
  unsigned int len;

  byte fusenumber = currentSignature.fuseWithBootloaderSize;

  if (fusenumber == NO_FUSE) {
    Serial.println("Info: No need to write Fuse");
    return false;
  }

  addr = currentSignature.flashSize;
  len = currentSignature.baseBootSize;

  if (lowestAddress == 0) {
    fuses[fusenumber] |= 1;
  } else {
    byte newval = 0xFF;

    if (lowestAddress == (addr - len))
      newval = 3;
    else if (lowestAddress == (addr - len * 2))
      newval = 2;
    else if (lowestAddress == (addr - len * 4))
      newval = 1;
    else if (lowestAddress == (addr - len * 8))
      newval = 0;
    else {
      _showMessage(MSG_BAD_START_ADDRESS);
      return true;
    }

    if (newval != 0xFF) {
      newval <<= 1;
      fuses[fusenumber] &= ~0x07;
      fuses[fusenumber] |= newval;
    }
  }

  if (writeIt) {
    _writeFuse(fuses[fusenumber], fuseCommands[fusenumber]);
  }

  return false;
}

bool AtmegaHexUploader::_chooseInputFile() {
  if (_readHexFile(flashFileName, checkFile)) {
    return true;
  }

  if (highestAddress > currentSignature.flashSize) {
    _showMessage(MSG_FILE_TOO_LARGE_FOR_FLASH);
    return true;
  }

  if (_updateFuses(false)) {
    return true;
  }

  return false;
}
#endif