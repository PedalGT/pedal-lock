/**************************************************************************/
/*!
    @file     emulatetag.h
    @author   Armin Wieser
    @license  BSD

    Implemented using NFC forum documents & library of libnfc
*/
/**************************************************************************/

#ifndef __EMULATETAG_H__
#define __EMULATETAG_H__

#include "PN532.h"

// Trace of the last emulate() session, for debugging a phone that connects but
// never gets the NDEF message. Each entry: INS, P1, P2, Le/Lc, SW1, SW2.
#define EMU_TRACE_MAX 24
extern uint8_t emuTrace[EMU_TRACE_MAX][6];
extern uint8_t emuTraceLen;
extern int16_t emuEndStatus;   // tgGetData status that ended the session

#define NDEF_MAX_LENGTH 128 // altough ndef can handle up to 0xfffe in size, arduino cannot.
typedef enum
{
  COMMAND_COMPLETE,
  TAG_NOT_FOUND,
  FUNCTION_NOT_SUPPORTED,
  MEMORY_FAILURE,
  END_OF_FILE_BEFORE_REACHED_LE_BYTES
} responseCommand;

class EmulateTag
{

public:
  EmulateTag(PN532Interface &interface) : pn532(interface), uidPtr(0), tagWrittenByInitiator(false), tagWriteable(true), updateNdefCallback(0) {}

  bool init();

  bool emulate(const uint16_t tgInitAsTargetTimeout = 0);

  // Always-on tag: arms target mode once and returns within ~waitMs. Call it
  // every loop; returns true after a reader (phone) was served.
  bool poll(const uint16_t waitMs = 10);
  bool isArmed() { return armed; }
  int8_t lastPollResult = 0;    // from tgInitAsTargetPoll, or -10 if arming failed
  uint16_t armCount = 0;        // how many times TgInitAsTarget was (re)sent

  /*
   * @param uid pointer to byte array of length 3 (uid is 4 bytes - first byte is fixed) or zero for uid 
   */
  void setUid(uint8_t *uid = 0);

  void setNdefFile(const uint8_t *ndef, const int16_t ndefLength);

  void getContent(uint8_t **buf, uint16_t *length)
  {
    *buf = ndef_file + 2; // first 2 bytes = length
    *length = (ndef_file[0] << 8) + ndef_file[1];
  }

  bool writeOccured()
  {
    return tagWrittenByInitiator;
  }

  void setTagWriteable(bool setWriteable)
  {
    tagWriteable = setWriteable;
  }

  uint8_t *getNdefFilePtr()
  {
    return ndef_file;
  }

  uint8_t getNdefMaxLength()
  {
    return NDEF_MAX_LENGTH;
  }

  void attach(void (*func)(uint8_t *buf, uint16_t length))
  {
    updateNdefCallback = func;
  };

private:
  PN532 pn532;
  uint8_t ndef_file[NDEF_MAX_LENGTH];
  uint8_t *uidPtr;
  bool tagWrittenByInitiator;
  bool tagWriteable;
  void (*updateNdefCallback)(uint8_t *ndef, uint16_t length);

  bool armed = false;
  void buildInitCommand(uint8_t *command);
  bool serveReader();
  void setResponse(responseCommand cmd, uint8_t *buf, uint8_t *sendlen, uint8_t sendlenOffset = 0);
};

#endif
