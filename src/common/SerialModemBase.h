/**
 * @file SerialModemBase.h
 * @brief Common base class for Circuit Design modems (MLR/MU series).
 * @copyright 2026 Circuit Design, Inc.
 */

#pragma once
#include <Arduino.h>

/**
 * @brief Common error codes for modem operations.
 */
enum class ModemError
{
    Ok,             //!< No error
    Busy,           //!< Driver is busy
    InvalidArg,     //!< Invalid argument
    FailLbt,        //!< Carrier sense failed (LBT)
    Fail,           //!< General failure
    BufferTooSmall, //!< Buffer provided is too small
    Timeout         //!< Operation timed out
};

/**
 * @brief Parser results indicating the status of the last parse step.
 */
enum class ModemParseResult
{
    Parsing,             //!< Still parsing, waiting for more data
    Garbage,             //!< Garbage data received and flushed
    Overflow,            //!< Buffer overflow detected
    FinishedCmdResponse, //!< A standard command response was received
    FinishedDrResponse   //!< A data reception message (DR/DS) was received
};

// --- Debug Configuration ---
// To enable debug prints, define ENABLE_SERIAL_MODEM_DEBUG
// #define ENABLE_SERIAL_MODEM_DEBUG

#ifdef ENABLE_SERIAL_MODEM_DEBUG
#define SM_DEBUG_PRINT(...)                              \
    if (this->_debugStream)                              \
    {                                                    \
        this->_debugStream->print(this->getLogPrefix()); \
        this->_debugStream->print(__VA_ARGS__);          \
    }
#define SM_DEBUG_PRINTLN(...)                            \
    if (this->_debugStream)                              \
    {                                                    \
        this->_debugStream->print(this->getLogPrefix()); \
        this->_debugStream->println(__VA_ARGS__);        \
    }
#define SM_DEBUG_PRINTF(...)                             \
    if (this->_debugStream)                              \
    {                                                    \
        this->_debugStream->print(this->getLogPrefix()); \
        this->_debugStream->printf(__VA_ARGS__);         \
    }
#define SM_DEBUG_WRITE(...)                     \
    if (this->_debugStream)                     \
    {                                           \
        this->_debugStream->write(__VA_ARGS__); \
    }
#define SM_DEBUG_PRINT_RAW(...)                 \
    if (this->_debugStream)                     \
    {                                           \
        this->_debugStream->print(__VA_ARGS__); \
    }
#define SM_DEBUG_PRINTLN_RAW(...)                 \
    if (this->_debugStream)                       \
    {                                             \
        this->_debugStream->println(__VA_ARGS__); \
    }
#define SM_DEBUG_PRINTF_RAW(...)                 \
    if (this->_debugStream)                      \
    {                                            \
        this->_debugStream->printf(__VA_ARGS__); \
    }
#else
#define SM_DEBUG_PRINT(...) \
    do                      \
    {                       \
    } while (0)
#define SM_DEBUG_PRINTLN(...) \
    do                        \
    {                         \
    } while (0)
#define SM_DEBUG_PRINTF(...) \
    do                       \
    {                        \
    } while (0)
#define SM_DEBUG_WRITE(...) \
    do                      \
    {                       \
    } while (0)
#define SM_DEBUG_PRINT_RAW(...) \
    do                          \
    {                           \
    } while (0)
#define SM_DEBUG_PRINTLN_RAW(...) \
    do                            \
    {                             \
    } while (0)
#define SM_DEBUG_PRINTF_RAW(...) \
    do                           \
    {                            \
    } while (0)
#endif

/**
 * @brief Base class handling low-level serial I/O, debugging, and common transaction logic.
 */
class SerialModemBase
{
public:
    SerialModemBase();
    virtual ~SerialModemBase() = default;

    /**
     * @brief Initializes the debug stream.
     * @param debugStream Pointer to the Stream for debug output (can be nullptr).
     */
    void setDebugStream(Stream *debugStream);

protected:
    // --- Initialization ---
    /**
     * @brief Sets the UART stream to use for modem communication.
     * @param stream Reference to the serial stream.
     */
    void initSerial(Stream &stream);

    // --- High-Level Transaction Helpers ---

    /**
     * @brief Waits for a command response, handling intervening Data Received events.
     * * This function loops until parse() returns FinishedCmdResponse or timeout occurs.
     * If FinishedDrResponse is returned during the wait, onRxDataReceived() is called
     * and the loop continues (preserving the "async" nature of radio packets).
     * * @param timeoutMs Max time to wait.
     * @return ModemError::Ok on success, Timeout or Fail otherwise.
     */
    ModemError waitForResponse(uint32_t timeoutMs = 500);

    /**
     * @brief Helper to set a 1-byte value (e.g., @CH0E) and verify the response.
     * * Handles the sequence:
     * 1. Send CMD + Value + (Save ? /W : "")
     * 2. Wait for response
     * 3. If Save, handle intermediate "*WR=PS" and wait for final response
     * 4. Parse final response to verify the value matches.
     * * @param cmd Command string (e.g., "@CH").
     * @param value The byte value to set.
     * @param save Whether to save to NVM (/W).
     * @param respPrefix Expected response prefix (e.g., "*CH=").
     * @param respLen Total length of expected response.
     * @return ModemError::Ok if set and verified successfully.
     */
    ModemError setByteValue(const char *cmd, uint8_t value, bool save, const char *respPrefix, size_t respLen);

    /**
     * @brief Helper to get a 1-byte value.
     * * @param cmd Command string (e.g., "@CH").
     * @param pValue Output pointer for the value.
     * @param respPrefix Expected response prefix.
     * @param respLen Total length of expected response.
     * @return ModemError::Ok on success.
     */
    ModemError getByteValue(const char *cmd, uint8_t *pValue, const char *respPrefix, size_t respLen);

    /**
     * @brief Sends a raw command string and fills the provided buffer with the response.
     * * @param command Command string.
     * @param responseBuffer Output buffer.
     * @param bufferSize Size of output buffer.
     * @param timeoutMs Timeout.
     * @return ModemError::Ok on success.
     */
    ModemError sendRawCommand(const char *command, char *responseBuffer, size_t bufferSize, uint32_t timeoutMs);

    // --- Low-Level I/O ---
    void writeString(const char *str, bool printPrefix = true);
    void writeData(const uint8_t *data, size_t len);
    uint8_t readByte();
    void unreadByte(uint8_t c);
    void clearUnreadByte();
    void flushGarbage(char keepChar = '*');

    // --- Timeout Management ---
    void startTimeout(uint32_t ms);
    bool isTimeout();

    // --- Parsing Helpers ---
    // Static helpers for parsing values
    static bool parseHex(const uint8_t *pData, size_t len, uint32_t *pResult);
    static bool parseDec(const uint8_t *pData, size_t len, uint32_t *pResult);

    // Helpers that validate prefix/length from the internal buffer
    ModemError parseResponseHex(const uint8_t *buffer, size_t length, const char *prefix, uint8_t hexDigits, uint32_t *pResult);
    ModemError parseResponseDec(const uint8_t *buffer, size_t length, const char *prefix, const char *suffix, size_t suffixLen, int32_t *pResult);

    // --- Virtual Methods (To be implemented by Derived Classes) ---
    /**
     * @brief Main parser state machine step.
     * Must use readByte() and update _rxBuffer / _rxIndex.
     * @return Current status of parsing.
     */
    virtual ModemParseResult parse() = 0;

    /**
     * @brief Called when waitForResponse() encounters a Data Received event.
     * Derived classes should typically invoke their registered user callback here.
     */
    virtual void onRxDataReceived() = 0;

    /**
     * @brief Returns the log prefix for debug output (e.g., "[MLR]").
     */
    virtual const char *getLogPrefix() const = 0;

protected:
    Stream *_uart = nullptr;
    Stream *_debugStream = nullptr;

    // Shared Receive Buffer
    // Size covers typical command responses and max payload frames (MU max ~300 bytes)
    static constexpr size_t RX_BUFFER_SIZE = 300;
    uint8_t _rxBuffer[RX_BUFFER_SIZE];
    uint16_t _rxIndex = 0;

    // Rx internal state
    int16_t _oneByteBuf = -1;
    bool _debugRxNewLine = true;

private:
    // Timeout state
    bool _bTimeout = true;
    uint32_t _startTime = 0;
    uint32_t _timeOutDuration = 0;
};