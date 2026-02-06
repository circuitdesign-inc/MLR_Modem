//
// MLR_Modem.h
//
// The original program
// (c) 2019 Reimesch Kommunikationssysteme
// Authors: aj, cl
// Created on: 13.03.2019
// Released under the MIT license
//
// (c) 2026 CircuitDesign,Inc.
// Interface driver to Circuit Design SLR/MLR modems.
// The driver functionality only covers the serial interface - no I/O, Reset etc.

#pragma once
#include <Arduino.h>

/**
 * @brief Default baud rate for the MLR modem.
 */
static constexpr uint32_t MLR_DEFAULT_BAUDRATE = 19200;

// --- Debug Configuration ---
// To enable debug prints for this library, define ENABLE_MLR_MODEM_DEBUG
// Uncomment the following line to enable debug output
// #define ENABLE_MLR_MODEM_DEBUG

#ifdef ENABLE_MLR_MODEM_DEBUG
#define MLR_DEBUG_PRINT(...) \
    if (m_pDebugStream)      \
    m_pDebugStream->print(__VA_ARGS__)
#define MLR_DEBUG_PRINTLN(...) \
    if (m_pDebugStream)        \
    m_pDebugStream->println(__VA_ARGS__)
#define MLR_DEBUG_PRINTF(...) \
    if (m_pDebugStream)       \
    m_pDebugStream->printf(__VA_ARGS__)
#define MLR_DEBUG_WRITE(...) \
    if (m_pDebugStream)      \
    m_pDebugStream->write(__VA_ARGS__)
#else
#define MLR_DEBUG_PRINT(...)
#define MLR_DEBUG_PRINTLN(...)
#define MLR_DEBUG_PRINTF(...)
#define MLR_DEBUG_WRITE(...)
#endif

/**
 * \brief Represents the type of response received from the modem.
 * Used internally to identify incoming messages.
 */
enum class MLR_Modem_Response
{
    // internal state of modem
    Idle,       //!< No message received or expected
    ParseError, //!< Garbage characters Received
    Timeout,    //!< No response received

    // serial commands
    ShowMode,           //!< Response to "@MO" (e.g., "FSK MODE", "LORA MODE")
    SaveValue,          //!< Response to saving a value ("*WR=PS")
    Channel,            //!< Response to "@CH" (Set frequency channel)
    SerialNumber,       //!< Response to "@SN" (Acquire serial number)
    MLR_Modem_DtIr,     //!< Information Response after "@DT" (LoRa only, e.g., *IR=03)
    DataReceived,       //!< Data received from another modem ("*DR=...")
    RssiLastRx,         //!< Response to "@RS" (Acquire RSSI for last reception)
    RssiCurrentChannel, //!< Response to "@RA" (Acquire current RSSI)
    UserID,             //!< "*UI=..." : Acquire User ID
    CarrierSenseRssi,   //!< "*CI=..." : Get/Set Carrier Sense RSSI Output
    FactoryReset,       //!< "*IZ=OK" : Factory Reset
    BaudRate,           //!< "*BR=..." : Get/Set UART Baud Rate
    GenericResponse     //!< Generic response from SendRawCommandAsync
};

/**
 * \brief API level error codes for driver functions.
 */
enum class MLR_Modem_Error
{
    Ok,            //!< No error
    Busy,          //!< Driver is busy waiting for another response
    InvalidArg,    //!< Command has invalid argument
    FailLbt,       //!< Transmit failed because of LBT (Listen Before Talk) / Carrier Sense
    Fail,          //!< A general error occurred
    BufferTooSmall //!< Provided response buffer is too small
};

/**
 * \brief Wireless communication mode.
 * \note Corresponds to the "@MO" command.
 */
enum class MLR_ModemMode : uint8_t
{
    FskBin = 0,  //!< FSK Binary Mode (Not supported by this driver)
    FskCmd = 1,  //!< FSK Command Mode
    LoRaBin = 2, //!< LoRa Binary Mode (Not supported by this driver)
    LoRaCmd = 3, //!< LoRa Command Mode
};

/**
 * \brief LoRa modem spreading factor (chip count).
 * \note Corresponds to the "@SF" command.
 */
enum class MLR_ModemSpreadFactor : uint8_t
{
    Chips128 = 0,  //!< 128 chips (SF 7)
    Chips256 = 1,  //!< 256 chips (SF 8)
    Chips512 = 2,  //!< 512 chips (SF 9)
    Chips1024 = 3, //!< 1024 chips (SF 10)
    Chips2048 = 4, //!< 2048 chips (SF 11)
    Chips4096 = 5, //!< 4096 chips (SF 12)
};

//! "high-level" internal command parser state
enum class MLR_ModemCmdState
{
    Parsing = 0,         //!< Still parsing, waiting for further input
    Garbage,             //!< Garbage received
    Overflow,            //!< Too many characters received
    FinishedCmdResponse, //!< Received a command that might be syntactically correct
    FinishedDrResponse,  //!< Received a data reception response (*DR)
};

//! "low-level" internal parser states
enum class MLR_ModemParserState
{
    Start = 0,

    ReadCmdFirstLetter,  //!< First char had been '*', now read first letter of command
    ReadCmdSecondLetter, //!< Now read second letter of command
    ReadCmdParam,        //!< So far '*XX' has been read, now read param of command (might be started by '=')

    ReadRawString, //!< Reading a raw string (e.g., "LORA MODE")

    RadioDrSize,    //!< Special case for *DR telegram -> wait for length information
    RadioDrPayload, //!< Wait for payload data to finish

    ReadCmdUntilCR, //!< Wait for '\r' at end of command
    ReadCmdUntilLF, //!< Wait for '\n' at end of command

    /* TODO discuss stay at one level of parser state
   Garbage,              // garbage received
   Overflow,             // too many characters received
   FinishedCmdResponse,  // received a command that might be syntactically correct
   FinishedDrResponse,   // received a dr response
   */
};

/**
 * \brief Callback for asynchronous calls and Radio Message Received events.
 * \param error - Status of the received response. If value != MLR_Modem_Error::Ok, all following fields are invalid.
 * \param responseType - Type of the received response (e.g., DataReceived, RssiCurrentChannel).
 * \param value - The returned value (e.g., RSSI value), or 0 if not applicable.
 * \param pPayload - Pointer to the received payload. For `MLR_Modem_Response::DataReceived`, this is the radio payload. For `MLR_Modem_Response::GenericResponse`, this is the raw response string.
 *                   \note The data pointed to by `pPayload` is only valid within the scope of this callback function, as it points to an internal library buffer. If you need to access the data after the callback has returned, you must copy it to your own buffer.
 * \param len - The length of the received payload.
 *
 */
typedef void (*MLR_Modem_AsyncCallback)(MLR_Modem_Error error, MLR_Modem_Response responseType, int32_t value, const uint8_t *pPayload, uint16_t len);

/**
 * \brief Main class for interfacing with the MLR Modem.
 */
class MLR_Modem
{

public: // methods
    /**
     * \brief Initializes the modem driver.
     * \param pUart The Serial port connected to the modem.
     * \param pCallback The function to call for async responses and received data.
     * \return MLR_Modem_Error::Ok on success.
     */
    MLR_Modem_Error begin(Stream &pUart, MLR_Modem_AsyncCallback pCallback = nullptr);

    /**
     * \brief Sets the frequency channel.
     * \param channel The channel to set (0x07 - 0x2E).
     * \param saveValue If true, saves the setting to non-volatile memory (/W option).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@CH" command.
     */
    MLR_Modem_Error SetChannel(uint8_t channel, bool saveValue);

    /**
     * \brief Gets the current frequency channel.
     * \param pChannel Pointer to store the current channel (0x07 - 0x2E).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@CH" command.
     */
    MLR_Modem_Error GetChannel(uint8_t *pChannel);

    /**
     * \brief Sets the wireless communication mode (e.g., FSK or LoRa).
     * \param mode The mode to set (e.g., LoRaCmd).
     * \param saveValue If true, saves the setting to non-volatile memory (/W option).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@MO" command.
     */
    MLR_Modem_Error SetMode(MLR_ModemMode mode, bool saveValue);

    /**
     * \brief Gets the current wireless communication mode.
     * \param pMode Pointer to store the current mode.
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@MO" command.
     */
    MLR_Modem_Error GetMode(MLR_ModemMode *pMode);

    /**
     * \brief Sets the LoRa spreading factor.
     * \param sf The spreading factor to set.
     * \param saveValue If true, saves the setting to non-volatile memory (/W option).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@SF" command.
     */
    MLR_Modem_Error SetSpreadFactor(MLR_ModemSpreadFactor sf, bool saveValue);

    /**
     * \brief Gets the current LoRa spreading factor.
     * \param pSf Pointer to store the current spreading factor.
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@SF" command.
     */
    MLR_Modem_Error GetSpreadFactor(MLR_ModemSpreadFactor *pSf);

    /**
     * \brief Sets the Equipment ID (self ID).
     * \param ei The Equipment ID to set (0x00 - 0xFF).
     * \param saveValue If true, saves the setting to non-volatile memory (/W option).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@EI" command.
     */
    MLR_Modem_Error SetEquipmentID(uint8_t ei, bool saveValue);

    /**
     * \brief Gets the Equipment ID (self ID).
     * \param pEI Pointer to store the current Equipment ID (0x00 - 0xFF).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@EI" command.
     */
    MLR_Modem_Error GetEquipmentID(uint8_t *pEI);

    /**
     * \brief Sets the Destination ID.
     * \param di The Destination ID to set (0x00 - 0xFF). (0x00 is broadcast)
     * \param saveValue If true, saves the setting to non-volatile memory (/W option).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@DI" command.
     */
    MLR_Modem_Error SetDestinationID(uint8_t di, bool saveValue);

    /**
     * \brief Gets the Destination ID.
     * \param pDI Pointer to store the current Destination ID (0x00 - 0xFF).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@DI" command.
     */
    MLR_Modem_Error GetDestinationID(uint8_t *pDI);

    /**
     * \brief Sets the Group ID.
     * \param gi The Group ID to set (0x00 - 0xFF).
     * \param saveValue If true, saves the setting to non-volatile memory (/W option).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@GI" command.
     */
    MLR_Modem_Error SetGroupID(uint8_t gi, bool saveValue);

    /**
     * \brief Gets the Group ID.
     * \param pGI Pointer to store the current Group ID (0x00 - 0xFF).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@GI" command.
     */
    MLR_Modem_Error GetGroupID(uint8_t *pGI);

    /**
     * \brief Gets the User ID.
     * \param pUserID Pointer to store the current User ID (0x0000 - 0xFFFF).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@UI" command.
     */
    MLR_Modem_Error GetUserID(uint16_t *pUserID);

    /**
     * \brief Gets the RSSI (Received Signal Strength) of the last successfully received packet.
     * \param pRssi Pointer to store the RSSI value in dBm.
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@RS" command.
     */
    MLR_Modem_Error GetRssiLastRx(int16_t *pRssi);

    /**
     * \brief Gets the current RSSI (noise floor) of the configured channel.
     * \param pRssi Pointer to store the RSSI value in dBm.
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@RA" command.
     */
    MLR_Modem_Error GetRssiCurrentChannel(int16_t *pRssi);

    /**
     * \brief Sets the Carrier Sense RSSI Output setting.
     * \param ciValue The setting to set ('00' = OFF, '01' = ON).
     * \param saveValue If true, saves the setting to non-volatile memory (/W option).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@CI" command.
     */
    MLR_Modem_Error SetCarrierSenseRssiOutput(uint8_t ciValue, bool saveValue);

    /**
     * \brief Gets the Carrier Sense RSSI Output setting.
     * \param pCiValue Pointer to store the current setting ('00' = OFF, '01' = ON).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@CI" command.
     */
    MLR_Modem_Error GetCarrierSenseRssiOutput(uint8_t *pCiValue);

    /**
     * \brief Gets the modem's serial number.
     * \param pSn Pointer to store the serial number.
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@SN" command.
     */
    MLR_Modem_Error GetSerialNumber(uint32_t *pSn);

    /**
     * \brief Resets the modem to factory settings.
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@IZ" command.
     */
    MLR_Modem_Error FactoryReset();

    /**
     * \brief Gets the UART Baud Rate setting.
     * \param pBaudRate Pointer to store the current baud rate code (e.g., '19' for 19200).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@BR" command.
     */
    MLR_Modem_Error GetBaudRate(uint8_t *pBaudRate);

    /**
     * \brief Sets the UART Baud Rate.
     * \param baudRate The baud rate to set (e.g., 9600, 19200).
     * \param saveValue If true, saves the setting to non-volatile memory (/W option).
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@BR" command.
     */
    MLR_Modem_Error SetBaudRate(uint32_t baudRate, bool saveValue);

    /**
     * \brief Sends a raw command string and waits synchronously for a response.
     * \param command The null-terminated command string (e.g., "@FV\r\n").
     * \param responseBuffer Buffer to store the raw response line (excluding CRLF).
     * \param bufferSize Size of the responseBuffer.
     * \param timeoutMs Timeout in milliseconds to wait for the response.
     * \return MLR_Modem_Error::Ok on success, or an error code on failure.
     */
    MLR_Modem_Error SendRawCommand(const char *command, char *responseBuffer, size_t bufferSize, uint32_t timeoutMs = 500);

    /**
     * \brief Sends a raw command string asynchronously.
     * The response will be delivered via the AsyncCallback as MLR_Modem_Response::GenericResponse.
     * \param command The null-terminated command string (e.g., "@FV\r\n").
     * \param timeoutMs Timeout in milliseconds to wait for the response.
     * \return MLR_Modem_Error::Ok if the command was sent, or an error code.
     */
    MLR_Modem_Error SendRawCommandAsync(const char *command, uint32_t timeoutMs = 500);

    /**
     * \brief Transmits data over the wireless link.
     * \param pMsg Pointer to the data payload to send.
     * \param len Length of the data payload (0-255 bytes).
     * \return MLR_Modem_Error::Ok on success, MLR_Modem_Error::FailLbt if carrier sense fails.
     * \note Uses the "@DT" command.
     */
    MLR_Modem_Error TransmitData(const uint8_t *pMsg, uint8_t len);

    /**
     * \brief Transmits data over the wireless link without waiting for transmission completion (*IR).
     * \param pMsg Pointer to the data payload to send.
     * \param len Length of the data payload (0-255 bytes).
     * \return MLR_Modem_Error::Ok on success (command accepted), MLR_Modem_Error::Busy if driver is busy.
     */
    MLR_Modem_Error TransmitDataFireAndForget(const uint8_t *pMsg, uint8_t len);

    /**
     * \brief Asynchronously requests the current RSSI of the configured channel.
     * The result will be delivered via the AsyncCallback.
     * \return MLR_Modem_Error::Ok if the request was sent, MLR_Modem_Error::Busy if another async operation is pending.
     * \note Uses the "@RA" command.
     */
    MLR_Modem_Error GetRssiCurrentChannelAsync();

    // /**
    //  * \brief Gets the contact function for DIO1..DIO8.
    //  */
    // MLR_Modem_Error GetContactFunction(uint8_t *pContactFunction);

    // /**
    //  * \brief Sets the contact function for DIO1..DIO8.
    //  */
    // MLR_Modem_Error SetContactFunction(uint8_t contactFunction, bool saveValue);

    /**
     * \brief Asynchronously requests the modem's serial number.
     * The result will be delivered via the AsyncCallback.
     * \return MLR_Modem_Error::Ok if the request was sent, MLR_Modem_Error::Busy if another async operation is pending.
     * \note Uses the "@SN" command.
     */
    MLR_Modem_Error GetSerialNumberAsync();

    /**
     * \brief Retrieves the last received packet.
     * \param ppData Pointer to a const uint8_t* that will be set to the packet data.
     *               \note The pointer `*ppData` will point to an internal library buffer. This pointer is only valid until the next call to `Work()` or `DeletePacket()`. If you need to access the data later, you must copy it to your own buffer.
     * \param len Pointer to a uint8_t that will be set to the packet length.
     * \return MLR_Modem_Error::Ok on success, MLR_Modem_Error::Fail if no packet is available.
     * \note This function does not remove the packet. Use DeletePacket() to clear it.
     */
    MLR_Modem_Error GetPacket(const uint8_t **ppData, uint8_t *len);

    /**
     * \brief Sets the asynchronous callback function.
     * \param pCallback The callback function. If set to nullptr, no callback will take place.
     */
    void SetAsyncCallback(MLR_Modem_AsyncCallback pCallback) { m_pCallback = pCallback; }

    /**
     * \brief Sets the stream for debug output.
     * \param debugStream Pointer to the Stream object (e.g., &Serial).
     */
    void setDebugStream(Stream *debugStream);

    /**
     * \brief Checks if a new radio packet has been received.
     * \return true if a packet is available, false otherwise.
     */
    bool HasPacket() { return m_drMessagePresent; }

    /**
     * \brief Deletes the currently stored received packet.
     */
    void DeletePacket() { m_drMessagePresent = false; }

    /**
     * \brief Main processing loop for the driver.
     * This function must be called regularly (e.g., in the Arduino loop())
     * to parse incoming serial data from the modem.
     */
    void Work();

private: // methods
    //! Internal: mock-up for timeout check
    bool m_IsTimeout()
    {
        if (!bTimeout && millis() - startTime > timeOut)
        {
            bTimeout = true;
        }
        return bTimeout;
    }
    bool bTimeout = true; //!< Internal timeout flag
    uint32_t startTime;   //!< Internal timeout start time
    uint32_t timeOut;     //!< Internal timeout duration

    //! Internal: mock-up for restarting timeout counter
    void m_StartTimeout(uint32_t ms = 500)
    {
        bTimeout = false;
        startTime = millis();
        timeOut = ms;
    }

    //! Internal: write string to UART
    void m_WriteString(const char *pString, bool printPrefix = true);

    //! Internal: write binary data to UART
    void m_WriteData(const uint8_t *pData, uint8_t len);

    //! Internal: methods for reading from UART, using a one-byte buffer
    uint8_t m_ReadByte();
    void m_UnreadByte(uint8_t unreadByte);
    void m_ClearUnreadByte();
    uint32_t m_Read(uint8_t *pDst, uint32_t count);

    //! Internal: Set the parser to the initial state
    void m_ResetParser();

    //! Internal: Flushes garbage from the serial buffer
    void m_FlushGarbage();

    //! Internal: Main parser state machine function
    MLR_ModemCmdState m_Parse();

    //! Internal: Waits for a standard command response (e.g., *CH=...)
    MLR_Modem_Error m_WaitCmdResponse(uint32_t ms = 500);

    //! Internal: Sets the expected async responses
    void m_SetExpectedResponses(MLR_Modem_Response ep0, MLR_Modem_Response ep1, MLR_Modem_Response ep2);

    //! Internal: Dispatches a received command response to the async callback
    MLR_Modem_Error m_DispatchCmdResponseAsync();

    //! Internal: Handles the "*WR=PS" response
    MLR_Modem_Error m_HandleMessage_WR();

    //! Internal: Helper to set a byte value and verify response
    MLR_Modem_Error m_SetByteValue(const char *cmdPrefix, uint8_t value, bool saveValue, const char *respPrefix, size_t respLen);

    //! Internal: Helper to send a command and wait for response
    MLR_Modem_Error m_SendCmd(const char *cmd);

    //! Internal: Helper to get a byte value from the modem
    MLR_Modem_Error m_GetByteValue(const char *cmdString, uint8_t *pValue, const char *respPrefix, size_t respLen);

    //! Internal: Generic parser for Hex values
    MLR_Modem_Error m_ParseResponseHex(uint32_t *pValue, const char *prefix, size_t prefixLen, uint8_t hexDigits);

    //! Internal: Generic parser for Decimal values with suffix
    MLR_Modem_Error m_ParseResponseDec(int16_t *pValue, const char *prefix, size_t prefixLen, const char *suffix, size_t suffixLen);

    //! Internal: Helper method for responses that contain a one-byte hex value (e.g., *CH=0E)
    MLR_Modem_Error m_HandleMessageHexByte(uint8_t *pValue, uint32_t responseLen, const char *responsePrefix);

    //! Internal: Helper method for responses that contain a two-byte hex value (e.g., *UI=0000)
    MLR_Modem_Error m_HandleMessageHexWord(uint16_t *pValue, uint32_t responseLen, const char *responsePrefix);

    //! Internal: Handles the "*RS=...dBm" response
    MLR_Modem_Error m_HandleMessage_RS(int16_t *pRssi);

    //! Internal: Handles the "*RA=...dBm" response
    MLR_Modem_Error m_HandleMessage_RA(int16_t *pRssi);

    //! Internal: Handles the "*SN=..." response
    MLR_Modem_Error m_HandleMessage_SN(uint32_t *pSerialNumber);

    // check if the received message is "*IZ=OK"
    MLR_Modem_Error m_HandleMessage_IZ();

    //! Internal: Clears one line (up to \n) from the serial buffer
    void m_ClearOneLine();

private:                                            // data
    Stream *m_pUart;                                //!< Pointer to the Arduino serial port
    Stream *m_pDebugStream = nullptr;               //!< Pointer to the stream for debug output.
    bool m_debugRxNewLine = true;                   //!< Flag to track if we are at start of RX line
    MLR_Modem_Response m_asyncExpectedResponse;     //!< The expected response for an async call
    MLR_Modem_Response m_asyncExpectedResponses[3]; //!< Array of possible expected responses
    MLR_ModemParserState m_parserState;             //!< Current state of the parser

    // receive buffer and index for modem response / data reception
    int16_t m_oneByteBuf;    //!< 1-byte buffer for m_UnreadByte()
    uint16_t m_rxIdx;        //!< Current index in the m_rxMessage buffer
    uint8_t m_rxMessage[32]; //!< Buffer for standard command responses (e.g., *CH=0E)

    // special receive buffer and data for '@DR' command
    bool m_drMessagePresent;             //!< Flag indicating a *DR packet is ready
    uint8_t m_drMessageLen;              //!< Length of the received *DR packet
    uint8_t m_drMessage[300];            //!< Buffer for the received *DR packet payload
    MLR_ModemMode m_mode;                //!< Cached modem mode
    MLR_Modem_AsyncCallback m_pCallback; //!< Pointer to the user's callback function
};