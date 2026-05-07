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
//

#pragma once
#include <Arduino.h>
#include <SerialModemBase.h>

/**
 * @brief Default baud rate for the MLR modem.
 */
static constexpr uint32_t MLR_DEFAULT_BAUDRATE = 19200;

/**
 * @brief Channel range constants (429MHz JP band).
 */
static constexpr uint8_t MLR_CHANNEL_MIN_429 = 0x07; //!< Minimum channel number for 429MHz
static constexpr uint8_t MLR_CHANNEL_MAX_429 = 0x2E; //!< Maximum channel number for 429MHz

/**
 * \brief Represents the type of response received from the modem.
 */
enum class MLR_Modem_Response
{
    // --- Internal state ---
    Idle,       //!< No message received or expected
    ParseError, //!< Garbage characters received
    Timeout,    //!< No response received

    // --- TX result (aligned with MU_Modem_Response) ---
    TxComplete, //!< Transmission successful (*IR=03 received after @DT)
    TxFailed,   //!< Transmission failed (*IR=01 or *IR=02 received after @DT)

    // --- Data reception ---
    DataReceived, //!< Data received from another modem ("*DR=..."). event.value carries
                  //!< RSSI (dBm) of the packet, retrieved automatically via internal @RS.
                  //!< value=0 if RSSI could not be obtained (e.g. another async command
                  //!< was in-flight when the packet arrived).

    // --- Command responses (common with MU_Modem_Response) ---
    ShowMode,           //!< Response to "@MO" (e.g., "FSK MODE", "LORA MODE")
    SaveValue,          //!< Response to NVM save ("*WR=PS")
    Channel,            //!< Response to "@CH" (frequency channel)
    SerialNumber,       //!< Response to "@SN" (serial number)
    GroupID,            //!< Response to "@GI" (group ID)
    EquipmentID,        //!< Response to "@EI" (equipment ID)
    DestinationID,      //!< Response to "@DI" (destination ID)
    RssiCurrentChannel, //!< Response to "@RA" (current channel RSSI)

    // --- MLR-specific responses ---
    RssiLastRx,       //!< Response to "@RS" (RSSI of last received packet)
    UserID,           //!< Response to "@UI" (user ID)
    CarrierSenseRssi, //!< Response to "@CI" (carrier sense RSSI output)
    FactoryReset,     //!< Response to "@IZ" (*IZ=OK)
    BaudRate,         //!< Response to "@BR" (UART baud rate)

    // --- Generic ---
    GenericResponse //!< Generic response from SendRawCommandAsync
};

// Use common ModemError for compatibility
using MLR_Modem_Error = ModemError;

/**
 * \brief Wireless communication mode.
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

//! "low-level" internal parser states (MLR specific)
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
};

/**
 * \brief Represents an event from the modem.
 */
struct MLR_Modem_Event
{
    MLR_Modem_Error error;   //!< Error code
    MLR_Modem_Response type; //!< Type of response
    int32_t value;           //!< Numerical value associated with the response
    const uint8_t *pPayload; //!< Pointer to payload data (e.g., for DataReceived)
    uint16_t payloadLen;     //!< Length of payload data

    // --- Constructors ---
    // 1. Default
    MLR_Modem_Event() : error(ModemError::Ok), type(MLR_Modem_Response::Idle), value(0), pPayload(nullptr), payloadLen(0) {}

    // 2. Helper for simple status events
    MLR_Modem_Event(ModemError err, MLR_Modem_Response t)
        : error(err), type(t), value(0), pPayload(nullptr), payloadLen(0) {}

    // 3. Helper for events with a value (RSSI, SN, etc.)
    MLR_Modem_Event(ModemError err, MLR_Modem_Response t, int32_t val)
        : error(err), type(t), value(val), pPayload(nullptr), payloadLen(0) {}

    // 4. Helper for data reception
    MLR_Modem_Event(ModemError err, MLR_Modem_Response t, int32_t val, const uint8_t *p, uint16_t l)
        : error(err), type(t), value(val), pPayload(p), payloadLen(l) {}
};

/**
 * \brief Callback for asynchronous calls and Radio Message Received events.
 */
typedef void (*MLR_Modem_AsyncCallback)(const MLR_Modem_Event &event);

/**
 * \brief Main class for interfacing with the MLR Modem.
 */
class MLR_Modem : public SerialModemBase
{
public: // methods
    MLR_Modem() : SerialModemBase("[MLR Modem] ") {}

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
     * \brief Transmits data over the wireless link asynchronously.
     * The result will be delivered via the AsyncCallback as
     * MLR_Modem_Response::TxComplete on success, or MLR_Modem_Response::TxFailed
     * on LBT/transmission failure.
     * \param pMsg Pointer to the data payload to send.
     * \param len Length of the data payload (0-255 bytes).
     * \return MLR_Modem_Error::Ok if the command was sent, MLR_Modem_Error::Busy if another async operation is pending.
     */
    MLR_Modem_Error TransmitDataAsync(const uint8_t *pMsg, uint8_t len);

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
     * \brief Checks if a new radio packet has been received.
     * \return true if a packet is available, false otherwise.
     */
    bool HasPacket() { return m_drMessagePresent; }

    /**
     * \brief Deletes the currently stored received packet.
     */
    void DeletePacket() { m_drMessagePresent = false; }

    /**
     * \brief Performs a software reset of the modem.
     * \return MLR_Modem_Error::Ok on success.
     * \note Uses the "@SR" command.
     */
    MLR_Modem_Error SoftReset();

    /**
     * \brief Main processing loop for the driver.
     * This function must be called regularly (e.g., in the Arduino loop())
     * to parse incoming serial data from the modem.
     * Now calls SerialModemBase::update() internally.
     */
    void Work() { update(); }

protected:
    // --- SerialModemBase Virtual Overrides ---
    ModemParseResult parse() override;
    void onRxDataReceived() override;
    void onCommandComplete(ModemError result) override;

private: // methods
    // Internal parser state machine handlers
    ModemParseResult m_HandleReadStart();
    ModemParseResult m_HandleReadCmdFirstLetter();
    ModemParseResult m_HandleReadCmdSecondLetter();
    ModemParseResult m_HandleReadCmdParam();
    ModemParseResult m_HandleRadioDrSize();
    ModemParseResult m_HandleRadioDrPayload();
    ModemParseResult m_HandleReadCmdUntilCR();
    ModemParseResult m_HandleReadCmdUntilLF();

    // Internal: Dispatches an event to the async callback
    void dispatchAsyncEvent(ModemError error, MLR_Modem_Response responseType, int32_t value = 0, const uint8_t *pPayload = nullptr, uint16_t len = 0);

    // Internal helpers using Base methods
    //! Internal: Helper method for responses that contain a one-byte hex value (e.g., *CH=0E)
    ModemError m_HandleMessageHexByte(uint8_t *pValue, uint32_t responseLen, const char *responsePrefix);
    //! Internal: Helper method for responses that contain a two-byte hex value (e.g., *UI=0000)
    ModemError m_HandleMessageHexWord(uint16_t *pValue, uint32_t responseLen, const char *responsePrefix);

    //! Internal: Handles the "*RS=...dBm" response
    ModemError m_HandleMessage_RS(int16_t *pRssi);
    //! Internal: Handles the "*RA=...dBm" response
    ModemError m_HandleMessage_RA(int16_t *pRssi);

    //! Internal: Handles the "*SN=..." response
    ModemError m_HandleMessage_SN(uint32_t *pSerialNumber);
    // check if the received message is "*IZ=OK"
    ModemError m_HandleMessage_IZ();

private:                                        // data
    MLR_Modem_Response m_asyncExpectedResponse; //!< The expected response for an async call
    MLR_ModemParserState m_parserState;         //!< Current state of the parser

    // special receive buffer and data for '@DR' command
    bool m_drMessagePresent;  //!< Flag indicating a *DR packet is ready
    uint8_t m_drMessageLen;   //!< Length of the received *DR packet
    uint8_t m_drMessage[300]; //!< Buffer for the received *DR packet payload

    // information response (*IR=...)
    bool m_irMessagePresent; //!< Flag indicating an *IR response is ready
    uint8_t m_irValue;       //!< The value of the *IR response

    // Auto-RSSI-on-RX state: an internal "@RS" is issued after each *DR
    // reception. While true, DataReceived dispatch is deferred until *RS=
    // arrives so the RSSI value can be attached. Mirrors MU_Modem's
    // hardware-appended RSSI behavior (value=dBm in DataReceived events).
    bool m_autoRssiPending;

    MLR_ModemMode m_mode;                //!< Cached modem mode
    MLR_Modem_AsyncCallback m_pCallback; //!< Pointer to the user's callback function
};