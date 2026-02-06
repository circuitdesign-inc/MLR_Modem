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
#include "common/SerialModemBase.h"

/**
 * @brief Default baud rate for the MLR modem.
 */
static constexpr uint32_t MLR_DEFAULT_BAUDRATE = 19200;

/**
 * \brief Represents the type of response received from the modem.
 */
enum class MLR_Modem_Response
{
    Idle,
    ParseError,
    Timeout,
    ShowMode,
    SaveValue,
    Channel,
    SerialNumber,
    MLR_Modem_DtIr,
    DataReceived,
    RssiLastRx,
    RssiCurrentChannel,
    UserID,
    CarrierSenseRssi,
    FactoryReset,
    BaudRate,
    GenericResponse
};

// Use common ModemError for compatibility
using MLR_Modem_Error = ModemError;

/**
 * \brief Wireless communication mode.
 */
enum class MLR_ModemMode : uint8_t
{
    FskBin = 0,
    FskCmd = 1,
    LoRaBin = 2,
    LoRaCmd = 3,
};

/**
 * \brief LoRa modem spreading factor (chip count).
 */
enum class MLR_ModemSpreadFactor : uint8_t
{
    Chips128 = 0,
    Chips256 = 1,
    Chips512 = 2,
    Chips1024 = 3,
    Chips2048 = 4,
    Chips4096 = 5,
};

//! "low-level" internal parser states (MLR specific)
enum class MLR_ModemParserState
{
    Start = 0,
    ReadCmdFirstLetter,
    ReadCmdSecondLetter,
    ReadCmdParam,
    ReadRawString,
    RadioDrSize,
    RadioDrPayload,
    ReadCmdUntilCR,
    ReadCmdUntilLF,
};

/**
 * \brief Callback for asynchronous calls and Radio Message Received events.
 */
typedef void (*MLR_Modem_AsyncCallback)(MLR_Modem_Error error, MLR_Modem_Response responseType, int32_t value, const uint8_t *pPayload, uint16_t len);

/**
 * \brief Main class for interfacing with the MLR Modem.
 */
class MLR_Modem : public SerialModemBase
{
public: // methods
    /**
     * \brief Initializes the modem driver.
     */
    MLR_Modem_Error begin(Stream &pUart, MLR_Modem_AsyncCallback pCallback = nullptr);

    MLR_Modem_Error SetChannel(uint8_t channel, bool saveValue);
    MLR_Modem_Error GetChannel(uint8_t *pChannel);

    MLR_Modem_Error SetMode(MLR_ModemMode mode, bool saveValue);
    MLR_Modem_Error GetMode(MLR_ModemMode *pMode);

    MLR_Modem_Error SetSpreadFactor(MLR_ModemSpreadFactor sf, bool saveValue);
    MLR_Modem_Error GetSpreadFactor(MLR_ModemSpreadFactor *pSf);

    MLR_Modem_Error SetEquipmentID(uint8_t ei, bool saveValue);
    MLR_Modem_Error GetEquipmentID(uint8_t *pEI);

    MLR_Modem_Error SetDestinationID(uint8_t di, bool saveValue);
    MLR_Modem_Error GetDestinationID(uint8_t *pDI);

    MLR_Modem_Error SetGroupID(uint8_t gi, bool saveValue);
    MLR_Modem_Error GetGroupID(uint8_t *pGI);

    MLR_Modem_Error GetUserID(uint16_t *pUserID);

    MLR_Modem_Error GetRssiLastRx(int16_t *pRssi);
    MLR_Modem_Error GetRssiCurrentChannel(int16_t *pRssi);

    MLR_Modem_Error SetCarrierSenseRssiOutput(uint8_t ciValue, bool saveValue);
    MLR_Modem_Error GetCarrierSenseRssiOutput(uint8_t *pCiValue);

    MLR_Modem_Error GetSerialNumber(uint32_t *pSn);
    MLR_Modem_Error FactoryReset();

    MLR_Modem_Error GetBaudRate(uint8_t *pBaudRate);
    MLR_Modem_Error SetBaudRate(uint32_t baudRate, bool saveValue);

    MLR_Modem_Error SendRawCommand(const char *command, char *responseBuffer, size_t bufferSize, uint32_t timeoutMs = 500);
    MLR_Modem_Error SendRawCommandAsync(const char *command, uint32_t timeoutMs = 500);

    MLR_Modem_Error TransmitData(const uint8_t *pMsg, uint8_t len);
    MLR_Modem_Error TransmitDataFireAndForget(const uint8_t *pMsg, uint8_t len);

    MLR_Modem_Error GetRssiCurrentChannelAsync();
    MLR_Modem_Error GetSerialNumberAsync();

    MLR_Modem_Error GetPacket(const uint8_t **ppData, uint8_t *len);

    void SetAsyncCallback(MLR_Modem_AsyncCallback pCallback) { m_pCallback = pCallback; }

    bool HasPacket() { return m_drMessagePresent; }
    void DeletePacket() { m_drMessagePresent = false; }

    void Work();

protected:
    // --- SerialModemBase Virtual Overrides ---
    ModemParseResult parse() override;
    void onRxDataReceived() override;
    const char *getLogPrefix() const override { return "[MLR"; }

private: // methods
    // Internal parser state machine function (Implemented inside parse())

    // Internal: Dispatches a received command response to the async callback
    MLR_Modem_Error m_DispatchCmdResponseAsync();

    // Internal: Handles the "*WR=PS" response (Now handled mostly by Base, kept for specific flows if needed)
    // MLR_Modem_Error m_HandleMessage_WR(); // Removed, used Base implementation

    // Internal helpers using Base methods
    MLR_Modem_Error m_HandleMessageHexByte(uint8_t *pValue, uint32_t responseLen, const char *responsePrefix);
    MLR_Modem_Error m_HandleMessageHexWord(uint16_t *pValue, uint32_t responseLen, const char *responsePrefix);

    MLR_Modem_Error m_HandleMessage_RS(int16_t *pRssi);
    MLR_Modem_Error m_HandleMessage_RA(int16_t *pRssi);
    MLR_Modem_Error m_HandleMessage_SN(uint32_t *pSerialNumber);
    MLR_Modem_Error m_HandleMessage_IZ();

    void m_ClearOneLine();

private: // data
    // _uart and _debugStream are in Base class
    MLR_Modem_Response m_asyncExpectedResponse; //!< The expected response for an async call
    MLR_ModemParserState m_parserState;         //!< Current state of the parser

    // _rxBuffer and _rxIndex are in Base class

    // special receive buffer and data for '@DR' command
    bool m_drMessagePresent;             //!< Flag indicating a *DR packet is ready
    uint8_t m_drMessageLen;              //!< Length of the received *DR packet
    uint8_t m_drMessage[300];            //!< Buffer for the received *DR packet payload
    MLR_ModemMode m_mode;                //!< Cached modem mode
    MLR_Modem_AsyncCallback m_pCallback; //!< Pointer to the user's callback function
};