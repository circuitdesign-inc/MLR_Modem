//
// MLR_Modem.cpp
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

#include "MLR_Modem.h"
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <array>

// string codes for SLR/MLR modem
// @W (Write to NVM) -> Handled by Base class (CD_WRITE_OK_RESPONSE)

// @CH (Channel Frequency)
static constexpr char MLR_CMD_CHANNEL[] = "@CH";
static constexpr char MLR_SET_CHANNEL_RESPONSE_PREFIX[] = "*CH=";
static constexpr size_t MLR_SET_CHANNEL_RESPONSE_LEN = 6;
static constexpr uint8_t MLR_SET_CHANNEL_MIN_VALUE_JP = 0x07;
static constexpr uint8_t MLR_SET_CHANNEL_MAX_VALUE_JP = 0x2E;

// @MO (Modem Mode)
static constexpr char MLR_CMD_MODE[] = "@MO";
static constexpr char MLR_SET_MODE_RESPONSE_PREFIX[] = "*MO=";
static constexpr size_t MLR_SET_MODE_RESPONSE_LEN = 6;

// @SF (Spreading Factor)
static constexpr char MLR_CMD_SF[] = "@SF";
static constexpr char MLR_SET_SF_RESPONSE_PREFIX[] = "*SF=";
static constexpr size_t MLR_SET_SF_RESPONSE_LEN = 6;
static constexpr uint8_t MLR_SET_SF_MIN_VALUE = 0x00;
static constexpr uint8_t MLR_SET_SF_MAX_VALUE = 0x05;

// @EI (Equipment ID)
static constexpr char MLR_CMD_EQUIPMENT_ID[] = "@EI";
static constexpr char MLR_SET_EQUIPMENT_RESPONSE_PREFIX[] = "*EI=";
static constexpr size_t MLR_SET_EQUIPMENT_RESPONSE_LEN = 6;

// @DI (Destination ID)
static constexpr char MLR_CMD_DESTINATION_ID[] = "@DI";
static constexpr char MLR_SET_DESTINATION_RESPONSE_PREFIX[] = "*DI=";
static constexpr size_t MLR_SET_DESTINATION_RESPONSE_LEN = 6;

// @GI (Group ID)
static constexpr char MLR_CMD_GROUP_ID[] = "@GI";
static constexpr char MLR_SET_GROUP_RESPONSE_PREFIX[] = "*GI=";
static constexpr size_t MLR_SET_GROUP_RESPONSE_LEN = 6;

// @UI (User ID)
static constexpr char MLR_GET_USERID_STRING[] = "@UI";
static constexpr char MLR_GET_USERID_RESPONSE_PREFIX[] = "*UI=";
static constexpr size_t MLR_GET_USERID_RESPONSE_LEN = 8;

// @RS (RSSI of Last Received Packet)
static constexpr char MLR_GET_RSSI_LAST_RX_STRING[] = "@RS";
static constexpr char MLR_GET_RSSI_LAST_RX_RESPONSE_PREFIX[] = "*RS=";

// @RA (RSSI of Current Channel)
static constexpr char MLR_GET_RSSI_CURRENT_CHANNEL_STRING[] = "@RA";
static constexpr char MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_PREFIX[] = "*RA=";

// @CI (Carrier Sense RSSI Output)
static constexpr char MLR_CMD_CI[] = "@CI";
static constexpr char MLR_SET_CI_RESPONSE_PREFIX[] = "*CI=";
static constexpr size_t MLR_SET_CI_RESPONSE_LEN = 6;

// @SN (Serial Number)
static constexpr char MLR_GET_SERIAL_NUMBER_STRING[] = "@SN";
static constexpr char MLR_GET_SERIAL_NUMBER_RESPONSE_PREFIX[] = "*SN=";
static constexpr size_t MLR_GET_SERIAL_NUMBER_RESPONSE_LEN = 12;

// @IZ (Factory Reset)
static constexpr char MLR_CMD_IZ[] = "@IZ";
static constexpr char MLR_SET_IZ_RESPONSE_PREFIX_OK[] = "*IZ=OK";
static constexpr size_t MLR_SET_IZ_RESPONSE_LEN_OK = 6;

// @BR (Baud Rate)
static constexpr char MLR_CMD_BAUDRATE[] = "@BR";
static constexpr char MLR_SET_BAUDRATE_RESPONSE_PREFIX[] = "*BR=";
static constexpr size_t MLR_SET_BAUDRATE_RESPONSE_LEN = 6;

// @DT (Data Transmission)
static constexpr char MLR_TRANSMISSION_PREFIX_STRING[] = "@DT";
static constexpr char MLR_TRANSMISSION_RESPONSE_PREFIX[] = "*DT=";
static constexpr size_t MLR_TRANSMISSION_RESPONSE_LEN = 6;

// *IR (Information Response)
static constexpr char MLR_INFORMATION_RESPONSE_PREFIX[] = "*IR=";
static constexpr size_t MLR_INFORMATION_RESPONSE_LEN = 6;
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_NO_TX = 1;
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_OTHER_WAVES = 2;
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_OK = 3;

template <uint16_t N>
uint16_t static_strlen(const char (&cstr)[N])
{
    for (uint16_t i = 0; i < N; i++)
    {
        if (cstr[i] == 0)
            return i;
    }
    return 0xFFFF;
}

MLR_Modem_Error MLR_Modem::begin(Stream &pUart, MLR_Modem_AsyncCallback pCallback)
{
    initSerial(pUart); // Base class init

    m_asyncExpectedResponse = MLR_Modem_Response::Idle;
    m_pCallback = pCallback;
    m_parserState = MLR_ModemParserState::Start;
    m_drMessagePresent = false;
    m_drMessageLen = 0;

    // Base class buffer and index are already reset by initSerial

    SM_DEBUG_PRINTLN(" Modem] begin: Getting current mode...");

    MLR_Modem_Error err = GetMode(&m_mode);
    if (err != MLR_Modem_Error::Ok)
    {
        SM_DEBUG_PRINTF(" Modem] begin: GetMode failed! err=%d\n", (int)err);
        return err;
    }

    SM_DEBUG_PRINTF(" Modem] begin: Initialization successful. Mode=%d\n", (int)m_mode);
    return MLR_Modem_Error::Ok;
}

MLR_Modem_Error MLR_Modem::SetChannel(uint8_t channel, bool saveValue)
{
    if ((channel < MLR_SET_CHANNEL_MIN_VALUE_JP) || (channel > MLR_SET_CHANNEL_MAX_VALUE_JP))
    {
        return MLR_Modem_Error::InvalidArg;
    }
    return setByteValue(MLR_CMD_CHANNEL, channel, saveValue, MLR_SET_CHANNEL_RESPONSE_PREFIX, MLR_SET_CHANNEL_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetChannel(uint8_t *pChannel)
{
    return getByteValue(MLR_CMD_CHANNEL, pChannel, MLR_SET_CHANNEL_RESPONSE_PREFIX, MLR_SET_CHANNEL_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetMode(MLR_ModemMode mode, bool saveValue)
{
    if (mode == MLR_ModemMode::FskBin || mode == MLR_ModemMode::LoRaBin)
    {
        return MLR_Modem_Error::InvalidArg;
    }

    MLR_Modem_Error rv = setByteValue(MLR_CMD_MODE, static_cast<uint8_t>(mode), saveValue, MLR_SET_MODE_RESPONSE_PREFIX, MLR_SET_MODE_RESPONSE_LEN);

    if (rv == MLR_Modem_Error::Ok)
    {
        m_mode = mode;
        m_ClearOneLine();
    }
    return rv;
}

MLR_Modem_Error MLR_Modem::GetMode(MLR_ModemMode *pMode)
{
    return getByteValue(MLR_CMD_MODE, reinterpret_cast<uint8_t *>(pMode), MLR_SET_MODE_RESPONSE_PREFIX, MLR_SET_MODE_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetSpreadFactor(MLR_ModemSpreadFactor sf, bool saveValue)
{
    uint8_t sfValue = static_cast<uint8_t>(sf);
    if ((sfValue < MLR_SET_SF_MIN_VALUE) || (sfValue > MLR_SET_SF_MAX_VALUE))
    {
        return MLR_Modem_Error::InvalidArg;
    }
    return setByteValue(MLR_CMD_SF, sfValue, saveValue, MLR_SET_SF_RESPONSE_PREFIX, MLR_SET_SF_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetSpreadFactor(MLR_ModemSpreadFactor *pSf)
{
    return getByteValue(MLR_CMD_SF, reinterpret_cast<uint8_t *>(pSf), MLR_SET_SF_RESPONSE_PREFIX, MLR_SET_SF_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetEquipmentID(uint8_t ei, bool saveValue)
{
    return setByteValue(MLR_CMD_EQUIPMENT_ID, ei, saveValue, MLR_SET_EQUIPMENT_RESPONSE_PREFIX, MLR_SET_EQUIPMENT_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetEquipmentID(uint8_t *pEI)
{
    return getByteValue(MLR_CMD_EQUIPMENT_ID, pEI, MLR_SET_EQUIPMENT_RESPONSE_PREFIX, MLR_SET_EQUIPMENT_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetDestinationID(uint8_t di, bool saveValue)
{
    return setByteValue(MLR_CMD_DESTINATION_ID, di, saveValue, MLR_SET_DESTINATION_RESPONSE_PREFIX, MLR_SET_DESTINATION_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetDestinationID(uint8_t *pDI)
{
    return getByteValue(MLR_CMD_DESTINATION_ID, pDI, MLR_SET_DESTINATION_RESPONSE_PREFIX, MLR_SET_DESTINATION_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetGroupID(uint8_t gi, bool saveValue)
{
    return setByteValue(MLR_CMD_GROUP_ID, gi, saveValue, MLR_SET_GROUP_RESPONSE_PREFIX, MLR_SET_GROUP_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetGroupID(uint8_t *pGI)
{
    return getByteValue(MLR_CMD_GROUP_ID, pGI, MLR_SET_GROUP_RESPONSE_PREFIX, MLR_SET_GROUP_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetUserID(uint16_t *pUserID)
{
    char cmdBuf[8];
    snprintf(cmdBuf, sizeof(cmdBuf), "%s\r\n", MLR_GET_USERID_STRING);
    writeString(cmdBuf);

    MLR_Modem_Error rv = waitForResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessageHexWord(pUserID, MLR_GET_USERID_RESPONSE_LEN, MLR_GET_USERID_RESPONSE_PREFIX);
    }
    return rv;
}

MLR_Modem_Error MLR_Modem::GetRssiLastRx(int16_t *pRssi)
{
    char cmdBuf[8];
    snprintf(cmdBuf, sizeof(cmdBuf), "%s\r\n", MLR_GET_RSSI_LAST_RX_STRING);
    writeString(cmdBuf);

    MLR_Modem_Error rv = waitForResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_RS(pRssi);
    }
    return rv;
}

MLR_Modem_Error MLR_Modem::GetRssiCurrentChannel(int16_t *pRssi)
{
    char cmdBuf[8];
    snprintf(cmdBuf, sizeof(cmdBuf), "%s\r\n", MLR_GET_RSSI_CURRENT_CHANNEL_STRING);
    writeString(cmdBuf);

    MLR_Modem_Error rv = waitForResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_RA(pRssi);
    }
    return rv;
}

MLR_Modem_Error MLR_Modem::SetCarrierSenseRssiOutput(uint8_t ciValue, bool saveValue)
{
    return setByteValue(MLR_CMD_CI, ciValue, saveValue, MLR_SET_CI_RESPONSE_PREFIX, MLR_SET_CI_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetCarrierSenseRssiOutput(uint8_t *pCiValue)
{
    return getByteValue(MLR_CMD_CI, pCiValue, MLR_SET_CI_RESPONSE_PREFIX, MLR_SET_CI_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetSerialNumber(uint32_t *pSerialNumber)
{
    char cmdBuf[8];
    snprintf(cmdBuf, sizeof(cmdBuf), "%s\r\n", MLR_GET_SERIAL_NUMBER_STRING);
    writeString(cmdBuf);

    MLR_Modem_Error rv = waitForResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_SN(pSerialNumber);
    }
    return rv;
}

MLR_Modem_Error MLR_Modem::FactoryReset()
{
    // First, send command
    char cmdBuf[8];
    snprintf(cmdBuf, sizeof(cmdBuf), "%s\r\n", MLR_CMD_IZ);
    writeString(cmdBuf);

    // Factory Reset flow is:
    // 1. *WR=PS (Save confirmation)
    MLR_Modem_Error rv = waitForResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = waitForResponse();
    }

    // 2. *IZ=OK
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_IZ();
    }

    // 3. "LORA MODE" or similar
    if (rv == MLR_Modem_Error::Ok)
    {
        m_ClearOneLine();
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::GetBaudRate(uint8_t *pBaudRate)
{
    return getByteValue(MLR_CMD_BAUDRATE, pBaudRate, MLR_SET_BAUDRATE_RESPONSE_PREFIX, MLR_SET_BAUDRATE_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetBaudRate(uint32_t baudRate, bool saveValue)
{
    uint8_t baudCode;
    switch (baudRate)
    {
    case 1200:
        baudCode = 0x12;
        break;
    case 2400:
        baudCode = 0x24;
        break;
    case 4800:
        baudCode = 0x48;
        break;
    case 9600:
        baudCode = 0x96;
        break;
    case 19200:
        baudCode = 0x19;
        break;
    default:
        return MLR_Modem_Error::InvalidArg;
    }

    return setByteValue(MLR_CMD_BAUDRATE, baudCode, saveValue, MLR_SET_BAUDRATE_RESPONSE_PREFIX, MLR_SET_BAUDRATE_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SendRawCommand(const char *command, char *responseBuffer, size_t bufferSize, uint32_t timeoutMs)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }
    return sendRawCommand(command, responseBuffer, bufferSize, timeoutMs);
}

MLR_Modem_Error MLR_Modem::SendRawCommandAsync(const char *command, uint32_t timeoutMs)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    writeString(command); // Base class method
    m_asyncExpectedResponse = MLR_Modem_Response::GenericResponse;
    startTimeout(timeoutMs); // Base class method

    return MLR_Modem_Error::Ok;
}

MLR_Modem_Error MLR_Modem::TransmitData(const uint8_t *pMsg, uint8_t len)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    std::array<char, 6> cmdHeader;
    snprintf(cmdHeader.data(), cmdHeader.size(), "%s%02X", MLR_TRANSMISSION_PREFIX_STRING, static_cast<unsigned>(len));
    writeString(cmdHeader.data(), true);
    writeData(pMsg, len);
    writeString("\r\n", false);

    MLR_Modem_Error rv = waitForResponse();

    // check transmission response
    uint8_t transmissionResponse{};
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessageHexByte(&transmissionResponse, MLR_TRANSMISSION_RESPONSE_LEN, MLR_TRANSMISSION_RESPONSE_PREFIX);
    }

    if (rv == MLR_Modem_Error::Ok && transmissionResponse != len)
    {
        rv = MLR_Modem_Error::Fail;
    }

    // check information response (*IR=...)
    if (rv == MLR_Modem_Error::Ok)
    {
        uint32_t waitTime = (m_mode == MLR_ModemMode::LoRaCmd) ? 15000 : 11;
        rv = waitForResponse(waitTime);
    }

    uint8_t informationResponse{};
    if (rv == MLR_Modem_Error::Ok)
    {
        // FSK mode handles timeout as OK in original code logic?
        // Original: "FSK mode: if send OK, no *IR response. Carrier sense error results in *IR=01"
        // If waitForResponse times out, it means no *IR error -> OK.
        // Wait, waitForResponse returns Timeout error.

        if (m_mode != MLR_ModemMode::LoRaCmd)
        {
            // FSK logic
            // If we received something (Ok), check if it is error.
            // If we timed out (Timeout), it is success.
            // However, waitForResponse() returns Timeout on timeout.
        }
    }

    // Re-implementing logic specifically because of the subtle FSK behavior:
    // If LoRa: *IR is mandatory.
    // If FSK: *IR only on error.

    if (m_mode == MLR_ModemMode::LoRaCmd)
    {
        if (rv == MLR_Modem_Error::Ok)
        {
            rv = m_HandleMessageHexByte(&informationResponse, MLR_INFORMATION_RESPONSE_LEN, MLR_INFORMATION_RESPONSE_PREFIX);
        }

        if (rv == MLR_Modem_Error::Ok)
        {
            switch (informationResponse)
            {
            case MLR_INFORMATION_RESPONSE_ERR_OTHER_WAVES:
            case MLR_INFORMATION_RESPONSE_ERR_NO_TX:
                rv = MLR_Modem_Error::FailLbt;
                break;
            default:
                break;
            }
        }
    }
    else // FSK
    {
        if (rv == MLR_Modem_Error::Timeout)
        {
            // FSK: Timeout means no *IR error, so Success.
            rv = MLR_Modem_Error::Ok;
        }
        else if (rv == MLR_Modem_Error::Ok)
        {
            // Received something, likely *IR=01
            rv = m_HandleMessageHexByte(&informationResponse, MLR_INFORMATION_RESPONSE_LEN, MLR_INFORMATION_RESPONSE_PREFIX);
            if (rv == MLR_Modem_Error::Ok)
            {
                if (informationResponse == MLR_INFORMATION_RESPONSE_ERR_NO_TX)
                    rv = MLR_Modem_Error::FailLbt;
                else
                    rv = MLR_Modem_Error::Fail;
            }
        }
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::TransmitDataFireAndForget(const uint8_t *pMsg, uint8_t len)
{
    if (!pMsg || len == 0)
        return MLR_Modem_Error::InvalidArg;
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
        return MLR_Modem_Error::Busy;

    std::array<char, 6> cmdHeader;
    snprintf(cmdHeader.data(), cmdHeader.size(), "%s%02X", MLR_TRANSMISSION_PREFIX_STRING, static_cast<unsigned>(len));
    writeString(cmdHeader.data(), true);
    writeData(pMsg, len);
    writeString("\r\n", false);

    MLR_Modem_Error rv = waitForResponse();
    uint8_t transmissionResponse{};
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessageHexByte(&transmissionResponse, MLR_TRANSMISSION_RESPONSE_LEN, MLR_TRANSMISSION_RESPONSE_PREFIX);
    }

    if (rv == MLR_Modem_Error::Ok && transmissionResponse != len)
    {
        rv = MLR_Modem_Error::Fail;
    }

    if (rv == MLR_Modem_Error::Ok)
    {
        m_asyncExpectedResponse = MLR_Modem_Response::MLR_Modem_DtIr;
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::GetRssiCurrentChannelAsync()
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
        return MLR_Modem_Error::Busy;

    char cmdBuf[8];
    snprintf(cmdBuf, sizeof(cmdBuf), "%s\r\n", MLR_GET_RSSI_CURRENT_CHANNEL_STRING);
    writeString(cmdBuf);
    m_asyncExpectedResponse = MLR_Modem_Response::RssiCurrentChannel;
    return MLR_Modem_Error::Ok;
}

MLR_Modem_Error MLR_Modem::GetSerialNumberAsync()
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
        return MLR_Modem_Error::Busy;

    char cmdBuf[8];
    snprintf(cmdBuf, sizeof(cmdBuf), "%s\r\n", MLR_GET_SERIAL_NUMBER_STRING);
    writeString(cmdBuf);
    m_asyncExpectedResponse = MLR_Modem_Response::SerialNumber;

    return MLR_Modem_Error::Ok;
}

MLR_Modem_Error MLR_Modem::GetPacket(const uint8_t **ppData, uint8_t *len)
{
    if (m_drMessagePresent)
    {
        *ppData = &m_drMessage[0];
        *len = m_drMessageLen;
        return MLR_Modem_Error::Ok;
    }
    else
        return MLR_Modem_Error::Fail;
}

void MLR_Modem::Work()
{
    // Check Async Timeout
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle && isTimeout())
    {
        SM_DEBUG_PRINTLN(" Work] Timeout during async operation.");
        // Notify callback if needed or just reset
        m_asyncExpectedResponse = MLR_Modem_Response::Idle;
        m_parserState = MLR_ModemParserState::Start;
    }

    ModemParseResult result = parse();
    switch (result)
    {
    case ModemParseResult::Parsing:
        break;
    case ModemParseResult::Garbage:
        SM_DEBUG_PRINTLN(" Work] Work: Parser encountered garbage.");
        if (m_pCallback)
        { /* Notify Error if needed */
        }
        break;
    case ModemParseResult::Overflow:
        SM_DEBUG_PRINTLN(" Work] Work: Parser encountered overflow.");
        if (m_pCallback)
        { /* Notify Error if needed */
        }
        break;
    case ModemParseResult::FinishedCmdResponse:
        SM_DEBUG_PRINTF(" Work] Work: Finished CMD response, dispatching async.\n");
        m_DispatchCmdResponseAsync();
        break;
    case ModemParseResult::FinishedDrResponse:
        SM_DEBUG_PRINTF(" Work] Work: Finished DR response (Len=%u). Calling callback.\n", m_drMessageLen);
        onRxDataReceived();
        break;
    }
}

// --- SerialModemBase Virtual Implementation ---

void MLR_Modem::onRxDataReceived()
{
    if (m_pCallback)
    {
        m_pCallback(MLR_Modem_Error::Ok, MLR_Modem_Response::DataReceived, 0, &m_drMessage[0], m_drMessageLen);
    }
}

ModemParseResult MLR_Modem::parse()
{
    // Note: readByte(), unreadByte(), flushGarbage() are from SerialModemBase
    // _rxBuffer, _rxIndex are from SerialModemBase

    while (_uart->available() || _oneByteBuf != -1)
    {
        switch (m_parserState)
        {
        case MLR_ModemParserState::Start:
            _rxIndex = 0;
            _rxBuffer[_rxIndex] = readByte();

            if (_rxBuffer[_rxIndex] == '*')
            {
                ++_rxIndex;
                m_parserState = MLR_ModemParserState::ReadCmdFirstLetter;
            }
            else
            {
                flushGarbage();
                return ModemParseResult::Parsing; // or Garbage
            }
            break;

        case MLR_ModemParserState::ReadCmdFirstLetter:
            _rxBuffer[_rxIndex] = readByte();
            if (isupper(_rxBuffer[_rxIndex]))
            {
                ++_rxIndex;
                m_parserState = MLR_ModemParserState::ReadCmdSecondLetter;
            }
            else
            {
                if (_rxBuffer[_rxIndex] == '*')
                    unreadByte('*');
                flushGarbage();
                return ModemParseResult::Garbage;
            }
            break;

        case MLR_ModemParserState::ReadCmdSecondLetter:
            _rxBuffer[_rxIndex] = readByte();
            if (isupper(_rxBuffer[_rxIndex]))
            {
                ++_rxIndex;
                m_parserState = MLR_ModemParserState::ReadCmdParam;
            }
            else
            {
                if (_rxBuffer[_rxIndex] == '*')
                    unreadByte('*');
                flushGarbage();
                return ModemParseResult::Garbage;
            }
            break;

        case MLR_ModemParserState::ReadCmdParam:
            _rxBuffer[_rxIndex] = readByte();

            if ((_rxBuffer[1] == 'D') && (_rxBuffer[2] == 'R') && (_rxBuffer[3] == '='))
            {
                // @DR telegram
                ++_rxIndex;
                m_parserState = MLR_ModemParserState::RadioDrSize;
            }
            else if (isupper(_rxBuffer[1]) && isupper(_rxBuffer[2]) && (_rxBuffer[3] == '='))
            {
                ++_rxIndex;
                m_parserState = MLR_ModemParserState::ReadCmdUntilCR;
            }
            else
            {
                if (_rxBuffer[_rxIndex] == '*')
                    unreadByte('*');
                flushGarbage();
                return ModemParseResult::Garbage;
            }
            break;

        case MLR_ModemParserState::RadioDrSize:
            _rxBuffer[_rxIndex] = readByte();
            ++_rxIndex;
            if (_rxIndex < 6)
                return ModemParseResult::Parsing;

            if (isxdigit(_rxBuffer[4]) && isxdigit(_rxBuffer[5]))
            {
                m_drMessagePresent = false;
                uint32_t msgLen = 0;
                // Using Base static helper
                parseHex(&_rxBuffer[4], 2, &msgLen);
                m_drMessageLen = msgLen;
                _rxIndex = 0;
                m_parserState = MLR_ModemParserState::RadioDrPayload;
            }
            else
            {
                flushGarbage();
                return ModemParseResult::Garbage;
            }
            break;

        case MLR_ModemParserState::RadioDrPayload:
        {
            m_drMessage[_rxIndex] = readByte();
            ++_rxIndex;

            if ((m_drMessageLen + 2 - _rxIndex) == 0)
            {
                if ((m_drMessage[_rxIndex - 2] == '\r') && m_drMessage[_rxIndex - 1] == '\n')
                {
                    m_drMessage[_rxIndex - 2] = 0; // null terminate
                    _rxIndex = 0;
                    _rxBuffer[0] = 0;
                    m_drMessagePresent = true;
                    m_parserState = MLR_ModemParserState::Start;
                    return ModemParseResult::FinishedDrResponse;
                }
                else
                {
                    flushGarbage();
                    return ModemParseResult::Garbage;
                }
            }
            break;
        }

        case MLR_ModemParserState::ReadCmdUntilCR:
            _rxBuffer[_rxIndex] = readByte();

            if (_rxBuffer[_rxIndex] == '\r')
            {
                ++_rxIndex;
                if (_rxIndex >= RX_BUFFER_SIZE)
                {
                    m_parserState = MLR_ModemParserState::Start;
                    return ModemParseResult::Overflow;
                }
                m_parserState = MLR_ModemParserState::ReadCmdUntilLF;
            }
            else if (_rxBuffer[_rxIndex] == '\n' || _rxBuffer[_rxIndex] == '*')
            {
                if (_rxBuffer[_rxIndex] == '*')
                    unreadByte('*');
                flushGarbage();
                return ModemParseResult::Garbage;
            }
            else
            {
                ++_rxIndex;
                if (_rxIndex >= RX_BUFFER_SIZE)
                {
                    m_parserState = MLR_ModemParserState::Start;
                    return ModemParseResult::Overflow;
                }
            }
            break;

        case MLR_ModemParserState::ReadCmdUntilLF:
            _rxBuffer[_rxIndex] = readByte();
            if (_rxBuffer[_rxIndex] == '\n')
            {
                // Remove CR/LF from length
                --_rxIndex;
                _rxBuffer[_rxIndex] = 0; // null terminate at CR
                m_parserState = MLR_ModemParserState::Start;
                return ModemParseResult::FinishedCmdResponse;
            }
            else
            {
                if (_rxBuffer[_rxIndex] == '*')
                    unreadByte('*');
                flushGarbage();
                return ModemParseResult::Garbage;
            }
            break;

        default:
            m_parserState = MLR_ModemParserState::Start;
            break;
        }
    }
    return ModemParseResult::Parsing;
}

// --- Internal Logic ---

MLR_Modem_Error MLR_Modem::m_DispatchCmdResponseAsync()
{
    // Implementation largely similar to original, but using m_pCallback directly
    MLR_Modem_Error err = MLR_Modem_Error::Fail;

    switch (m_asyncExpectedResponse)
    {
    case MLR_Modem_Response::Idle:
        break;
    case MLR_Modem_Response::SerialNumber:
        if (m_pCallback)
        {
            uint32_t sn{};
            err = m_HandleMessage_SN(&sn);
            m_pCallback(err, MLR_Modem_Response::SerialNumber, (int32_t)sn, nullptr, 0);
        }
        break;
    case MLR_Modem_Response::MLR_Modem_DtIr:
        if (m_pCallback)
        {
            uint8_t irValue{};
            err = m_HandleMessageHexByte(&irValue, MLR_INFORMATION_RESPONSE_LEN, MLR_INFORMATION_RESPONSE_PREFIX);
            m_pCallback(err, MLR_Modem_Response::MLR_Modem_DtIr, (int32_t)irValue, nullptr, 0);
        }
        break;
    case MLR_Modem_Response::RssiCurrentChannel:
        if (m_pCallback)
        {
            int16_t rssi{};
            err = m_HandleMessage_RA(&rssi);
            m_pCallback(err, MLR_Modem_Response::RssiCurrentChannel, (int32_t)rssi, nullptr, 0);
        }
        break;
    case MLR_Modem_Response::GenericResponse:
        if (m_pCallback)
        {
            m_pCallback(MLR_Modem_Error::Ok, MLR_Modem_Response::GenericResponse, 0, _rxBuffer, _rxIndex);
        }
        break;
    default:
        break;
    }
    m_asyncExpectedResponse = MLR_Modem_Response::Idle;
    return err;
}

MLR_Modem_Error MLR_Modem::m_HandleMessageHexByte(uint8_t *pValue, uint32_t responseLen, const char *responsePrefix)
{
    uint32_t val;
    // Using Base helper
    ModemError err = parseResponseHex(_rxBuffer, _rxIndex, responsePrefix, (uint8_t)(responseLen - strlen(responsePrefix)), &val);
    if (err == ModemError::Ok)
        *pValue = (uint8_t)val;
    return err;
}

MLR_Modem_Error MLR_Modem::m_HandleMessageHexWord(uint16_t *pValue, uint32_t responseLen, const char *responsePrefix)
{
    uint32_t val;
    // Using Base helper (adapting hexDigits calculation)
    ModemError err = parseResponseHex(_rxBuffer, _rxIndex, responsePrefix, (uint8_t)(responseLen - strlen(responsePrefix)), &val);
    if (err == ModemError::Ok)
        *pValue = (uint16_t)val;
    return err;
}

MLR_Modem_Error MLR_Modem::m_HandleMessage_RS(int16_t *pRssi)
{
    int32_t val;
    ModemError err = parseResponseDec(_rxBuffer, _rxIndex, MLR_GET_RSSI_LAST_RX_RESPONSE_PREFIX, "dBm", 3, &val);
    if (err == ModemError::Ok)
        *pRssi = (int16_t)val;
    return err;
}

MLR_Modem_Error MLR_Modem::m_HandleMessage_RA(int16_t *pRssi)
{
    int32_t val;
    ModemError err = parseResponseDec(_rxBuffer, _rxIndex, MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_PREFIX, "dBm", 3, &val);
    if (err == ModemError::Ok)
        *pRssi = (int16_t)val;
    return err;
}

MLR_Modem_Error MLR_Modem::m_HandleMessage_SN(uint32_t *pSerialNumber)
{
    if (_rxIndex < MLR_GET_SERIAL_NUMBER_RESPONSE_LEN)
        return MLR_Modem_Error::Fail;
    if (strncmp(MLR_GET_SERIAL_NUMBER_RESPONSE_PREFIX, (char *)_rxBuffer, strlen(MLR_GET_SERIAL_NUMBER_RESPONSE_PREFIX)) != 0)
        return MLR_Modem_Error::Fail;

    uint8_t startIdx = 4;
    uint8_t hexLen = 8;
    if (!isdigit(_rxBuffer[4])) // Handle "*SN=S..."
    {
        startIdx = 5;
        hexLen = 7;
    }

    uint32_t sn;
    if (parseDec(&_rxBuffer[startIdx], hexLen, &sn))
    {
        if (pSerialNumber)
            *pSerialNumber = sn;
        return MLR_Modem_Error::Ok;
    }
    return MLR_Modem_Error::Fail;
}

MLR_Modem_Error MLR_Modem::m_HandleMessage_IZ()
{
    if (_rxIndex == MLR_SET_IZ_RESPONSE_LEN_OK &&
        strncmp(MLR_SET_IZ_RESPONSE_PREFIX_OK, (char *)_rxBuffer, MLR_SET_IZ_RESPONSE_LEN_OK) == 0)
    {
        return MLR_Modem_Error::Ok;
    }
    return MLR_Modem_Error::Fail;
}

void MLR_Modem::m_ClearOneLine()
{
    if (_uart)
    {
        _uart->setTimeout(500);
        _uart->readStringUntil('\n');
    }
}