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
// @W (Write to NVM)
static constexpr char MLR_WRITE_VALUE_RESPONSE_PREFIX[] = "*WR=PS";
static constexpr size_t MLR_WRITE_VALUE_RESPONSE_LEN = 6; // length of "*WR=PS" excluding "\r\n"

// @CH (Channel Frequency)
static constexpr char MLR_GET_CHANNEL_STRING[] = "@CH\r\n";
static constexpr char MLR_SET_CHANNEL_PREFIX_STRING[] = "@CH";
static constexpr char MLR_SET_CHANNEL_RESPONSE_PREFIX[] = "*CH=";
static constexpr size_t MLR_SET_CHANNEL_RESPONSE_LEN = 6;     // length of "*CH=0E" excluding "\r\n"
static constexpr uint8_t MLR_SET_CHANNEL_MIN_VALUE_JP = 0x07; // channel 7
static constexpr uint8_t MLR_SET_CHANNEL_MAX_VALUE_JP = 0x2E; // channel 46

// @MO (Modem Mode)
static constexpr char MLR_GET_MODE_STRING[] = "@MO\r\n";
static constexpr char MLR_SET_MODE_PREFIX_STRING[] = "@MO";
static constexpr char MLR_SET_MODE_RESPONSE_PREFIX[] = "*MO=";
static constexpr size_t MLR_SET_MODE_RESPONSE_LEN = 6; // length of "*MO=01" excluding "\r\n"

// @SF (Spreading Factor)
static constexpr char MLR_GET_SF_STRING[] = "@SF\r\n";
static constexpr char MLR_SET_SF_PREFIX_STRING[] = "@SF";
static constexpr char MLR_SET_SF_RESPONSE_PREFIX[] = "*SF=";
static constexpr size_t MLR_SET_SF_RESPONSE_LEN = 6; // length of "*SF=00" excluding "\r\n"
static constexpr uint8_t MLR_SET_SF_MIN_VALUE = 0x00;
static constexpr uint8_t MLR_SET_SF_MAX_VALUE = 0x05;

// @EI (Equipment ID)
static constexpr char MLR_GET_EQUIPMENT_STRING[] = "@EI\r\n";
static constexpr char MLR_SET_EQUIPMENT_PREFIX_STRING[] = "@EI";
static constexpr char MLR_SET_EQUIPMENT_RESPONSE_PREFIX[] = "*EI=";
static constexpr size_t MLR_SET_EQUIPMENT_RESPONSE_LEN = 6; // length of "*EI=0E" excluding "\r\n"

// @DI (Destination ID)
static constexpr char MLR_GET_DESTINATION_STRING[] = "@DI\r\n";
static constexpr char MLR_SET_DESTINATION_PREFIX_STRING[] = "@DI";
static constexpr char MLR_SET_DESTINATION_RESPONSE_PREFIX[] = "*DI=";
static constexpr size_t MLR_SET_DESTINATION_RESPONSE_LEN = 6; // length of "*DI=0E" excluding "\r\n"

// @GI (Group ID)
static constexpr char MLR_GET_GROUP_STRING[] = "@GI\r\n";
static constexpr char MLR_SET_GROUP_PREFIX_STRING[] = "@GI";
static constexpr char MLR_SET_GROUP_RESPONSE_PREFIX[] = "*GI=";
static constexpr size_t MLR_SET_GROUP_RESPONSE_LEN = 6; // length of "*GI=0E" excluding "\r\n"

// @UI (User ID)
static constexpr char MLR_GET_USERID_STRING[] = "@UI\r\n";
static constexpr char MLR_GET_USERID_RESPONSE_PREFIX[] = "*UI=";
static constexpr size_t MLR_GET_USERID_RESPONSE_LEN = 8; // length of "*UI=0000" excluding "\r\n"

// @RS (RSSI of Last Received Packet)
static constexpr char MLR_GET_RSSI_LAST_RX_STRING[] = "@RS\r\n";
static constexpr char MLR_GET_RSSI_LAST_RX_RESPONSE_PREFIX[] = "*RS=";
static constexpr size_t MLR_GET_RSSI_LAST_RX_RESPONSE_MIN_LEN = 10; // length of "*RS=-12dBm" excluding "\r\n"
static constexpr size_t MLR_GET_RSSI_LAST_RX_RESPONSE_MAX_LEN = 11; // length of "*RS=-123dBm" excluding "\r\n"

// @RA (RSSI of Current Channel)
static constexpr char MLR_GET_RSSI_CURRENT_CHANNEL_STRING[] = "@RA\r\n";
static constexpr char MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_PREFIX[] = "*RA=";
static constexpr size_t MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_MIN_LEN = 10; // length of "*RA=-12dBm" excluding "\r\n"
static constexpr size_t MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_MAX_LEN = 11; // length of "*RA=-123dBm" excluding "\r\n"

// @CI (Carrier Sense RSSI Output)
static constexpr char MLR_GET_CI_STRING[] = "@CI\r\n";
static constexpr char MLR_SET_CI_PREFIX_STRING[] = "@CI";
static constexpr char MLR_SET_CI_RESPONSE_PREFIX[] = "*CI=";
static constexpr size_t MLR_SET_CI_RESPONSE_LEN = 6; // length of "*CI=01" excluding "\r\n"

// @SN (Serial Number)
static constexpr char MLR_GET_SERIAL_NUMBER_STRING[] = "@SN\r\n";
static constexpr char MLR_GET_SERIAL_NUMBER_RESPONSE_PREFIX[] = "*SN=";
static constexpr size_t MLR_GET_SERIAL_NUMBER_RESPONSE_LEN = 12; // length of "*SN=A1234567" excluding "\r\n"

// @IZ (Factory Reset)
static constexpr char MLR_SET_IZ_STRING[] = "@IZ\r\n";
static constexpr char MLR_SET_IZ_RESPONSE_PREFIX_OK[] = "*IZ=OK";
static constexpr size_t MLR_SET_IZ_RESPONSE_LEN_OK = 6; // length of "*IZ=OK" excluding "\r\n"

// @BR (Baud Rate)
static constexpr char MLR_GET_BAUDRATE_STRING[] = "@BR\r\n";
static constexpr char MLR_SET_BAUDRATE_PREFIX_STRING[] = "@BR";
static constexpr char MLR_SET_BAUDRATE_RESPONSE_PREFIX[] = "*BR=";
static constexpr size_t MLR_SET_BAUDRATE_RESPONSE_LEN = 6; // length of "*BR=19" excluding "\r\n"

// @DT (Data Transmission)
static constexpr char MLR_TRANSMISSION_PREFIX_STRING[] = "@DT";
static constexpr char MLR_TRANSMISSION_RESPONSE_PREFIX[] = "*DT=";
static constexpr size_t MLR_TRANSMISSION_RESPONSE_LEN = 6; // length of "*DT=06" excluding "\r\n"

// // @PS (Contact Function for SLR429)
// static constexpr char MLR_GET_CONTACT_FUNCTION_STRING[] = "@PS\r\n";
// static constexpr char MLR_SET_CONTACT_FUNCTION_PREFIX_STRING[] = "@PS";
// static constexpr char MLR_SET_CONTACT_FUNCTION_RESPONSE_PREFIX[] = "*PS=";
// static constexpr size_t MLR_SET_CONTACT_FUNCTION_RESPONSE_LEN = 6; // length of "*PS=0F" excluding "\r\n"

// *IR (Information Response)
static constexpr char MLR_INFORMATION_RESPONSE_PREFIX[] = "*IR=";
static constexpr size_t MLR_INFORMATION_RESPONSE_LEN = 6;              // length of "*IR=03" excluding "\r\n"
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_NO_TX = 1;       // data transmission is not possible (for unknown reasons)
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_OTHER_WAVES = 2; // data transmission is not possible because of presence of other LoRa modules
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_OK = 3;          // data transmission complete

// string length calculated at compile time
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

static bool s_ParseHex(const uint8_t *pData, uint8_t len, uint32_t *pResult)
{
    *pResult = 0;

    while (len--)
    {
        *pResult <<= 4;
        if (isxdigit(*pData))
        {
            if (*pData >= 'a')
            {
                *pResult |= *pData - 'a' + 10;
            }
            else if (*pData >= 'A')
            {
                *pResult |= *pData - 'A' + 10;
            }
            else
            {
                *pResult |= *pData - '0';
            }
        }
        else
        {
            return false;
        }

        ++pData;
    }

    return true;
}

static bool s_ParseDec(const uint8_t *pData, uint8_t len, uint32_t *pResult)
{
    *pResult = 0;

    while (len--)
    {
        *pResult = *pResult * 10;
        if (isdigit(*pData))
        {
            *pResult += *pData - '0';
        }
        else
        {
            return false;
        }

        ++pData;
    }

    return true;
}

MLR_Modem_Error MLR_Modem::begin(Stream &pUart, MLR_Modem_AsyncCallback pCallback)
{
    m_pDebugStream = nullptr;
    m_asyncExpectedResponse = MLR_Modem_Response::Idle;
    m_pCallback = pCallback;
    m_pUart = &pUart;
    m_rxIdx = 0;
    m_parserState = MLR_ModemParserState::Start;
    m_drMessagePresent = false;
    m_drMessageLen = 0;
    m_ResetParser();

    MLR_DEBUG_PRINTLN("[MLR Modem] begin: Getting current mode...");
    MLR_Modem_Error err = GetMode(&m_mode); // Get and cache the current mode
    if (err != MLR_Modem_Error::Ok)
    {
        MLR_DEBUG_PRINTF("[MLR Modem] begin: GetMode failed! err=%d\n", (int)err);
        return err; // Return error if GetMode fails
    }

    MLR_DEBUG_PRINTF("[MLR Modem] begin: Initialization successful. Mode=%d\n", (int)m_mode);
    return MLR_Modem_Error::Ok;
}

MLR_Modem_Error MLR_Modem::SetChannel(uint8_t channel, bool saveValue)
{
    if ((channel < MLR_SET_CHANNEL_MIN_VALUE_JP) || (channel > MLR_SET_CHANNEL_MAX_VALUE_JP))
    {
        return MLR_Modem_Error::InvalidArg;
    }

    return m_SetByteValue(MLR_SET_CHANNEL_PREFIX_STRING, channel, saveValue, MLR_SET_CHANNEL_RESPONSE_PREFIX, MLR_SET_CHANNEL_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetChannel(uint8_t *pChannel)
{
    return m_GetByteValue(MLR_GET_CHANNEL_STRING, pChannel, MLR_SET_CHANNEL_RESPONSE_PREFIX, MLR_SET_CHANNEL_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetMode(MLR_ModemMode mode, bool saveValue)
{
    if (mode == MLR_ModemMode::FskBin || mode == MLR_ModemMode::LoRaBin)
    {
        // binary modes currently not supported
        return MLR_Modem_Error::InvalidArg;
    }

    MLR_Modem_Error rv = m_SetByteValue(MLR_SET_MODE_PREFIX_STRING, static_cast<uint8_t>(mode), saveValue, MLR_SET_MODE_RESPONSE_PREFIX, MLR_SET_MODE_RESPONSE_LEN);

    if (rv == MLR_Modem_Error::Ok)
    {
        m_mode = mode;
        // Handle messages "FSK CMD MODE" etc.
        m_ClearOneLine();
    }
    return rv;
}

MLR_Modem_Error MLR_Modem::GetMode(MLR_ModemMode *pMode)
{
    return m_GetByteValue(MLR_GET_MODE_STRING, reinterpret_cast<uint8_t *>(pMode), MLR_SET_MODE_RESPONSE_PREFIX, MLR_SET_MODE_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetSpreadFactor(MLR_ModemSpreadFactor sf, bool saveValue)
{
    uint8_t sfValue = static_cast<uint8_t>(sf);
    if ((sfValue < MLR_SET_SF_MIN_VALUE) || (sfValue > MLR_SET_SF_MAX_VALUE))
    {
        return MLR_Modem_Error::InvalidArg;
    }
    return m_SetByteValue(MLR_SET_SF_PREFIX_STRING, sfValue, saveValue, MLR_SET_SF_RESPONSE_PREFIX, MLR_SET_SF_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetSpreadFactor(MLR_ModemSpreadFactor *pSf)
{
    return m_GetByteValue(MLR_GET_SF_STRING, reinterpret_cast<uint8_t *>(pSf), MLR_SET_SF_RESPONSE_PREFIX, MLR_SET_SF_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetEquipmentID(uint8_t ei, bool saveValue)
{
    return m_SetByteValue(MLR_SET_EQUIPMENT_PREFIX_STRING, ei, saveValue, MLR_SET_EQUIPMENT_RESPONSE_PREFIX, MLR_SET_EQUIPMENT_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetEquipmentID(uint8_t *pEI)
{
    return m_GetByteValue(MLR_GET_EQUIPMENT_STRING, pEI, MLR_SET_EQUIPMENT_RESPONSE_PREFIX, MLR_SET_EQUIPMENT_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetDestinationID(uint8_t di, bool saveValue)
{
    return m_SetByteValue(MLR_SET_DESTINATION_PREFIX_STRING, di, saveValue, MLR_SET_DESTINATION_RESPONSE_PREFIX, MLR_SET_DESTINATION_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetDestinationID(uint8_t *pDI)
{
    return m_GetByteValue(MLR_GET_DESTINATION_STRING, pDI, MLR_SET_DESTINATION_RESPONSE_PREFIX, MLR_SET_DESTINATION_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetGroupID(uint8_t gi, bool saveValue)
{
    return m_SetByteValue(MLR_SET_GROUP_PREFIX_STRING, gi, saveValue, MLR_SET_GROUP_RESPONSE_PREFIX, MLR_SET_GROUP_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetGroupID(uint8_t *pGI)
{
    return m_GetByteValue(MLR_GET_GROUP_STRING, pGI, MLR_SET_GROUP_RESPONSE_PREFIX, MLR_SET_GROUP_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetUserID(uint16_t *pUserID)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(MLR_GET_USERID_STRING);

    MLR_Modem_Error rv = m_WaitCmdResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessageHexWord(pUserID, MLR_GET_USERID_RESPONSE_LEN, MLR_GET_USERID_RESPONSE_PREFIX);
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::GetRssiLastRx(int16_t *pRssi)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(MLR_GET_RSSI_LAST_RX_STRING);

    MLR_Modem_Error rv = m_WaitCmdResponse();

    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_RS(pRssi);
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::GetRssiCurrentChannel(int16_t *pRssi)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(MLR_GET_RSSI_CURRENT_CHANNEL_STRING);

    MLR_Modem_Error rv = m_WaitCmdResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_RA(pRssi);
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::SetCarrierSenseRssiOutput(uint8_t ciValue, bool saveValue)
{
    return m_SetByteValue(MLR_SET_CI_PREFIX_STRING, ciValue, saveValue, MLR_SET_CI_RESPONSE_PREFIX, MLR_SET_CI_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetCarrierSenseRssiOutput(uint8_t *pCiValue)
{
    return m_GetByteValue(MLR_GET_CI_STRING, pCiValue, MLR_SET_CI_RESPONSE_PREFIX, MLR_SET_CI_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::GetSerialNumber(uint32_t *pSerialNumber)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(MLR_GET_SERIAL_NUMBER_STRING);

    MLR_Modem_Error rv = m_WaitCmdResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_SN(pSerialNumber);
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::FactoryReset()
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(MLR_SET_IZ_STRING);

    // First response is *WR=PS
    MLR_Modem_Error rv = m_WaitCmdResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_WR();
    }

    // Second response is *IZ=OK
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_WaitCmdResponse();
    }
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessage_IZ();
    }

    // Third response is "LORA MODE" or similar
    if (rv == MLR_Modem_Error::Ok)
    {
        m_ClearOneLine();
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::GetBaudRate(uint8_t *pBaudRate)
{
    return m_GetByteValue(MLR_GET_BAUDRATE_STRING, pBaudRate, MLR_SET_BAUDRATE_RESPONSE_PREFIX, MLR_SET_BAUDRATE_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SetBaudRate(uint32_t baudRate, bool saveValue)
{
    // Convert baud rate (BPS) to modem command code
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
        return MLR_Modem_Error::InvalidArg; // Invalid baud rate specified
    }

    return m_SetByteValue(MLR_SET_BAUDRATE_PREFIX_STRING, baudCode, saveValue, MLR_SET_BAUDRATE_RESPONSE_PREFIX, MLR_SET_BAUDRATE_RESPONSE_LEN);
}

// MLR_Modem_Error MLR_Modem::GetContactFunction(uint8_t *pContactFunction)
// {
//     return m_GetByteValue(MLR_GET_CONTACT_FUNCTION_STRING, pContactFunction, MLR_SET_CONTACT_FUNCTION_RESPONSE_PREFIX, MLR_SET_CONTACT_FUNCTION_RESPONSE_LEN);
// }

// MLR_Modem_Error MLR_Modem::SetContactFunction(uint8_t contactFunction, bool saveValue)
// {
//     return m_SetByteValue(MLR_SET_CONTACT_FUNCTION_PREFIX_STRING, contactFunction, saveValue, MLR_SET_CONTACT_FUNCTION_RESPONSE_PREFIX, MLR_SET_CONTACT_FUNCTION_RESPONSE_LEN);
// }

MLR_Modem_Error MLR_Modem::SendRawCommand(const char *command, char *responseBuffer, size_t bufferSize, uint32_t timeoutMs)
{
    if (!command || !responseBuffer || bufferSize == 0)
    {
        MLR_DEBUG_PRINTLN("[MLR_Modem] SendRawCommand: Invalid args.");
        return MLR_Modem_Error::InvalidArg;
    }

    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        MLR_DEBUG_PRINTLN("[MLR_Modem] SendRawCommand: Busy with async command.");
        return MLR_Modem_Error::Busy;
    }

    MLR_DEBUG_PRINTF("[MLR_Modem] SendRawCommand: Sending raw command (timeout=%lu ms)...\n", timeoutMs);
    m_WriteString(command);

    MLR_Modem_Error rv = m_WaitCmdResponse(timeoutMs);

    if (rv != MLR_Modem_Error::Ok)
    {
        MLR_DEBUG_PRINTF("[MLR_Modem] SendRawCommand: Failed waiting for response. err=%d\n", (int)rv);
        responseBuffer[0] = '\0';
        return rv;
    }

    if (m_rxIdx >= bufferSize)
    {
        MLR_DEBUG_PRINTF("[MLR_Modem] SendRawCommand: Response length (%u) exceeds buffer size (%zu).\n", m_rxIdx, bufferSize);
        responseBuffer[0] = '\0';
        return MLR_Modem_Error::BufferTooSmall;
    }

    memcpy(responseBuffer, m_rxMessage, m_rxIdx);
    responseBuffer[m_rxIdx] = '\0';
    MLR_DEBUG_PRINTF("[MLR_Modem] SendRawCommand: Response received: %s\n", responseBuffer);

    return MLR_Modem_Error::Ok;
}

MLR_Modem_Error MLR_Modem::SendRawCommandAsync(const char *command, uint32_t timeoutMs)
{
    if (!command)
    {
        return MLR_Modem_Error::InvalidArg;
    }

    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(command);
    m_asyncExpectedResponse = MLR_Modem_Response::GenericResponse;
    m_StartTimeout(timeoutMs); // Start the async timeout

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
    m_WriteString(cmdHeader.data());
    m_pUart->write(pMsg, len);
    m_WriteString("\r\n");

    MLR_Modem_Error rv = m_WaitCmdResponse();

    // check transmission response
    uint8_t transmissionResponse{};
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessageHexByte(&transmissionResponse, MLR_TRANSMISSION_RESPONSE_LEN, MLR_TRANSMISSION_RESPONSE_PREFIX);
    }

    // check if length of data in response is correct
    if (rv == MLR_Modem_Error::Ok && transmissionResponse != len)
    {
        rv = MLR_Modem_Error::Fail;
    }

    // check information response
    if (rv == MLR_Modem_Error::Ok)
    {
        if (m_mode == MLR_ModemMode::LoRaCmd)
        {
            rv = m_WaitCmdResponse(15000);
        }
        else
        {
            rv = m_WaitCmdResponse(11);
        }
    }

    // check if transmission has been completed
    uint8_t informationResponse{};
    if (m_mode == MLR_ModemMode::LoRaCmd)
    {
        if (rv == MLR_Modem_Error::Ok)
        {
            rv = m_HandleMessageHexByte(&informationResponse, MLR_INFORMATION_RESPONSE_LEN, MLR_INFORMATION_RESPONSE_PREFIX);
        }

        if (rv != MLR_Modem_Error::Ok)
        {
            rv = MLR_Modem_Error::Fail;
        }
        else
        {
            switch (informationResponse)
            {
                // fallthrough
            case MLR_INFORMATION_RESPONSE_ERR_OTHER_WAVES:
            case MLR_INFORMATION_RESPONSE_ERR_NO_TX:
                rv = MLR_Modem_Error::FailLbt;
                break;
            default:
                break;
            }
        }
    }
    else
    {
        // FSK mode:
        //  if send OK, no *IR response. Carrier sense error results in *IR=01
        if (rv != MLR_Modem_Error::Ok)
        {
            // timeout mean send ok!
            rv = MLR_Modem_Error::Ok;
        }
        else
        {
            rv = m_HandleMessageHexByte(&informationResponse, MLR_INFORMATION_RESPONSE_LEN, MLR_INFORMATION_RESPONSE_PREFIX);
            if (rv != MLR_Modem_Error::Ok)
            {
                rv = MLR_Modem_Error::Fail;
            }
            else
            {
                switch (informationResponse)
                {
                case MLR_INFORMATION_RESPONSE_ERR_NO_TX:
                    rv = MLR_Modem_Error::FailLbt;
                    break;
                default:
                    break;
                }
            }
        }
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::TransmitDataFireAndForget(const uint8_t *pMsg, uint8_t len)
{
    if (!pMsg || len == 0)
    {
        return MLR_Modem_Error::InvalidArg;
    }

    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    std::array<char, 6> cmdHeader;
    snprintf(cmdHeader.data(), cmdHeader.size(), "%s%02X", MLR_TRANSMISSION_PREFIX_STRING, static_cast<unsigned>(len));
    m_WriteString(cmdHeader.data());
    m_pUart->write(pMsg, len);
    m_WriteString("\r\n");

    MLR_Modem_Error rv = m_WaitCmdResponse();

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
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(MLR_GET_RSSI_CURRENT_CHANNEL_STRING);
    m_asyncExpectedResponse = MLR_Modem_Response::RssiCurrentChannel;
    MLR_Modem_Error rv = MLR_Modem_Error::Ok;

    return rv;
}

MLR_Modem_Error MLR_Modem::GetSerialNumberAsync()
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(MLR_GET_SERIAL_NUMBER_STRING);
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
    switch (m_Parse())
    {
    case MLR_ModemCmdState::Parsing:
        // nop
        break;

    case MLR_ModemCmdState::Garbage:
        MLR_DEBUG_PRINTLN("[MLR Work] Work: Parser encountered garbage.");
        if (m_pCallback)
        {
            // Garbage
        }
        break;
    case MLR_ModemCmdState::Overflow:
        MLR_DEBUG_PRINTLN("[MLR Work] Work: Parser encountered overflow.");
        if (m_pCallback)
        {
            // Overflow
        }
        break;
    case MLR_ModemCmdState::FinishedCmdResponse:
        MLR_DEBUG_PRINTF("[MLR Work] Work: Finished CMD response, dispatching async.\n");
        m_DispatchCmdResponseAsync();
        break;
    case MLR_ModemCmdState::FinishedDrResponse:
        MLR_DEBUG_PRINTF("[MLR Work] Work: Finished DR response (Len=%u). Calling callback.\n", m_drMessageLen);
        if (m_pCallback)
        {
            // FinishedDrResponse
            m_pCallback(MLR_Modem_Error::Ok, MLR_Modem_Response::DataReceived, 0, &m_drMessage[0], m_drMessageLen);
        }
        break;
    default:
        MLR_DEBUG_PRINTLN(""); // Final newline for RX log
        break;
    }
}

void MLR_Modem::m_WriteString(const char *pString)
{
    size_t len = strlen(pString);
    MLR_DEBUG_PRINT("[MLR TX]: ");
    MLR_DEBUG_WRITE(reinterpret_cast<const uint8_t *>(pString), len);
    m_pUart->write(reinterpret_cast<const uint8_t *>(pString), len);
}

uint8_t MLR_Modem::m_ReadByte()
{
    if (m_oneByteBuf != -1)
    {
        int rcv_int = m_oneByteBuf;
        m_oneByteBuf = -1;
        uint8_t rcv = static_cast<uint8_t>(rcv_int);
        if (rcv >= 32 && rcv <= 126)
        {
            MLR_DEBUG_WRITE(rcv);
        }
        else if (rcv == '\r')
        {
            MLR_DEBUG_PRINT("<CR>");
        }
        else if (rcv == '\n')
        {
            MLR_DEBUG_PRINT("<LF>\n");
        }
        else
        {
            MLR_DEBUG_PRINTF("<%02X>", rcv);
        }
        return rcv;
    }

    if (m_pUart->available())
    {
        int rcv_int = m_pUart->read();
        if (rcv_int != -1)
        {
            uint8_t rcv = static_cast<uint8_t>(rcv_int);
            if (rcv >= 32 && rcv <= 126)
            {
                MLR_DEBUG_WRITE(rcv);
            }
            else if (rcv == '\r')
            {
                MLR_DEBUG_PRINT("<CR>");
            }
            else if (rcv == '\n')
            {
                MLR_DEBUG_PRINT("<LF>\n");
            }
            else
            {
                MLR_DEBUG_PRINTF("<%02X>", rcv);
            }
            return rcv;
        }
    }

    // Should not happen if available() is checked, but as a fallback.
    return 0;
}

void MLR_Modem::m_UnreadByte(uint8_t unreadByte)
{
    m_oneByteBuf = unreadByte;
}

void MLR_Modem::m_ClearUnreadByte()
{
    m_oneByteBuf = -1;
}

uint32_t MLR_Modem::m_Read(uint8_t *pDst, uint32_t count)
{
    if (m_oneByteBuf != -1)
    {
        *pDst++ = static_cast<uint8_t>(m_oneByteBuf);
        m_oneByteBuf = -1;
        --count;
    }

    return m_pUart->readBytes(pDst, count);
}

void MLR_Modem::m_ResetParser()
{
    m_parserState = MLR_ModemParserState::Start;
    m_ClearUnreadByte();
}

void MLR_Modem::m_ClearOneLine()
{
    m_pUart->setTimeout(500);
    m_pUart->readStringUntil('\n');
}

void MLR_Modem::m_FlushGarbage()
{
    MLR_DEBUG_PRINT("[MLR Flush]: Flushing garbage... ");
    // remove all remaining garbage from the pipeline, except '*' implies a valid message will follow
    // don't care about special cases
    if (m_oneByteBuf == -1)
    {
        while (m_pUart->available())
        {
            if ('*' == m_ReadByte())
            {
                m_UnreadByte('*');
                MLR_DEBUG_PRINT(" Found '*'.");
                break;
            }
        }
    }
    m_parserState = MLR_ModemParserState::Start;
    MLR_DEBUG_PRINTLN(" Flushed & Reset.");
}

MLR_ModemCmdState MLR_Modem::m_Parse()
{
    while (m_pUart->available())
    {
        switch (m_parserState)
        {
        case MLR_ModemParserState::Start:
            m_rxIdx = 0;
            m_rxMessage[m_rxIdx] = m_ReadByte();

            if (m_rxMessage[m_rxIdx] == '*')
            {
                ++m_rxIdx;
                m_parserState = MLR_ModemParserState::ReadCmdFirstLetter;
            }
#if 0 
            // TODO discuss whether we want to read the non CMD-Formatted messages, or just delete them silently
            else if (isupper(m_rxMessage[m_rxIdx]))
            {
               ++m_rxIdx;
               m_parserState = MLR_ModemParserState::ReadRawString;
            }
#endif
            else // garbage
            {
                MLR_DEBUG_PRINTF("\n[MLR Parse]: Expected '*', got 0x%02X. Flushing.\n", m_rxMessage[m_rxIdx]);
                m_FlushGarbage();
                // clearing the pipeline is not reported as error
                return MLR_ModemCmdState::Parsing;
            }
            break;

        case MLR_ModemParserState::ReadCmdFirstLetter:
            m_rxMessage[m_rxIdx] = m_ReadByte();

            if (isupper(m_rxMessage[m_rxIdx]))
            {
                ++m_rxIdx;
                m_parserState = MLR_ModemParserState::ReadCmdSecondLetter;
            }
            else
            {
                if (m_rxMessage[m_rxIdx] == '*') // two '*' in a row, ignore first
                {
                    m_UnreadByte('*');
                }
                MLR_DEBUG_PRINTF("\n[MLR Parse]: Expected A-Z, got 0x%02X. Flushing.\n", m_rxMessage[m_rxIdx]);
                m_FlushGarbage();
                return MLR_ModemCmdState::Garbage;
            }
            break;

        case MLR_ModemParserState::ReadCmdSecondLetter:
            m_rxMessage[m_rxIdx] = m_ReadByte();

            if (isupper(m_rxMessage[2]))
            {
                ++m_rxIdx;
                m_parserState = MLR_ModemParserState::ReadCmdParam;
            }
            else
            {
                if (m_rxMessage[2] == '*')
                {
                    m_UnreadByte('*'); // keep unexpected '*' for next message
                }
                MLR_DEBUG_PRINTF("\n[MLR Parse]: Expected A-Z, got 0x%02X. Flushing.\n", m_rxMessage[m_rxIdx]);
                m_FlushGarbage();
                return MLR_ModemCmdState::Garbage;
            }
            break;

        case MLR_ModemParserState::ReadCmdParam:
            m_rxMessage[m_rxIdx] = m_ReadByte();

            if ((m_rxMessage[1] == 'D') && (m_rxMessage[2] == 'R') && (m_rxMessage[3] == '='))
            {
                // @DR telegram
                ++m_rxIdx;
                m_parserState = MLR_ModemParserState::RadioDrSize;
            }
            else if (isupper(m_rxMessage[1]) && isupper(m_rxMessage[2]) && (m_rxMessage[3] == '='))
            {
                ++m_rxIdx;
                m_parserState = MLR_ModemParserState::ReadCmdUntilCR;
            }
            else
            {
                if (m_rxMessage[m_rxIdx] == '*') // another '*', ignore the three read characters
                {
                    m_UnreadByte('*');
                }
                MLR_DEBUG_PRINTF("\n[MLR Parse]: Unexpected param char 0x%02X. Flushing.\n", m_rxMessage[m_rxIdx]);
                m_FlushGarbage();
                return MLR_ModemCmdState::Garbage;
            }
            break;

        case MLR_ModemParserState::RadioDrSize:
            m_rxMessage[m_rxIdx] = m_ReadByte();
            ++m_rxIdx;
            if (m_rxIdx < 6)
            {
                // parser cannot continue until there are at least two characters
                return MLR_ModemCmdState::Parsing;
            }

            if (isxdigit(m_rxMessage[4]) && isxdigit(m_rxMessage[5]))
            {
                m_drMessagePresent = false;
                uint32_t msgLen = 0;
                s_ParseHex(&m_rxMessage[4], 2, &msgLen);
                m_drMessageLen = msgLen;
                m_rxIdx = 0; // now m_rxIdx to m_drMessage Buffer
                m_parserState = MLR_ModemParserState::RadioDrPayload;
            }
            else
            {
                m_FlushGarbage();
                return MLR_ModemCmdState::Garbage;
            }
            break;

        case MLR_ModemParserState::RadioDrPayload:
        {
            // example DR:
            // *DR=05hallo\r\n

            m_drMessage[m_rxIdx] = m_ReadByte();
            ++m_rxIdx;

            if ((m_drMessageLen + 2 - m_rxIdx) == 0)
            {
                if ((m_drMessage[m_rxIdx - 2] == '\r') && m_drMessage[m_rxIdx - 1] == '\n')
                {
                    m_drMessage[m_rxIdx - 2] = 0; // set null at end of the message
                    m_rxIdx = 0;
                    m_rxMessage[0] = 0; // "destroy" the old CMD message, so nobody will expect the new message to be a regular command response instead of a radio packet
                    m_drMessagePresent = true;
                    m_parserState = MLR_ModemParserState::Start;
                    return MLR_ModemCmdState::FinishedDrResponse;
                }
                else
                {
                    m_FlushGarbage();
                    return MLR_ModemCmdState::Garbage;
                }
            }

            break;
        }

        case MLR_ModemParserState::ReadCmdUntilCR:
            m_rxMessage[m_rxIdx] = m_ReadByte();

            if (m_rxMessage[m_rxIdx] == '\r')
            {
                ++m_rxIdx;
                if (m_rxIdx == sizeof(m_rxMessage))
                {
                    m_parserState = MLR_ModemParserState::Start;
                    return MLR_ModemCmdState::Overflow;
                }
                else
                {
                    m_parserState = MLR_ModemParserState::ReadCmdUntilLF;
                }
            }
            else if (m_rxMessage[m_rxIdx] == '\n') // unexpected end of command, reset parser
            {
                m_FlushGarbage();
                return MLR_ModemCmdState::Garbage;
            }
            else if (m_rxMessage[m_rxIdx] == '*') // another '*', ignore already read characters
            {
                m_UnreadByte('*');
                m_FlushGarbage();
                return MLR_ModemCmdState::Garbage;
            }
            else
            {
                ++m_rxIdx;
                if (m_rxIdx == sizeof(m_rxMessage))
                {
                    m_parserState = MLR_ModemParserState::Start;
                    return MLR_ModemCmdState::Overflow;
                }
            }
            break;

        case MLR_ModemParserState::ReadCmdUntilLF:
            m_rxMessage[m_rxIdx] = m_ReadByte();
            if (m_rxMessage[m_rxIdx] == '\n')
            {
                // decrease index pointer to compensate for increasing with CR;
                // CR\LF is not considered part of message
                --m_rxIdx;
                m_parserState = MLR_ModemParserState::Start;
                return MLR_ModemCmdState::FinishedCmdResponse;
            }
            else // garbage
            {
                if (m_rxMessage[m_rxIdx] == '*') // another '*', ignore already read characters
                {
                    m_UnreadByte('*');
                }
                m_FlushGarbage();
                return MLR_ModemCmdState::Garbage;
            }
            break;

        default:
            // this should never be reached
            m_parserState = MLR_ModemParserState::Start;
            m_rxIdx = 0;
            break;
        }
    }

    return MLR_ModemCmdState::Parsing;
}

MLR_Modem_Error MLR_Modem::m_WaitCmdResponse(uint32_t ms)
{
    // We might just receiving a Dr Telegram, when sending a normal command to the modem.
    // Thus while waiting for the command response, receiving a Dr message must be taken into account.
    MLR_DEBUG_PRINTF("[MLR Wait]: Waiting up to %lu ms...\n", ms);
    m_StartTimeout(ms);
    while (!m_IsTimeout())
    {
        switch (m_Parse())
        {
        case MLR_ModemCmdState::Parsing:
            // nothing to do
            break;

        case MLR_ModemCmdState::FinishedCmdResponse:
            MLR_DEBUG_PRINTF("[MLR Wait]: Finished CMD response received: '%.*s'\n", m_rxIdx, m_rxMessage);
            return MLR_Modem_Error::Ok;
            break;

        case MLR_ModemCmdState::FinishedDrResponse:
            MLR_DEBUG_PRINTF("[MLR Wait]: Intervening DR received (Len=%u). Calling callback...\n", m_drMessageLen);
            if (m_pCallback)
            {
                m_pCallback(MLR_Modem_Error::Ok, MLR_Modem_Response::DataReceived, 0, m_drMessage, m_drMessageLen);
            }
            MLR_DEBUG_PRINTLN("[MLR Wait]: Continuing to wait for original CMD response...");
            break;

        default:
            MLR_DEBUG_PRINTLN("[MLR Wait]: Parser encountered error (Garbage/Overflow/Fail).");
            return MLR_Modem_Error::Fail;
        }

        delay(1);
    }
    m_parserState = MLR_ModemParserState::Start;
    MLR_DEBUG_PRINTLN("[MLR Wait]: Timeout.");
    return MLR_Modem_Error::Fail;
}

void MLR_Modem::m_SetExpectedResponses(MLR_Modem_Response ep0, MLR_Modem_Response ep1, MLR_Modem_Response ep2)
{
    m_asyncExpectedResponses[0] = ep0;
    m_asyncExpectedResponses[1] = ep1;
    m_asyncExpectedResponses[2] = ep2;
}

MLR_Modem_Error MLR_Modem::m_DispatchCmdResponseAsync()
{
    MLR_Modem_Error err = MLR_Modem_Error::Fail;

    switch (m_asyncExpectedResponse)
    {
    case MLR_Modem_Response::Idle:
        MLR_DEBUG_PRINTLN("[MLR Async] Warning: Received response but no async command pending (or late sync response).");
        break;
    case MLR_Modem_Response::ParseError:
        MLR_DEBUG_PRINTLN("[MLR Async] Error: Parse error during async command processing.");
        break;
    case MLR_Modem_Response::Timeout:
        MLR_DEBUG_PRINTLN("[MLR Async] Error: Timeout during async command processing.");
        break;
    case MLR_Modem_Response::ShowMode:
        break;
    case MLR_Modem_Response::SaveValue:
        break;
    case MLR_Modem_Response::Channel:
        break;
    case MLR_Modem_Response::SerialNumber:
        if (m_pCallback)
        {
            uint32_t sn{};
            err = m_HandleMessage_SN(&sn);
            int32_t value = static_cast<int32_t>(sn);
            m_pCallback(err, MLR_Modem_Response::SerialNumber, value, nullptr, 0);
        }
        break;
    case MLR_Modem_Response::MLR_Modem_DtIr:
        if (m_pCallback)
        {
            uint8_t irValue{};
            err = m_HandleMessageHexByte(&irValue, MLR_INFORMATION_RESPONSE_LEN, MLR_INFORMATION_RESPONSE_PREFIX);
            m_pCallback(err, MLR_Modem_Response::MLR_Modem_DtIr, static_cast<int32_t>(irValue), nullptr, 0);
        }
        break;
    case MLR_Modem_Response::DataReceived:
        break;
    case MLR_Modem_Response::RssiLastRx:
        break;
    case MLR_Modem_Response::RssiCurrentChannel:
        if (m_pCallback)
        {
            int16_t rssi{};
            err = m_HandleMessage_RA(&rssi);
            m_pCallback(err, MLR_Modem_Response::RssiCurrentChannel, static_cast<int32_t>(rssi), nullptr, 0);
        }
        break;
    case MLR_Modem_Response::UserID:
        break;
    case MLR_Modem_Response::CarrierSenseRssi:
        break;
    case MLR_Modem_Response::FactoryReset:
        break;
    case MLR_Modem_Response::BaudRate:
        break;
    case MLR_Modem_Response::GenericResponse:
        if (m_pCallback)
        {
            const uint8_t *payloadPtr = m_rxMessage;
            uint16_t payloadLen = m_rxIdx; // Length of the response (excluding CR/LF)
            err = MLR_Modem_Error::Ok;     // Assume OK since we got a response
            m_pCallback(err, MLR_Modem_Response::GenericResponse, 0, payloadPtr, payloadLen);
        }
        break;
    default:
        break;
    }

    m_asyncExpectedResponse = MLR_Modem_Response::Idle;
    return err;
}

MLR_Modem_Error MLR_Modem::m_SetByteValue(const char *cmdPrefix, uint8_t value, bool saveValue, const char *respPrefix, size_t respLen)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    std::array<char, 12> cmdStr;
    snprintf(cmdStr.data(), cmdStr.size(), "%s%02X%s\r\n", cmdPrefix, static_cast<unsigned>(value), (saveValue ? ("/W") : ("")));
    m_WriteString(cmdStr.data());

    MLR_Modem_Error rv = m_WaitCmdResponse();
    if (rv == MLR_Modem_Error::Ok && saveValue)
    {
        rv = m_HandleMessage_WR();
        if (rv == MLR_Modem_Error::Ok)
        {
            rv = m_WaitCmdResponse();
        }
    }

    uint8_t responseVal{};
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessageHexByte(&responseVal, respLen, respPrefix);
    }

    if (rv == MLR_Modem_Error::Ok && responseVal != value)
    {
        rv = MLR_Modem_Error::Fail;
    }

    return rv;
}

MLR_Modem_Error MLR_Modem::m_GetByteValue(const char *cmdString, uint8_t *pValue, const char *respPrefix, size_t respLen)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_WriteString(cmdString);

    MLR_Modem_Error rv = m_WaitCmdResponse();
    if (rv == MLR_Modem_Error::Ok)
    {
        rv = m_HandleMessageHexByte(pValue, respLen, respPrefix);
    }
    return rv;
}

MLR_Modem_Error MLR_Modem::m_HandleMessage_WR()
{
    uint16_t messageLen = m_rxIdx;

    if ((messageLen == MLR_WRITE_VALUE_RESPONSE_LEN) && !strncmp(MLR_WRITE_VALUE_RESPONSE_PREFIX, (char *)&m_rxMessage[0], MLR_WRITE_VALUE_RESPONSE_LEN))
    {
        return MLR_Modem_Error::Ok;
    }

    return MLR_Modem_Error::Fail;
}

MLR_Modem_Error MLR_Modem::m_HandleMessageHexByte(uint8_t *pValue, uint32_t responseLen, const char *responsePrefix)
{
    uint16_t messageLen = m_rxIdx;
    if (messageLen != responseLen)
    {
        return MLR_Modem_Error::Fail; // message wrong size
    }

    uint32_t responsePrefixLen = strlen(responsePrefix);
    uint16_t channelHexIndex = responsePrefixLen;
    if (!strncmp(responsePrefix, (char *)&m_rxMessage[0], responsePrefixLen))
    {
        uint32_t value{};
        if (s_ParseHex(&m_rxMessage[channelHexIndex], 2, &value))
        {
            *pValue = value;
            return MLR_Modem_Error::Ok;
        }

        return MLR_Modem_Error::Fail;
    }

    return MLR_Modem_Error::Fail;
}

MLR_Modem_Error MLR_Modem::m_HandleMessageHexWord(uint16_t *pValue, uint32_t responseLen, const char *responsePrefix)
{
    uint16_t messageLen = m_rxIdx;
    if (messageLen != responseLen)
    {
        return MLR_Modem_Error::Fail; // message wrong size
    }

    uint32_t responsePrefixLen = strlen(responsePrefix);
    uint16_t valIndex = responsePrefixLen;
    if (!strncmp(responsePrefix, (char *)&m_rxMessage[0], responsePrefixLen))
    {
        uint32_t value{};
        if (s_ParseHex(&m_rxMessage[valIndex], 4, &value))
        {
            *pValue = static_cast<uint16_t>(value);
            return MLR_Modem_Error::Ok;
        }
    }
    return MLR_Modem_Error::Fail;
}

MLR_Modem_Error MLR_Modem::m_HandleMessage_RS(int16_t *pRssi)
{
    uint16_t messageLen = m_rxIdx;
    if ((messageLen < MLR_GET_RSSI_LAST_RX_RESPONSE_MIN_LEN) || (messageLen > MLR_GET_RSSI_LAST_RX_RESPONSE_MAX_LEN))
    {
        return MLR_Modem_Error::Fail; // message wrong size
    }

    uint16_t responsePrefixLen = static_strlen(MLR_GET_RSSI_LAST_RX_RESPONSE_PREFIX);
    if (!strncmp(MLR_GET_RSSI_LAST_RX_RESPONSE_PREFIX, (char *)&m_rxMessage[0], responsePrefixLen))
    {
        // check the last chars to be "dBm"
        if ((m_rxMessage[messageLen - 3] == 'd') &&
            (m_rxMessage[messageLen - 2] == 'B') &&
            (m_rxMessage[messageLen - 1] == 'm'))
        {
            m_rxMessage[messageLen - 3] = 0; // cut the "d" which will make the number string a zero terminated string
            uint8_t *pEnd;
            long result = strtol((char *)&m_rxMessage[4], (char **)&pEnd, 10);

            if (pEnd != &m_rxMessage[messageLen - 3])
            {
                // indicating the parsing stopped early because some char was not a number digit
                return MLR_Modem_Error::Fail;
            }

            *pRssi = (int16_t)result;
            return MLR_Modem_Error::Ok;
        }

        return MLR_Modem_Error::Fail;
    }

    return MLR_Modem_Error::Fail;
}

// check if the received message is of RA format and fill the RSSI
MLR_Modem_Error MLR_Modem::m_HandleMessage_RA(int16_t *pRssi)
{
    uint16_t messageLen = m_rxIdx;
    if ((messageLen < MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_MIN_LEN) || (messageLen > MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_MAX_LEN))
    {
        return MLR_Modem_Error::Fail; // message wrong size
    }

    uint16_t responsePrefixLen = static_strlen(MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_PREFIX);
    if (!strncmp(MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_PREFIX, (char *)&m_rxMessage[0], responsePrefixLen))
    {
        // check the last chars to be "dBm"
        if ((m_rxMessage[messageLen - 3] == 'd') &&
            (m_rxMessage[messageLen - 2] == 'B') &&
            (m_rxMessage[messageLen - 1] == 'm'))
        {
            m_rxMessage[messageLen - 3] = 0; // cut the "d" which will make the number string a zero terminated string
            uint8_t *pEnd;
            long result = strtol((char *)&m_rxMessage[4], (char **)&pEnd, 10);

            if (pEnd != &m_rxMessage[messageLen - 3])
            {
                // indicating the parsing stopped early because some char was not a number digit
                return MLR_Modem_Error::Fail;
            }

            *pRssi = (int16_t)result;
            return MLR_Modem_Error::Ok;
        }

        return MLR_Modem_Error::Fail;
    }

    return MLR_Modem_Error::Fail;
}

MLR_Modem_Error MLR_Modem::m_HandleMessage_SN(uint32_t *pSerialNumber)
{
    uint16_t messageLen = m_rxIdx;

    uint16_t responsePrefixLen = static_strlen(MLR_GET_SERIAL_NUMBER_RESPONSE_PREFIX);
    if ((messageLen == MLR_GET_SERIAL_NUMBER_RESPONSE_LEN) && !strncmp(MLR_GET_SERIAL_NUMBER_RESPONSE_PREFIX, (char *)&m_rxMessage[0], responsePrefixLen))
    {
        uint32_t serialNumber{};

        // maybe leading "S" ahead of hex code, take into account! (*SN=S0000001 vs *SN=00000001)
        uint8_t startIdx = 4;
        uint8_t hexLen = 8;
        if (!isdigit(m_rxMessage[4]))
        {
            startIdx = 5;
            hexLen = 7;
        }

        if (!s_ParseDec(&m_rxMessage[startIdx], hexLen, &serialNumber))
        {
            return MLR_Modem_Error::Fail;
        }
        else
        {
            if (pSerialNumber)
            {
                *pSerialNumber = serialNumber;
            }

            return MLR_Modem_Error::Ok;
        }
    }

    return MLR_Modem_Error::Fail;
}

// check if the received message is "*IZ=OK"
MLR_Modem_Error MLR_Modem::m_HandleMessage_IZ()
{
    uint16_t messageLen = m_rxIdx;

    if ((messageLen == MLR_SET_IZ_RESPONSE_LEN_OK) && !strncmp(MLR_SET_IZ_RESPONSE_PREFIX_OK, (char *)&m_rxMessage[0], MLR_SET_IZ_RESPONSE_LEN_OK))
    {
        return MLR_Modem_Error::Ok;
    }

    // Note: This does not explicitly check for *IZ=NG [cite: 468]
    return MLR_Modem_Error::Fail;
}

void MLR_Modem::setDebugStream(Stream *debugStream)
{
    m_pDebugStream = debugStream;
}