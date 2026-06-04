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

// --- Circuit Design modem protocol strings (local to this driver) ---
static constexpr char MLR_NVM_SAVE_RESPONSE[] = "*WR=PS";
static constexpr size_t MLR_NVM_SAVE_RESPONSE_LEN = 6;
static constexpr char MLR_CMD_WRITE_SUFFIX[] = "/W";
static constexpr char MLR_VAL_ON[] = "ON";
static constexpr char MLR_VAL_OFF[] = "OF";
static constexpr size_t MLR_INFORMATION_RESPONSE_LEN = 6;
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_NO_TX = 1;
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_OTHER_WAVES = 2;
static constexpr uint8_t MLR_INFORMATION_RESPONSE_ERR_OK = 3;

// FSK mode does not return *IR=03 on success. After *DT=XX is accepted,
// wait this many milliseconds for a possible *IR=01/02 (LBT failure).
// If none arrives, the transmission is treated as successful.
static constexpr uint32_t MLR_LBT_CHECK_TIMEOUT_MS = 60;

MLR_Modem_Error MLR_Modem::begin(Stream &pUart, MLR_Modem_FrequencyModel frequencyModel,
                                 MLR_Modem_AsyncCallback pCallback)
{
    initSerial(pUart); // Base class init
    setNvmConfig(MLR_NVM_SAVE_RESPONSE, MLR_NVM_SAVE_RESPONSE_LEN,
                 MLR_CMD_WRITE_SUFFIX, MLR_VAL_ON, MLR_VAL_OFF);

    m_asyncExpectedResponse = MLR_Modem_Response::Idle;
    m_pCallback = pCallback;
    m_frequencyModel = frequencyModel;
    m_parserState = MLR_ModemParserState::Start;
    m_drMessagePresent = false;
    m_drMessageLen = 0;
    m_irMessagePresent = false;
    m_irValue = 0;
    m_autoRssiPending = false;

    SM_DEBUG_PRINTLN("begin: Getting current mode...");

    MLR_Modem_Error err = GetMode(&m_mode);
    if (err != MLR_Modem_Error::Ok)
    {
        SM_DEBUG_PRINTF("begin: GetMode failed! err=%d\n", (int)err);
        return err;
    }

    SM_DEBUG_PRINTF("begin: Initialization successful. Mode=%d\n", (int)m_mode);
    return MLR_Modem_Error::Ok;
}

MLR_Modem_Error MLR_Modem::SetChannel(uint8_t channel, bool saveValue)
{
    if ((channel < MLR_CHANNEL_MIN_429) || (channel > MLR_CHANNEL_MAX_429))
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
    char buf[16];
    char *p = appendStr(buf, buf, MLR_GET_USERID_STRING);
    appendStr(buf, p, "\r\n");

    MLR_Modem_Error err = enqueueCommand(buf, CommandType::Simple);
    if (err != MLR_Modem_Error::Ok) return err;

    err = waitForSyncComplete(500);
    if (err == MLR_Modem_Error::Ok)
    {
        err = m_HandleMessageHexWord(pUserID, MLR_GET_USERID_RESPONSE_LEN, MLR_GET_USERID_RESPONSE_PREFIX);
    }
    return err;
}

MLR_Modem_Error MLR_Modem::GetRssiLastRx(int16_t *pRssi)
{
    char buf[16];
    char *p = appendStr(buf, buf, MLR_GET_RSSI_LAST_RX_STRING);
    appendStr(buf, p, "\r\n");

    MLR_Modem_Error err = enqueueCommand(buf, CommandType::Simple);
    if (err != MLR_Modem_Error::Ok) return err;

    err = waitForSyncComplete(500);
    if (err == MLR_Modem_Error::Ok)
    {
        err = m_HandleMessage_RS(pRssi);
    }
    return err;
}

MLR_Modem_Error MLR_Modem::GetRssiCurrentChannel(int16_t *pRssi)
{
    char buf[16];
    char *p = appendStr(buf, buf, MLR_GET_RSSI_CURRENT_CHANNEL_STRING);
    appendStr(buf, p, "\r\n");

    MLR_Modem_Error err = enqueueCommand(buf, CommandType::Simple);
    if (err != MLR_Modem_Error::Ok) return err;

    err = waitForSyncComplete(500);
    if (err == MLR_Modem_Error::Ok)
    {
        err = m_HandleMessage_RA(pRssi);
    }
    return err;
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
    char buf[16];
    char *p = appendStr(buf, buf, MLR_GET_SERIAL_NUMBER_STRING);
    appendStr(buf, p, "\r\n");

    MLR_Modem_Error err = enqueueCommand(buf, CommandType::Simple);
    if (err != MLR_Modem_Error::Ok) return err;

    err = waitForSyncComplete(500);
    if (err == MLR_Modem_Error::Ok)
    {
        err = m_HandleMessage_SN(pSerialNumber);
    }
    return err;
}

MLR_Modem_Error MLR_Modem::FactoryReset()
{
    MLR_Modem_Error err = enqueueCommand(MLR_CMD_IZ, CommandType::NvmSave);
    if (err != MLR_Modem_Error::Ok) return err;

    err = waitForSyncComplete(2000);
    if (err == MLR_Modem_Error::Ok)
    {
        err = m_HandleMessage_IZ();
    }
    return err;
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
    case 1200: baudCode = 0x12; break;
    case 2400: baudCode = 0x24; break;
    case 4800: baudCode = 0x48; break;
    case 9600: baudCode = 0x96; break;
    case 19200: baudCode = 0x19; break;
    default: return MLR_Modem_Error::InvalidArg;
    }

    return setByteValue(MLR_CMD_BAUDRATE, baudCode, saveValue, MLR_SET_BAUDRATE_RESPONSE_PREFIX, MLR_SET_BAUDRATE_RESPONSE_LEN);
}

MLR_Modem_Error MLR_Modem::SendRawCommand(const char *command, char *responseBuffer, size_t bufferSize, uint32_t timeoutMs)
{
    return sendRawCommand(command, responseBuffer, bufferSize, timeoutMs);
}

MLR_Modem_Error MLR_Modem::SendRawCommandAsync(const char *command, uint32_t timeoutMs)
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle)
    {
        return MLR_Modem_Error::Busy;
    }

    m_asyncExpectedResponse = MLR_Modem_Response::GenericResponse;
    return enqueueCommand(command, CommandType::Simple, timeoutMs);
}

MLR_Modem_Error MLR_Modem::TransmitData(const uint8_t *pMsg, uint8_t len)
{
    m_irMessagePresent = false;

    char cmdHeader[16];
    char *p = appendStr(cmdHeader, cmdHeader, MLR_TRANSMISSION_PREFIX_STRING);
    appendHex2(cmdHeader, p, len);

    MLR_Modem_Error err = enqueueTxCommand(cmdHeader, pMsg, len, "\r\n");
    if (err != MLR_Modem_Error::Ok) return err;

    // 1. Wait for *DT=len (Transmission accepted)
    err = waitForSyncComplete(2000);
    if (err != MLR_Modem_Error::Ok) return err;

    uint8_t txResp{};
    err = m_HandleMessageHexByte(&txResp, MLR_TRANSMISSION_RESPONSE_LEN, MLR_TRANSMISSION_RESPONSE_PREFIX);
    if (err != MLR_Modem_Error::Ok) return err;
    if (txResp != len) return MLR_Modem_Error::Fail;

    // 2. Wait for *IR=xx
    if (m_mode == MLR_ModemMode::LoRaCmd)
    {
        // LoRa returns *IR=03 on on-air success, or *IR=01/02 on LBT failure.
        uint32_t start = millis();
        while (!m_irMessagePresent && (millis() - start < 15000))
        {
            update();
            delay(1);
        }
        if (!m_irMessagePresent) return MLR_Modem_Error::Timeout;
        if (m_irValue == MLR_INFORMATION_RESPONSE_ERR_OTHER_WAVES ||
            m_irValue == MLR_INFORMATION_RESPONSE_ERR_NO_TX)
            return MLR_Modem_Error::FailLbt;
    }
    else
    {
        // FSK does not emit *IR=03 on success. Poll for MLR_LBT_CHECK_TIMEOUT_MS
        // and treat any *IR=01/02 within the window as LBT failure; otherwise OK.
        uint32_t start = millis();
        while (millis() - start < MLR_LBT_CHECK_TIMEOUT_MS)
        {
            update();
            if (m_irMessagePresent &&
                (m_irValue == MLR_INFORMATION_RESPONSE_ERR_NO_TX ||
                 m_irValue == MLR_INFORMATION_RESPONSE_ERR_OTHER_WAVES))
            {
                return MLR_Modem_Error::FailLbt;
            }
            delay(1);
        }
    }

    return MLR_Modem_Error::Ok;
}

MLR_Modem_Error MLR_Modem::TransmitDataAsync(const uint8_t *pMsg, uint8_t len)
{
    if (!pMsg || len == 0) return MLR_Modem_Error::InvalidArg;
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle) return MLR_Modem_Error::Busy;

    char cmdHeader[16];
    char *p = appendStr(cmdHeader, cmdHeader, MLR_TRANSMISSION_PREFIX_STRING);
    appendHex2(cmdHeader, p, len);


    m_asyncExpectedResponse = MLR_Modem_Response::TxComplete;
    return enqueueTxCommand(cmdHeader, pMsg, len, "\r\n");
}

MLR_Modem_Error MLR_Modem::GetRssiCurrentChannelAsync()
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle) return MLR_Modem_Error::Busy;

    char buf[16];
    char *p = appendStr(buf, buf, MLR_GET_RSSI_CURRENT_CHANNEL_STRING);
    appendStr(buf, p, "\r\n");

    m_asyncExpectedResponse = MLR_Modem_Response::RssiCurrentChannel;
    return enqueueCommand(buf, CommandType::Simple);
}

MLR_Modem_Error MLR_Modem::GetSerialNumberAsync()
{
    if (m_asyncExpectedResponse != MLR_Modem_Response::Idle) return MLR_Modem_Error::Busy;

    char buf[16];
    char *p = appendStr(buf, buf, MLR_GET_SERIAL_NUMBER_STRING);
    appendStr(buf, p, "\r\n");

    m_asyncExpectedResponse = MLR_Modem_Response::SerialNumber;
    return enqueueCommand(buf, CommandType::Simple);
}

MLR_Modem_Error MLR_Modem::GetPacket(const uint8_t **ppData, uint8_t *len)
{
    if (m_drMessagePresent)
    {
        *ppData = &m_drMessage[0];
        *len = m_drMessageLen;
        return MLR_Modem_Error::Ok;
    }
    return MLR_Modem_Error::Fail;
}

// --- SerialModemBase Virtual Implementation ---

void MLR_Modem::onRxDataReceived()
{
    if (m_drMessagePresent)
    {
        // Enrich the DataReceived event with RSSI (dBm) by issuing an internal
        // "@RS" query, deferring dispatch until *RS= arrives.
        if (m_autoRssiPending)
        {
            // A previous *DR is still waiting for its @RS to return. The parser
            // has overwritten the held buffer with this newer packet, so the
            // older one is silently lost (m_drMessage holds only this packet now).
            // Let the in-flight @RS resolve and dispatch this packet with its RSSI.
            return;
        }
        if (m_asyncExpectedResponse == MLR_Modem_Response::Idle)
        {
            char buf[8];
            char *p = appendStr(buf, buf, MLR_GET_RSSI_LAST_RX_STRING);
            appendStr(buf, p, "\r\n");
            if (enqueueCommand(buf, CommandType::Simple) == ModemError::Ok)
            {
                m_autoRssiPending = true;
                return;
            }
        }
        // Fallback: a user async command is in flight, or @RS could not be queued.
        // Dispatch immediately without RSSI so the packet is not lost.
        dispatchAsyncEvent(ModemError::Ok, MLR_Modem_Response::DataReceived, 0, &m_drMessage[0], m_drMessageLen);
        m_drMessagePresent = false;
        return;
    }

    // Check for *IR= information response
    uint8_t irVal;
    if (m_HandleMessageHexByte(&irVal, MLR_INFORMATION_RESPONSE_LEN, MLR_INFORMATION_RESPONSE_PREFIX) != ModemError::Ok)
        return;

    m_irMessagePresent = true;
    m_irValue = irVal;

    if (m_mode == MLR_ModemMode::LoRaCmd)
    {
        // LoRa: TxComplete callback is deferred until *IR arrives.
        // *IR=03 -> TxComplete, *IR=01/02 -> TxFailed.
        if (m_asyncExpectedResponse == MLR_Modem_Response::TxComplete)
        {
            bool txOk = (irVal == MLR_INFORMATION_RESPONSE_ERR_OK);
            ModemError txErr     = txOk ? ModemError::Ok      : ModemError::FailLbt;
            MLR_Modem_Response t = txOk ? MLR_Modem_Response::TxComplete
                                        : MLR_Modem_Response::TxFailed;
            dispatchAsyncEvent(txErr, t, (int32_t)irVal);
            m_asyncExpectedResponse = MLR_Modem_Response::Idle;
        }
    }
    else
    {
        // FSK: TxComplete was already dispatched on *DT=XX in onCommandComplete.
        // *IR=01/02 here means LBT failure -> dispatch TxFailed (dual callback, MU pattern).
        if (irVal == MLR_INFORMATION_RESPONSE_ERR_NO_TX ||
            irVal == MLR_INFORMATION_RESPONSE_ERR_OTHER_WAVES)
        {
            dispatchAsyncEvent(ModemError::FailLbt, MLR_Modem_Response::TxFailed, (int32_t)irVal);
        }
    }
}

void MLR_Modem::onCommandComplete(ModemError result)
{
    // Auto-RSSI-on-RX completion: handle the internal @RS issued by onRxDataReceived().
    // Always processed before the user-async path, since m_asyncExpectedResponse stays
    // Idle for this internally-issued command.
    if (m_autoRssiPending)
    {
        m_autoRssiPending = false;
        int16_t rssi = 0;
        if (result == ModemError::Ok)
        {
            (void)m_HandleMessage_RS(&rssi);
        }
        if (m_drMessagePresent)
        {
            dispatchAsyncEvent(ModemError::Ok, MLR_Modem_Response::DataReceived,
                               (int32_t)rssi, &m_drMessage[0], m_drMessageLen);
            m_drMessagePresent = false;
        }
        return;
    }

    if (m_asyncExpectedResponse == MLR_Modem_Response::Idle) return;

    MLR_Modem_Response respType = m_asyncExpectedResponse;
    int32_t value = 0;
    const uint8_t *pPayload = nullptr;
    uint16_t len = 0;

    if (result == ModemError::Ok)
    {
        switch (respType)
        {
        case MLR_Modem_Response::SerialNumber:
        {
            uint32_t sn;
            if (m_HandleMessage_SN(&sn) == ModemError::Ok) value = (int32_t)sn;
            else result = ModemError::Fail;
            break;
        }
        case MLR_Modem_Response::RssiCurrentChannel:
        {
            int16_t rssi;
            if (m_HandleMessage_RA(&rssi) == ModemError::Ok) value = (int32_t)rssi;
            else result = ModemError::Fail;
            break;
        }
        case MLR_Modem_Response::GenericResponse:
            pPayload = _rxBuffer;
            len = _rxIndex;
            break;
        case MLR_Modem_Response::TxComplete:
            // *DT=XX received (command accepted by modem).
            if (m_mode == MLR_ModemMode::LoRaCmd)
            {
                // LoRa: keep waiting; on-air result arrives later as *IR (handled in onRxDataReceived).
                return;
            }
            // FSK: no *IR=03 follows. Dispatch TxComplete now; if *IR=01/02 follows
            // within MLR_LBT_CHECK_TIMEOUT_MS, a separate TxFailed will be dispatched.
            break;
        default:
            break;
        }
    }

    dispatchAsyncEvent(result, respType, value, pPayload, len);
    m_asyncExpectedResponse = MLR_Modem_Response::Idle;
}

ModemParseResult MLR_Modem::parse()
{
    while (_uart->available() || _oneByteBuf != -1)
    {
        ModemParseResult result = ModemParseResult::Parsing;
        switch (m_parserState)
        {
        case MLR_ModemParserState::Start: result = m_HandleReadStart(); break;
        case MLR_ModemParserState::ReadCmdFirstLetter: result = m_HandleReadCmdFirstLetter(); break;
        case MLR_ModemParserState::ReadCmdSecondLetter: result = m_HandleReadCmdSecondLetter(); break;
        case MLR_ModemParserState::ReadCmdParam: result = m_HandleReadCmdParam(); break;
        case MLR_ModemParserState::RadioDrSize: result = m_HandleRadioDrSize(); break;
        case MLR_ModemParserState::RadioDrPayload: result = m_HandleRadioDrPayload(); break;
        case MLR_ModemParserState::ReadCmdUntilCR: result = m_HandleReadCmdUntilCR(); break;
        case MLR_ModemParserState::ReadCmdUntilLF: result = m_HandleReadCmdUntilLF(); break;
        default: m_parserState = MLR_ModemParserState::Start; break;
        }

        if (result != ModemParseResult::Parsing) return result;
    }
    return ModemParseResult::Parsing;
}

// --- Internal Parser Handlers ---

ModemParseResult MLR_Modem::m_HandleReadStart()
{
    _rxIndex = 0;
    _rxBuffer[_rxIndex] = readByte();
    if (_rxBuffer[_rxIndex] == '*')
    {
        ++_rxIndex;
        m_parserState = MLR_ModemParserState::ReadCmdFirstLetter;
    }
    else flushGarbage();
    return ModemParseResult::Parsing;
}

ModemParseResult MLR_Modem::m_HandleReadCmdFirstLetter()
{
    _rxBuffer[_rxIndex] = readByte();
    if (isupper(_rxBuffer[_rxIndex]))
    {
        ++_rxIndex;
        m_parserState = MLR_ModemParserState::ReadCmdSecondLetter;
    }
    else
    {
        if (_rxBuffer[_rxIndex] == '*') unreadByte('*');
        flushGarbage();
        return ModemParseResult::Garbage;
    }
    return ModemParseResult::Parsing;
}

ModemParseResult MLR_Modem::m_HandleReadCmdSecondLetter()
{
    _rxBuffer[_rxIndex] = readByte();
    if (isupper(_rxBuffer[_rxIndex]))
    {
        ++_rxIndex;
        m_parserState = MLR_ModemParserState::ReadCmdParam;
    }
    else
    {
        if (_rxBuffer[_rxIndex] == '*') unreadByte('*');
        flushGarbage();
        return ModemParseResult::Garbage;
    }
    return ModemParseResult::Parsing;
}

ModemParseResult MLR_Modem::m_HandleReadCmdParam()
{
    _rxBuffer[_rxIndex] = readByte();
    if ((_rxBuffer[1] == 'D') && (_rxBuffer[2] == 'R') && (_rxBuffer[3] == '='))
    {
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
        if (_rxBuffer[_rxIndex] == '*') unreadByte('*');
        flushGarbage();
        return ModemParseResult::Garbage;
    }
    return ModemParseResult::Parsing;
}

ModemParseResult MLR_Modem::m_HandleRadioDrSize()
{
    _rxBuffer[_rxIndex] = readByte();
    ++_rxIndex;
    if (_rxIndex < 6) return ModemParseResult::Parsing;

    if (isxdigit(_rxBuffer[4]) && isxdigit(_rxBuffer[5]))
    {
        m_drMessagePresent = false;
        uint32_t msgLen = 0;
        parseHex(&_rxBuffer[4], 2, &msgLen);
        m_drMessageLen = (uint8_t)msgLen;
        _rxIndex = 0;
        m_parserState = MLR_ModemParserState::RadioDrPayload;
    }
    else
    {
        flushGarbage();
        return ModemParseResult::Garbage;
    }
    return ModemParseResult::Parsing;
}

ModemParseResult MLR_Modem::m_HandleRadioDrPayload()
{
    m_drMessage[_rxIndex] = readByte();
    ++_rxIndex;
    if ((m_drMessageLen + 2 - _rxIndex) == 0)
    {
        if ((m_drMessage[_rxIndex - 2] == '\r') && m_drMessage[_rxIndex - 1] == '\n')
        {
            m_drMessage[_rxIndex - 2] = 0;
            // Note: We don't reset _rxIndex here so it reflects payload length + CRLF for debug prints in Base class.
            // It will be reset to 0 in the next m_HandleReadStart().
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
    return ModemParseResult::Parsing;
}

ModemParseResult MLR_Modem::m_HandleReadCmdUntilCR()
{
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
        if (_rxBuffer[_rxIndex] == '*') unreadByte('*');
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
    return ModemParseResult::Parsing;
}

ModemParseResult MLR_Modem::m_HandleReadCmdUntilLF()
{
    _rxBuffer[_rxIndex] = readByte();
    if (_rxBuffer[_rxIndex] == '\n')
    {
        --_rxIndex;
        _rxBuffer[_rxIndex] = 0;
        m_parserState = MLR_ModemParserState::Start;

        // Information Response (*IR=) is treated as unsolicited to trigger onRxDataReceived
        if (_rxBuffer[1] == 'I' && _rxBuffer[2] == 'R')
        {
            return ModemParseResult::FinishedDrResponse;
        }
        return ModemParseResult::FinishedCmdResponse;
    }
    else
    {
        if (_rxBuffer[_rxIndex] == '*') unreadByte('*');
        flushGarbage();
        return ModemParseResult::Garbage;
    }
    return ModemParseResult::Parsing;
}

// --- Internal Logic ---

void MLR_Modem::dispatchAsyncEvent(ModemError error, MLR_Modem_Response responseType, int32_t value, const uint8_t *pPayload, uint16_t len)
{
    if (m_pCallback)
    {
        m_pCallback(MLR_Modem_Event(error, responseType, value, pPayload, len));
    }
}

ModemError MLR_Modem::m_HandleMessageHexByte(uint8_t *pValue, uint32_t responseLen, const char *responsePrefix)
{
    uint32_t val;
    ModemError err = parseResponseHex(_rxBuffer, _rxIndex, responsePrefix, (uint8_t)(responseLen - strlen(responsePrefix)), &val);
    if (err == ModemError::Ok && pValue) *pValue = (uint8_t)val;
    return err;
}

ModemError MLR_Modem::m_HandleMessageHexWord(uint16_t *pValue, uint32_t responseLen, const char *responsePrefix)
{
    uint32_t val;
    ModemError err = parseResponseHex(_rxBuffer, _rxIndex, responsePrefix, (uint8_t)(responseLen - strlen(responsePrefix)), &val);
    if (err == ModemError::Ok && pValue) *pValue = (uint16_t)val;
    return err;
}

ModemError MLR_Modem::m_HandleMessage_RS(int16_t *pRssi)
{
    int32_t val;
    ModemError err = parseResponseDec(_rxBuffer, _rxIndex, MLR_GET_RSSI_LAST_RX_RESPONSE_PREFIX, "dBm", 3, &val);
    if (err == ModemError::Ok && pRssi) *pRssi = (int16_t)val;
    return err;
}

ModemError MLR_Modem::m_HandleMessage_RA(int16_t *pRssi)
{
    int32_t val;
    ModemError err = parseResponseDec(_rxBuffer, _rxIndex, MLR_GET_RSSI_CURRENT_CHANNEL_RESPONSE_PREFIX, "dBm", 3, &val);
    if (err == ModemError::Ok && pRssi) *pRssi = (int16_t)val;
    return err;
}

ModemError MLR_Modem::m_HandleMessage_SN(uint32_t *pSerialNumber)
{
    if (_rxIndex < MLR_GET_SERIAL_NUMBER_RESPONSE_LEN) return ModemError::Fail;
    if (strncmp(MLR_GET_SERIAL_NUMBER_RESPONSE_PREFIX, (char *)_rxBuffer, strlen(MLR_GET_SERIAL_NUMBER_RESPONSE_PREFIX)) != 0) return ModemError::Fail;

    uint8_t startIdx = 4;
    uint8_t hexLen = 8;
    if (!isdigit(_rxBuffer[4])) { startIdx = 5; hexLen = 7; }

    uint32_t sn;
    if (parseDec(&_rxBuffer[startIdx], hexLen, &sn))
    {
        if (pSerialNumber) *pSerialNumber = sn;
        return ModemError::Ok;
    }
    return ModemError::Fail;
}

ModemError MLR_Modem::m_HandleMessage_IZ()
{
    if (_rxIndex == MLR_SET_IZ_RESPONSE_LEN_OK &&
        strncmp(MLR_SET_IZ_RESPONSE_PREFIX_OK, (char *)_rxBuffer, MLR_SET_IZ_RESPONSE_LEN_OK) == 0)
    {
        return ModemError::Ok;
    }
    return ModemError::Fail;
}
