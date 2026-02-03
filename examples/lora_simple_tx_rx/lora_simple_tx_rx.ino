/**
 * @file lora_simple_tx_rx.ino
 * @brief MLR-ModemライブラリのLoRaモードにおける基本的な送受信サンプル
 * @copyright Copyright (c) 2026 CircuitDesign,Inc.
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 *
 * @details
 * このサンプルプログラムは、MLR-Modemライブラリを使用して、モデムの初期化、
 * LoRaモード設定、拡散率(SF)の設定、各種IDとチャンネルの設定、
 * そして定期的なデータ送信とポーリングによるデータ受信を行う基本的な方法を示します。
 *
 * 動作:
 * 1. setup()関数でモデムを初期化し、LoRaモード、SF、チャンネル、グループID、目的局ID、自機IDを設定します。
 * 2. loop()関数内で、10秒ごとに "Hello LoRa!" というメッセージを送信します。
 * 3. 同時に、modem.Work()を呼び出した後、modem.HasPacket()で受信データの有無をポーリングします。
 * 4. データがある場合はmodem.GetPacket()で内容を取得しシリアルモニタに表示します。
 * (処理後、modem.DeletePacket()でバッファを解放します)
 *
 * このサンプルを実行するには、Arduino互換ボードのSerial1に
 * MLRモデムが接続されている必要があります。
 */
#include <MLR_Modem.h>

MLR_Modem modem;

void setup()
{
    Serial.begin(115200);

    // シリアルポートが開くまで待機
    while (!Serial)
        ;

    // モデム用のシリアルポートを初期化 (MLRモデムのデフォルトは19200)
    Serial1.begin(MLR_DEFAULT_BAUDRATE);

    // モデムドライバを初期化
    MLR_Modem_Error err = modem.begin(Serial1);

    if (err != MLR_Modem_Error::Ok)
    {
        Serial.println("MLRモデムの初期化に失敗しました。接続とボーレートを確認してください。");
        while (true)
            ;
    }
    Serial.println("MLRモデムが初期化されました。");

    // 動作モード及び、各種IDとチャンネルを設定します。
    // 第2引数(saveValue)をtrueにすると、設定がモデムの不揮発性メモリに保存されます。
    // このサンプルでは、電源を入れ直すと元に戻るようにfalseに設定しています。

    // 無線通信モードをLoRaモードに設定
    modem.SetMode(MLR_ModemMode::LoRaCmd, false);

    // 拡散率(Spreading Factor)を設定 (例: SF9 = 512 chips)
    // LoRaモード特有の設定です。通信距離と通信速度のトレードオフになります。
    modem.SetSpreadFactor(MLR_ModemSpreadFactor::Chips512, false);

    // チャンネルを設定 (例: チャンネル7)
    uint8_t channel = 0x07;
    modem.SetChannel(channel, false);

    // グループIDを設定
    uint8_t groupID = 0x01;
    modem.SetGroupID(groupID, false);

    // 目的局IDを設定 (0x00はブロードキャスト)
    uint8_t destID = 0x00;
    modem.SetDestinationID(destID, false);

    // 自機IDを設定
    uint8_t equipID = 0x02;
    modem.SetEquipmentID(equipID, false);
}

void loop()
{
    // モデムの内部処理(受信データの解析など)を実行するために、Work()を常に呼び出します。
    modem.Work();

    // --- ポーリングによる受信処理 ---
    if (modem.HasPacket())
    {
        const uint8_t *pPayload;
        uint8_t len;

        // 受信パケットの取得
        if (modem.GetPacket(&pPayload, &len) == MLR_Modem_Error::Ok)
        {
            Serial.print("パケット受信 (");
            Serial.print(len);
            Serial.print(" バイト): ");

            // 受信データを文字列として表示
            for (int i = 0; i < len; i++)
            {
                Serial.write(pPayload[i]);
            }
            Serial.println();
        }

        // 重要: データ処理が終わったら必ずパケットを破棄して、次の受信ができるようにしてください。
        modem.DeletePacket();
    }

    // --- 送信処理 (例: 10秒ごとに "Hello LoRa!" を送信) ---
    static uint32_t lastSendTime = 0;
    if (millis() - lastSendTime > 10000)
    {
        lastSendTime = millis();
        const char *message = "Hello LoRa!";
        Serial.print("メッセージを送信中: ");
        Serial.println(message);
        modem.TransmitData((const uint8_t *)message, strlen(message));
    }
}