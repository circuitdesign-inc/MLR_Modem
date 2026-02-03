/**
 * @file lora_tx_rx_example.ino
 * @brief MLR-ModemライブラリのLoRaモード送受信サンプル
 * @copyright Copyright (c) 2026 CircuitDesign,Inc.
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 *
 * @details
 * このサンプルプログラムは、MLR-ModemライブラリのLoRaモードに特化した
 * 設定方法と、エラー処理を含むデータ送受信を行う方法を示します。
 *
 * 動作:
 * 1. setup()関数でモデムを初期化し、コールバック関数を登録します。
 * 2. SetMode() を呼び出して、無線通信モードを明示的に **LoRaモード** に設定します (デフォルトもLoRaモードです)。
 * 3. SetSpreadFactor() を呼び出して、LoRaの **chip数 (拡散率)** を設定します。
 * 4. チャンネル、各種IDを設定します。
 * 5. loop()関数内で、10秒ごとにカウンター付きのメッセージを送信します。
 * 6. 送信時、LBT/相関センスで失敗した場合はリトライ処理を行います。
 * 7. データを受信するとコールバックが呼ばれ、loop()関数側で受信データを処理します。
 *
 * このサンプルを実行するには、Arduino互換ボードのSerial1に
 * MLRモデムが接続されている必要があります (デフォルトボーレート 19200 bps)。
 */
#include <MLR_Modem.h>

// MLRモデムのデフォルトボーレート
#define MLR_DEFAULT_BAUDRATE 19200
// MLRモデムの最大ペイロード長
#define MLR_MAX_PAYLOAD_LEN 255

MLR_Modem modem;

// --- 受信データ共有用の変数 ---
volatile bool g_packetReceived = false;
uint8_t g_receivedPayload[MLR_MAX_PAYLOAD_LEN];
volatile uint16_t g_receivedLen = 0;

/**
 * @brief モデムからの非同期イベントを処理するコールバック関数
 *
 * @param error エラーコード (MLR_Modem_Error)
 * @param responseType 応答の種類 (MLR_Modem_Response)
 * @param value 応答に含まれる数値
 * @param pPayload 受信したデータのペイロードへのポインタ (DataReceived時のみ有効)
 * @param len ペイロードの長さ (バイト単位) (DataReceived時のみ有効)
 */
void modemCallback(MLR_Modem_Error error, MLR_Modem_Response responseType, int32_t value, const uint8_t *pPayload, uint16_t len)
{
  // データ受信イベントかを確認
  if (responseType == MLR_Modem_Response::DataReceived)
  {
    if (error == MLR_Modem_Error::Ok)
    {
      Serial.println("\n[コールバック] データを受信しました！");
      // loop()で処理するために、受信データをグローバル変数にコピーしてフラグを立てる
      if (len > 0 && len <= sizeof(g_receivedPayload))
      {
        memcpy(g_receivedPayload, pPayload, len);
        g_receivedLen = len;
        g_packetReceived = true;
      }
    }
    else
    {
      Serial.printf("[コールバック] データ受信エラー: %d\n", (int)error);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("\n--- MLRモデム LoRa送受信サンプル ---");

  // モデム用のシリアルポートを初期化 (MLRモデムのデフォルトは19200)
  Serial1.begin(MLR_DEFAULT_BAUDRATE);

  // モデムドライバを初期化
  MLR_Modem_Error err = modem.begin(Serial1, modemCallback);

  if (err != MLR_Modem_Error::Ok)
  {
    Serial.println("MLRモデムの初期化に失敗しました。接続とボーレートを確認してください。");
    while (true)
      ;
  }
  Serial.println("MLRモデムが初期化されました。");

  // 1. 無線通信モードを LoRa モード (0x03) に設定します。
  // (MLR-429のデフォルトは LoRa モードです)
  err = modem.SetMode(MLR_ModemMode::LoRaCmd, false);
  if (err != MLR_Modem_Error::Ok)
  {
    Serial.println("LoRa モードの設定に失敗しました。");
  }
  else
  {
    Serial.println("無線通信モードを LoRa に設定しました。");
  }

  // 2. LoRa変調方式の chip数 (拡散率) を設定します。
  // '00' (128 chip) から '05' (4096 chip) まであります。
  // chip数が多いほど長距離になりますが、通信時間が長くなります。
  // 例: '03' (1024 chip / SF10) に設定
  MLR_ModemSpreadFactor sf = MLR_ModemSpreadFactor::Chips1024;
  err = modem.SetSpreadFactor(sf, false);
  if (err != MLR_Modem_Error::Ok)
  {
    Serial.println("LoRa Spreading Factor (chip数) の設定に失敗しました。");
  }
  else
  {
    Serial.println("LoRa Spreading Factor を 1024 chip (SF10) に設定しました。");
  }

  // --- 共通設定 ---

  // チャンネルを設定 (例: チャンネル7)
  uint8_t channel = 0x07;
  if (modem.SetChannel(channel, false) != MLR_Modem_Error::Ok)
  {
    Serial.println("チャンネルの設定に失敗しました。");
  }
  Serial.print("チャンネルを ");
  Serial.print(channel, HEX);
  Serial.println(" に設定しました。");

  // グループIDを設定
  uint8_t groupID = 0x01;
  if (modem.SetGroupID(groupID, false) != MLR_Modem_Error::Ok)
  {
    Serial.println("グループIDの設定に失敗しました。");
  }
  Serial.print("グループIDを ");
  Serial.print(groupID, HEX);
  Serial.println(" に設定しました。");

  // 目的局IDを設定 (0x00はブロードキャスト)
  uint8_t destID = 0x00;
  if (modem.SetDestinationID(destID, false) != MLR_Modem_Error::Ok)
  {
    Serial.println("目的局IDの設定に失敗しました。");
  }
  Serial.print("目的局IDを ");
  Serial.print(destID, HEX);
  Serial.println(" に設定しました。");

  // 自機IDを設定
  uint8_t equipID = 0x02;
  if (modem.SetEquipmentID(equipID, false) != MLR_Modem_Error::Ok)
  {
    Serial.println("自機IDの設定に失敗しました。");
  }
  Serial.print("自機IDを ");
  Serial.print(equipID, HEX);
  Serial.println(" に設定しました。");
}

void loop()
{
  // モデムの内部処理(シリアルデータの解析やコールバックの呼び出し)のために、
  // Work()メソッドを常に呼び出す必要があります。
  modem.Work();

  // --- データ受信処理 ---
  if (g_packetReceived)
  {
    Serial.println("\n--- 受信パケット処理 ---");

    // 最後に受信したRSSIを要求する
    int16_t lastRssi = 0;
    if (modem.GetRssiLastRx(&lastRssi) == MLR_Modem_Error::Ok)
    {
      Serial.printf("RSSI: %d dBm\n", lastRssi);
    }
    else
    {
      Serial.println("RSSI: 取得失敗");
    }

    Serial.printf("長さ: %u バイト\n", g_receivedLen);
    Serial.print("ペイロード (ASCII): ");
    Serial.write(g_receivedPayload, g_receivedLen);
    Serial.println("\n----------------------------------");

    // 処理が終わったらフラグをリセット
    g_packetReceived = false;
  }

  // --- データ送信処理 (10秒ごと) ---
  static uint32_t lastSendTime = 0;
  static uint8_t counter = 0;
  if (millis() - lastSendTime > 10000)
  {
    lastSendTime = millis();

    char message[32];
    snprintf(message, sizeof(message), "Hello LoRa! %u", counter++);

    Serial.printf("\n--- パケット送信: \"%s\" ---\n", message);

    const int MAX_RETRIES = 3;
    for (int i = 0; i < MAX_RETRIES; i++)
    {
      MLR_Modem_Error err = modem.TransmitData((const uint8_t *)message, strlen(message));

      if (err == MLR_Modem_Error::Ok)
      {
        Serial.println("TransmitDataコマンドの受付・送信完了しました。");
        // LoRaモードの場合、TransmitData関数は送信完了レスポンス(*IR=03)が返るまで待機する同期処理となっています。
        break;
      }
      else if (err == MLR_Modem_Error::FailLbt)
      {
        // FailLbtは、キャリアセンス(*IR=01) または LoRa相関センス(*IR=02) の失敗を示します。
        Serial.printf("送信失敗 (LBT/相関センス)。チャンネルがビジーです。リトライします... (%d/%d)\n", i + 1, MAX_RETRIES);
        delay(100); // 少し待ってからリトライ
      }
      else
      {
        Serial.printf("送信失敗。エラー: %d\n", (int)err);
        break; // LBT以外のエラーではリトライしない
      }

      if (i == MAX_RETRIES - 1)
      {
        Serial.println("リトライ回数の上限に達したため、送信を中止しました。");
      }
    }
  }
}