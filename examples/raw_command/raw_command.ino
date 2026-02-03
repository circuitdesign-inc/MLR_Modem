/**
 * @file raw_command.ino
 * @brief MLR-ModemライブラリのRAWコマンド送受信サンプル
 * @copyright Copyright (c) 2026 CircuitDesign,Inc.
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 *
 * @details
 * このサンプルプログラムは、MLR-Modemライブラリの低レベルAPIである
 * SendRawCommand() (同期的) と
 * SendRawCommandAsync() (非同期的)
 * の使用方法を示します。
 *
 * 動作:
 * 1. setup()関数でモデムを初期化し、コールバック関数を登録します。
 * 2. setup()内で、同同期待機版の SendRawCommand() を使用して、
 * ファームウェアバージョン取得コマンド ("@FV\r\n") [cite: 487-491] を送信し、
 * その場で応答を表示します。
 * 3. loop()関数内で、10秒ごとに非同期版の SendRawCommandAsync() を使用して、
 * 現在のRSSI取得コマンド ("@RA\r\n") [cite: 460-465] を送信します。
 * 4. コールバック関数 (modemCallback) は、"@RA" に対する応答
 * (MLR_Modem_Response::GenericResponse) を検知し、受信した生データを表示します。
 * 5. 通常のデータ受信 (*DR=...) も並行して処理されます。
 */
#include <MLR_Modem.h>

// MLRモデムのデフォルトボーレート
#define MLR_DEFAULT_BAUDRATE 19200
// MLRモデムの最大ペイロード長
#define MLR_MAX_PAYLOAD_LEN 255

MLR_Modem modem;

/**
 * @brief モデムからの非同期イベントを処理するコールバック関数
 *
 * @param error エラーコード
 * @param responseType 応答の種類
 * @param value 応答に含まれる数値
 * @param pPayload 受信したデータのペイロードへのポインタ
 * @param len ペイロードの長さ (バイト単位)
 */
void modemCallback(MLR_Modem_Error error, MLR_Modem_Response responseType, int32_t value, const uint8_t *pPayload, uint16_t len)
{
  if (error != MLR_Modem_Error::Ok)
  {
    Serial.printf("[コールバック] エラー発生 Type: %d, Error: %d\n", (int)responseType, (int)error);
    return;
  }

  // --- 非同期RAWコマンドの応答処理 ---
  if (responseType == MLR_Modem_Response::GenericResponse)
  {
    Serial.print("[非同期RAW応答] 受信 (");
    Serial.print(len);
    Serial.print(" バイト): ");

    // pPayloadは応答文字列 (終端ヌル文字なし)
    for (int i = 0; i < len; i++)
    {
      Serial.write(pPayload[i]);
    }
    Serial.println();
  }
  // --- 通常のデータ受信処理 ---
  else if (responseType == MLR_Modem_Response::DataReceived)
  {
    Serial.print("\n[データ受信] (");
    Serial.print(len);
    Serial.print(" バイト): ");

    for (int i = 0; i < len; i++)
    {
      Serial.write(pPayload[i]);
    }
    Serial.println();

    // (オプション) 必要ならRSSIも取得
    int16_t rssi;
    if (modem.GetRssiLastRx(&rssi) == MLR_Modem_Error::Ok)
    {
      Serial.printf("  (RSSI: %d dBm)\n", rssi);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("\n--- MLRモデム RAWコマンド送受信サンプル ---");

  // モデム用のシリアルポートを初期化
  Serial1.begin(MLR_DEFAULT_BAUDRATE);

  // モデムドライバを初期化
  MLR_Modem_Error err = modem.begin(Serial1, modemCallback);

  if (err != MLR_Modem_Error::Ok)
  {
    Serial.println("MLRモデムの初期化に失敗しました。");
    while (true)
      ;
  }
  Serial.println("MLRモデムが初期化されました。");

  // --- 1. 同期RAWコマンドのデモ (@FV) ---
  Serial.println("\n--- 同期RAWコマンド (@FV) 送信中... ---");

  // 応答を格納するバッファ
  char responseBuffer[32];

  // コマンドの末尾には "\r\n" が必要です [cite: 237]。
  const char *cmd = "@FV\r\n";

  // 同期コマンドを送信
  err = modem.SendRawCommand(cmd, responseBuffer, sizeof(responseBuffer));

  if (err == MLR_Modem_Error::Ok)
  {
    // 成功時、responseBufferに応答 (*FV=...など) が格納される
    Serial.print("同期応答 受信成功: ");
    Serial.println(responseBuffer);
  }
  else if (err == MLR_Modem_Error::BufferTooSmall)
  {
    Serial.println("同期応答 失敗: 応答バッファが小さすぎます。");
  }
  else if (err == MLR_Modem_Error::Fail)
  {
    Serial.println("同期応答 失敗: タイムアウトまたはパースエラー。");
  }
  else
  {
    Serial.printf("同期応答 失敗: エラー %d\n", (int)err);
  }

  Serial.println("-------------------------------------\n");
}

void loop()
{
  // モデムの内部処理(シリアルデータの解析やコールバックの呼び出し)のために、
  // Work()メソッドを常に呼び出す必要があります。
  modem.Work();

  // --- 2. 非同期RAWコマンドのデモ (@RA) ---
  static uint32_t lastSendTime = 0;
  if (millis() - lastSendTime > 10000) // 10秒ごと
  {
    lastSendTime = millis();

    // コマンドの末尾には "\r\n" が必要です [cite: 237]。
    const char *cmd = "@RA\r\n";

    Serial.print("--- 非同期RAWコマンド (@RA) 送信中... ---\n");

    MLR_Modem_Error err = modem.SendRawCommandAsync(cmd);

    if (err == MLR_Modem_Error::Ok)
    {
      Serial.println("非同期コマンド 送信成功。応答はコールバックで待機します。");
    }
    else if (err == MLR_Modem_Error::Busy)
    {
      Serial.println("非同期コマンド 失敗: モデムがビジーです (他の非同期処理が実行中)。");
    }
    else
    {
      Serial.printf("非同期コマンド 失敗: エラー %d\n", (int)err);
    }
  }
}