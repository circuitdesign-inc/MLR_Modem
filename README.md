# MLR_Modem

サーキットデザイン社製MLR-429無線モデムを制御するためのArduinoライブラリです。

**本ライブラリは、動作保証、並びに個別のサポートは行っておりません。 自己責任の上、各ソフトウェアライセンスを尊重のうえご利用ください。<br>ご質問やバグのご報告はGitHubのIssueをご利用ください。**

## 概要

![MLR-429](https://www.circuitdesign.jp/wp/wp-content/uploads/products/mlr-429-front.png)
このライブラリは、Arduinoでサーキットデザイン社製MLR-429無線モデムを制御するためのインターフェースを提供します。
シリアルコマンドインターフェースを介して、データの送受信やモデムの設定を簡単に行うことができます。

API等は以下ドキュメントをご参照ください。<br>
https://circuitdesign-inc.github.io/MLR_Modem/

## 対応ハードウェア

*   Circuit Design MLR-429

## インストール

1.  GitHubリポジトリから最新のリリースをダウンロードします。
2.  Arduino IDEで、`スケッチ` > `ライブラリをインクルード` > `.ZIP形式のライブラリをインストール...` に移動します。
3.  ダウンロードしたZIPファイルを選択します。

## 基本的な使い方

### ハードウェアのセットアップ

MLRモデムをArduinoなどのマイクロコントローラと接続するための基本的なセットアップ方法です。
詳細は各モデムのデータシートを必ずご確認ください。

#### Arduinoとの接続

以下の4つの端子を接続する必要があります。

| MLR端子 | 接続先 (Arduino) | 説明 |
| :---: | :---: | :--- |
| `VCC` | 3.3V ~ 5.5V | 安定化された電源を接続します。Arduinoの`5V`または`3.3V`ピンに接続します。 |
| `GND` | `GND` | グランドを接続します。 |
| `TXD` | Arduinoの `RX` | MLRモデムの送信(TXD)をArduinoの受信(RX)に接続します。コントロール電圧はVCCに依存します。 |
| `RXD` | Arduinoの `TX` | MLRモデムの受信(RXD)をArduinoの送信(TX)に接続します。コントロール電圧はVCCに依存します。|

**【重要】**
*   **ロジックレベル**: MLRモデムのロジックレベルは3.3Vです。ロジックレベルが5Vのマイコンを使用する際は、レベル変換IC等を使用する必要があります。
*   **未使用端子**: 使用しない端子はデータシートの指示に従い、オープン（未接続）にしてください。


### プログラム
以下は、MLRモデムを初期化し、データを受信し、5秒ごとにメッセージを送信する基本的なサンプルコードです。
詳細なサンプルはexsamplesフォルダをご確認ください。

```cpp
#include <MLR_Modem.h>

// MLRモデムのインスタンスを作成
MLR_Modem modem;

void setup() {
  // PCとの通信用シリアルを開始
  Serial.begin(115200);
  while (!Serial);
  Serial.println("MLR Modem Basic Example");

  // MLRモデムとの通信用シリアル(Serial1など)を開始
  // デフォルトのボーレートは19200bps
  Serial1.begin(MLR_DEFAULT_BAUDRATE);

  // モデムを初期化
  // 第1引数: 通信に使うStreamオブジェクト
  MLR_Modem_Error err = modem.begin(Serial1);

  if (err != MLR_Modem_Error::Ok) {
    Serial.println("MLRモデムの初期化に失敗しました。");
    while (1);
  }
  Serial.println("MLRモデムの初期化が完了しました。");

  // モデムの各種設定 (任意)
  // 第2引数(saveValue)をtrueにすると設定が不揮発性メモリに保存されます
  modem.SetMode(MLR_ModemMode::LoRaCmd, false);                  // LoRaモードに設定
  modem.SetSpreadFactor(MLR_ModemSpreadFactor::Chips512, false); // 拡散率をSF9相当に設定
  modem.SetChannel(0x07, false);                                 // チャンネルを7に設定
  modem.SetGroupID(0x01, false);                                 // グループIDを1に設定
  modem.SetDestinationID(0x00, false);                           // 目的局IDを0(ブロードキャスト)に設定
  modem.SetEquipmentID(0x01, false);                             // 自局IDを1に設定
}

void loop() {
  // この関数をループ内で常に呼び出す必要があります。
  // 受信データの解析やコールバックの呼び出しを行います。
  modem.Work();

  // --- ポーリングによる受信処理 ---
  if (modem.HasPacket())
  {
      const uint8_t *pPayload;
      uint8_t len;
      int16_t rssi;

      // 受信パケットの取得
      if (modem.GetPacket(&pPayload, &len, &rssi) == MLR_Modem_Error::Ok)
      {
          Serial.print("パケット受信 (RSSI: ");
          Serial.print(rssi);
          Serial.print(" dBm, ");
          Serial.print(len);
          Serial.print(" バイト): ");

          // 受信データを文字列として表示
          Serial.write(pPayload, len);
          Serial.println();
      }
      // 重要: データ処理が終わったら必ずパケットを破棄してください
      modem.DeletePacket();
  }


  // 10秒ごとにメッセージを送信する例
  static uint32_t lastSendTime = 0;
  if (millis() - lastSendTime > 10000) {
    lastSendTime = millis();
    
    const char* message = "Hello, MLR-Modem!";
    Serial.printf("'%s' を送信します...\n", message);
    
    // データ送信 (目的局IDはSetDestinationID()で設定済み)
    modem.TransmitData((const uint8_t*)message, strlen(message));
  }
}
```

## デバッグ
platformioを使用している場合、ライブラリのデバッグ出力を有効にすることができます。

デバッグ機能を有効にするには、platformio.iniに以下のビルドフラグ追加してください:
```
build_flags = -D ENABLE_MLR_MODEM_DEBUG
```

また、デバッグの出力先を以下の関数で指定する必要があります。
```cpp
// デバッグの出力先を設定
modem.setDebugStream(&Serial);
```

## License

このライブラリはMITライセンスの下でリリースされています。
