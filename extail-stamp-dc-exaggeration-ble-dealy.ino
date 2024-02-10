/*
   logかつdelayで動かすソース
   MultiTaskにextail-stamp-exaggeration-dcを移植してstamp仕様のMultiTaskで動かしたソース
  BLE通信モニタリング機能（stampが送信側）を搭載　LEDのソースはextail-device-stamppico-bleを参照
*******************************************************************************
  Copyright (c) 2021 by M5Stack
                   Equipped with STAMP-PICO sample source code
                           配套  STAMP-PICO 示例源代码
  Visit the website for more information：https://docs.m5stack.com/en/core/stamp_pico
  获取更多资料请访问：https://docs.m5stack.com/zh_CN/core/stamp_pico

  describe: MultiTask.  多线程
  date：2021/9/25
*******************************************************************************
*/
#include <Arduino.h>
#include <NimBLEDevice.h> //BLE
#include <FastLED.h> //LED

#define Button 39
static bool btnState;//Default:1, Pressed:0
static bool btnState_old;

// *--- 9軸センサ BNO055 ---
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // サンプル取得間のdelay

// I2C device address
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

//遅延系
const float delaySec = 1.0; // 遅延秒数（検証対象：(0)0.1,0.2,0.3,0.5,0.75,1.0,1.25,1.5
const int sampleRate = 100; // サンプリングレート（Hz）BNO055_SAMPLERATE_DELAY_MSと（実質）揃える．型の便宜上変数は分けてある．

const int bufferSize = delaySec * sampleRate; // バッファサイズ（delaySec秒分）
double buffer[bufferSize]; // ロールセンサの値を保存するバッファ
int bufferIndex = 0; // バッファの現在のインデックス

double sensVal;
double delayedSensVal;


//BLE 通信test用変数
String sampleText[] = {"GoodMorning", "Hello", "GoodBye", "GoodNight"};
uint8_t FSM = 0;    //Store the number of key presses.  存储按键按下次数
int SensingTarget = 0;

//遅延量をボタンで調整する
float delaySecBtnCnt = 0;

// *--- DC motor ---
#define IN1 32 // モーター1正転信号端子
#define IN2 33 // モーター1逆転信号端子

#define CH_IN1 0  // PWM出力チャンネル設定（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15で周波数、分解能設定が共通）
#define CH_IN2 1  //  ※チャンネル7は液晶バックライトと共用になるため指定禁止
#define FREQ 500  // PWM出力周波数　ここを変えてみても意味はなかった
#define BIT_NUM 8 // PWM出力bit数（今回は12bit指定。分解能は2の12乗で4096）//FREQ500のとき、4でもまわるけど12は回らなかった

int dir = 0; // 回転方向

int duty_max = 255; // 回転スピードに関わるが、delayだけで調整しそう。一旦MAXで一定にしておく。

// 回転スピード
int rotationSpeed = 10;
int rotationSpeed_max = 0;   // 最速
int rotationSpeed_min = 100; // 最遅

// 回転量の計算用
int targetAngle = 0;                 // 目標角度（-45〜45度の範囲）
int rotationQuantity = 0;            // センサ値に応じて動かすステップ数（角度からステップ数に変換）
int rotationQuantity_total = 0;      // のべステップ数（現在地）
int rotationQuantity_total_max = 100; // 振幅の最大値（片側分）

// *--- センサ値関係 ---
// 加速度
double accX = 0.0;
double accX_th_min = 1.5; // 閾値
double accX_th_max = 8;

// 加速度
double accZ = 0.0;
double accZ_th_min = 1.5; // 閾値
double accZ_th_max = 8;


float roll_logBase = 1.0617;//20-50
float accX_logBase = 1.0105;//8-200
float accZ_logBase = 1.0105;

// 姿勢角
double roll;     //-90~90の値を取る ただ普段人間の首はせいぜい-45~45くらいしか傾げないので、設計上は-45~45外は丸める
int roll_roundf; // rollを四捨五入した整数値
int roll_old;    // 1loop前のroll値
int roll_diff;   // 現roll値と1loop前のroll値の差（絶対値）0-180をとる

// diffの閾値 絶対値がこれ以上の場合回転させる roll
int roll_diff_th_min = 3;//3;
int roll_diff_th_max = 20;//20;

// 誇張係数（未使用）
int expand = 1;

bool back = false; // ホームポジションに戻す状態か（まだ仕込めてない）


// ## BLE
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define LOCAL_NAME "Extail_DEVICE"
#define COMPLETE_LOCAL_NAME "Extail_DEVICE_LOCAL_NAME" //接続する時に使用するデバイス名
#define SERVICE_UUID "3c3996e0-4d2c-11ed-bdc3-0242ac120002" //ServiceのUUID
#define CHARACTERISTIC_UUID "3c399a64-4d2c-11ed-bdc3-0242ac120002" //CharacteristicのUUID
#define CHARACTERISTIC_UUID_NOTIFY "3c399c44-4d2c-11ed-bdc3-0242ac120002" //通知を行うためのCharacteristicのUUID

NimBLECharacteristic *pNotifyCharacteristic; //Characteristic
NimBLEServer *pServer = NULL; //NimBLEServer

bool deviceConnected = false;
bool oldDeviceConnected = false;
//uint8_t data_buff[2]; //データ通知用バッファ

// ### 心拍系
//BLE通信で送られてきた値をintに直して保管（初期値は標準的な値）
int receivedValue = 0;
bool flag_volume = false;


// *--- LED ---
// How many leds in your strip?
#define NUM_LEDS 1
#define DATA_PIN 27
#define BRIGHTNESS 2

// Define the array of leds
CRGB leds[NUM_LEDS];

//LED点灯タスクを渡す変数
int ledTask = 0;



// --------------------------
// *--- Multithread tasks ---
// task1：センサ値に応じてステッパーを回す
void task1(void * pvParameters) { //Define the tasks to be executed in thread 1.  定义线程1内要执行的任务
  while (1) { //Keep the thread running.  使线程一直运行

   
      switch (SensingTarget)
      {
      case 0:
        stepRoll();
        break;

      case 1:
        stepAccX();
        break;

      case 2:
        stepAccZ();
        break;

      default:
        //適度に待つ
        delay(100);

      }


    vTaskDelay(5); // ステッパーがセンサ値に応じて回転する1セット分を待つインターバル
    // 0でもうごきはするが挙動が不安定になりやすいので5か10くらいにしておく。50だと流石にカクつく
  }
}

void task2(void * pvParameters) {
  while (1) {


    //このままだと眩しすぎるから光量調整しておきたいな
    switch (ledTask)
    {
      case 1:
        leds[0] = CRGB::Green;//緑
        break;

      case 2:
        leds[0] = CRGB::Blue; //青
        break;

      case 3:
        leds[0] = CRGB::Red;//赤
        break;

      case 4:
        leds[0] = CRGB::White; //白
        break;

      case 5: //これちゃんと点滅するんか？ してないわ
        leds[0] = CRGB::Yellow; //黄
        break;

      default:
        //適度に待つ
        delay(100);
        continue; // スイッチ内で待ち時間がない場合は、次のループに進むようにcontinueを追加
    }

    FastLED.show();
    delay(1000);
    leds[0] = CRGB::Black; //消灯
    FastLED.show();
    ledTask = 0; // 共通処理なので、ここでリセット


  }
}

void task3(void * pvParameters) {
  while (1) {
    btnState = digitalRead(Button);
    if (btnState == 0 && btnState_old == 1) {
      Serial.printf("Button Pressed", btnState);
      ledTask = 5;
      //Serial.println("");


      /*if (deviceConnected) { //接続されていたら
        // 送信する値（仮の値）
        String valueToSend = delaySecBtnCnt;

        // BLE通知を行う
        pNotifyCharacteristic->setValue(valueToSend);
        pNotifyCharacteristic->notify();

        Serial.print("send");
        Serial.println(valueToSend);
        }

        delaySecBtnCnt += 0.1;
        if (delaySecBtnCnt > 1.5) {
        delaySecBtnCnt = 0;
        }*/

      FSM++;

      if (FSM >= 3) {
        FSM = 0;
      }
      Serial.println(FSM);



    }
    delay(10);
    btnState_old = btnState;
  }
}




// *--- BLE ---



//接続、切断などのメソッドは後にsetupメソッドにてコールバックの形でセットするのでひとまとめにしておきます
class ServerCallbacks : public NimBLEServerCallbacks
{
    //接続時
    void onConnect(NimBLEServer *pServer)
    {
      Serial.println("Client connected");
      deviceConnected = true;
    };

    /* //SecurytyRequestをペリフェラルから送るとき
      void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
      Serial.print("Client address: ");
      Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());

      NimBLEDevice::startSecurity(desc->conn_handle);

      pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
      deviceConnected = true;
      };*/

    //切断時
    void onDisconnect(NimBLEServer * pServer)
    {
      Serial.println("Client disconnected - start advertising");
      deviceConnected = false;
      NimBLEDevice::startAdvertising();
    };
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc * desc)
    {
      Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
    };
    // Passのリクエスト
    uint32_t onPassKeyRequest()
    { /**これは、セキュリティのために6桁のランダムな番号を返す必要があります。
         または、ここにあるように、独自の静的パスキーを作成します。
      */
      Serial.println("Server Passkey Request");
      return 123456;
    };
    //確認
    bool onConfirmPIN(uint32_t pass_key)
    { /** パスキーが一致しない場合は、falseを返します。*/
      Serial.print("The passkey YES/NO number: ");
      Serial.println(pass_key);
      return true;
    };
    //認証完了時の処理
    void onAuthenticationComplete(ble_gap_conn_desc * desc)
    { /** 暗号化が成功したかどうかを確認し、成功しなかった場合はクライアントを切断する */
      if (!desc->sec_state.encrypted)
      {
        NimBLEDevice::getServer()->disconnect(desc->conn_handle);
        Serial.println("Encrypt connection failed - disconnecting client");
        return;
      }
      Serial.println("Starting BLE work!");
    };
};

//特性アクションのハンドラクラス
//BLE Rceive　セントラル側からのwriteやnotifyでペリフェラル側から通知などの処理もクラスにしておきます。
class CharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0)
      {
        String rxValue_string = rxValue.c_str();//std::stringはそのままではprintf()で表示できません。表示するには.c_str()で変換します。
        Serial.print("received: ");
        Serial.println(rxValue_string);
        ledTask = 4;
        //pNotifyCharacteristic->setValue(rxValue_string);//これはAtom側でreceive(iOSからAtomへの書き込み)成功したらその値をiOSに送り直すやつ、つまり確認用途であり、必須では無い。
        //pNotifyCharacteristic->notify();


        // メッセージの処理が終了したことを示すフラグ
        boolean noMoreEvent = false;

        // メッセージの処理が終了するまで以下を繰り返す
        while (!noMoreEvent)
        {

          //①受信した文字列の中で「>」を探す
          detectReceivedDataType(noMoreEvent, rxValue, ">");//要件的には要らないけど、これが無いと異常に受信してしまう？？
          detectReceivedDataType(noMoreEvent, rxValue, "*");//【volume用】
        }
      }
    }

    /** 通知または表示が送信される前に呼び出されます、
      必要であれば、送信前にここで値を変更することができます。
    */
    /* void onNotify(NimBLECharacteristic* pCharacteristic) {
       Serial.println("Sending notification to clients");
      };*/
};


void detectReceivedDataType(boolean& noMoreEvent, std::string rxValue, String index_str) {
  //noMoreEventは参照渡しにすることで、関数内での変数値の変更が 呼び出し元の変数に反映されるようにする必要がある。
  //そうしないと呼び出し元の onWrite 関数におけるnoMoreEventの値は更新されないので、無限ループに陥ってしまう
  int from = 0;
  int index = rxValue.find(index_str.c_str(), from);
  Serial.print("detect... "/* + index*/);
  // もし見つからなければ
  if (index < 0)
  {
    // 処理が終了したと判断してフラグをセット
    noMoreEvent = true;
    receivedValue = 0;
    flag_volume = false;
    Serial.println("---end---");
  }
  // もし見つかったら
  else
  {
    // 次に処理する読み取り開始位置を更新
    from = index + 1;
    // index_str 以降の数字文字列を取り出す
    const char* valuePtr = rxValue.c_str() + index + 1;
    receivedValue = atoi(valuePtr);
    flag_volume = true;
    Serial.println("receivedValue: " + String(receivedValue) + ", index: " + index_str); //ここで数字拾うだけだと loopの中で直近の1以上の値を持ち続けてしまう？

    if (index_str == ">") {
      SensingTarget = receivedValue;
    }
  }

}


// BLE loop
void loopBLE()
{
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); //ブルートゥーススタックに準備の機会を与える

    pServer->startAdvertising();//アドバタイズを再開する
    Serial.println("restartAdvertising");

    oldDeviceConnected = deviceConnected;
    Serial.println("DisConnected");
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Connected");
  }
}


void setup() {
  Serial.begin(115200);

  FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.setBrightness(BRIGHTNESS);

  delay(10);
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(50);

  Serial.println("Setup BLE....");

  Serial.println("Starting NimBLE Server");
  NimBLEDevice::init(COMPLETE_LOCAL_NAME); //デバイス名（CompleteLocalName）のセット

  //オプション：送信電力を設定、デフォルトは3db
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);// +9db

  //セキュリティセッティング
  NimBLEDevice::setSecurityAuth(true, true, true);

  NimBLEDevice::setSecurityPasskey(123456); //PassKeyのセット
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); //パラメータでディスプレイ有りに設定
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); //パラメータでInOut無しに設定
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  //RxCharacteristic
  NimBLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, NIMBLE_PROPERTY::WRITE);

  //NotifyCharacteristic
  pNotifyCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_NOTIFY, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN); //Need Enc Authen

  pRxCharacteristic->setCallbacks(new CharacteristicCallbacks()); //RxCharacteristicにコールバックをセット
  pService->start();//Serivice開始
  NimBLEAdvertising *pNimBleAdvertising = NimBLEDevice::getAdvertising();//アドバタイズの設定
  pNimBleAdvertising->addServiceUUID(SERVICE_UUID);//アドバタイズするUUIDのセット
  pNimBleAdvertising->addTxPower();//アドバタイズにTxPowerセット

  NimBLEAdvertisementData advertisementData;//アドバタイズデータ作成
  advertisementData.setName(COMPLETE_LOCAL_NAME);//アドバタイズにCompleteLoacaNameセット
  advertisementData.setManufacturerData("NORA");//アドバタイズのManufactureSpecificにデータセット
  pNimBleAdvertising->setScanResponse(true);//ScanResponseを行う
  pNimBleAdvertising->setScanResponseData(advertisementData);//ScanResponseにアドバタイズデータセット
  pNimBleAdvertising->start();//アドバタイズ開始
  Serial.println("first startAdvertising");

  //Button
  pinMode(Button, INPUT);

  // *--- DC motor ---
  pinMode(IN1, OUTPUT);             // PWM出力端子（INT1：正転用）を出力設定
  pinMode(IN2, OUTPUT);             // PWM出力端子（INT2：逆転用）を出力設定
  ledcSetup(CH_IN1, FREQ, BIT_NUM); // PWM出力設定（チャンネル, 周波数, bit数）
  ledcSetup(CH_IN2, FREQ, BIT_NUM);
  ledcAttachPin(IN1, CH_IN1); // PWMチャンネルを端子に割り当て（端子番号, チャンネル）
  ledcAttachPin(IN2, CH_IN2);


  // Creat Task1.  创建线程1
  xTaskCreatePinnedToCore(
    task1,     // タスクを実行する関数
    "task1",   // スレッド名
    4096,      // バイト数で指定されるタスク・スタックのサイズ
    NULL,      // 作成されるタスクのパラメータとして使用されるポインタ。
    1,         // タスクの優先順位。
    NULL,      // タスクハンドラ。
    0);        // タスクを実行するコア。

  // Task 2
  xTaskCreatePinnedToCore(
    task2,
    "task2",
    4096,
    NULL,
    2,
    NULL,
    0);

  // Task 3
  xTaskCreatePinnedToCore(
    task3,
    "task3",
    4096,
    NULL,
    3,
    NULL,
    0);




  Wire.begin(21, 22);//pico
  while (!Serial)
    delay(50); // wait for serial port to open!

  // Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  printEvent(&orientationData, receivedValue);
  Serial.print("roll: ");
  Serial.println(roll);
  roll_old = roundf(roll);
  delay(1000);

  Serial.println("Ready.");



}



void loop() {

  loopBLE();


 if (SensingTarget == 0) {

     bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
     sensVal = printEvent(&orientationData, receivedValue);

    } else {

  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  sensVal =  printEvent(&linearAccelData, receivedValue);
  }


  // バッファにセンサの値を保存
  buffer[bufferIndex] = sensVal;

  // インデックスを更新し、バッファの最後まで達したら最初に戻す
  bufferIndex = (bufferIndex + 1) % bufferSize;

  // delaySec秒遅れて値を参照する関数を呼び出す
  delayedSensVal = getDelayedValue(delaySec);
  // String str = "now:" + String(sensVal) + "," + "delayed:" + String(delayedSensVal);
  // Serial.println(str);

  // String str = "x:" + String(accX) + " target:" + String(rotationQuantity) + " rotationQuantity_total:" + String(rotationQuantity_total);
  // Serial.println(str);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}





// --------------------------
// *--- Functions ---


float logN(float x, float n) {
  //底nのlogの結果を得る
  // log(x) / log(n) を計算
  return log(x) / log(n);
}


double getDelayedValue(float seconds) {

  // delaySec秒数分遅れた位置の値を取得
  int delayedIndex = (bufferIndex - int(seconds * sampleRate) + bufferSize) % bufferSize;
  return buffer[delayedIndex];
}



// *--- Stepper BaCsics ---
void turn_CW(int rotationQuantity, int rotationSpeed) // 時計回り
{
  ledcWrite(CH_IN1, rotationSpeed);
  ledcWrite(CH_IN2, 0);

  delay(rotationQuantity);
  rotationQuantity_total -= rotationQuantity;

  turn_brake();
}

void turn_CCW(int rotationQuantity, int rotationSpeed) // 反時計回り
{

  ledcWrite(CH_IN1, 0);
  ledcWrite(CH_IN2, rotationSpeed);

  delay(rotationQuantity);
  rotationQuantity_total += rotationQuantity;

  turn_brake();
}

void turn_standby()
{
  ledcWrite(CH_IN1, 0);
  ledcWrite(CH_IN2, 0);
}

void turn_brake()
{
  ledcWrite(CH_IN1, duty_max);
  ledcWrite(CH_IN2, duty_max);
}



// *--- tools ---
// 回転数をシリアルモニタに出力
void printSteps(int dir, int roundf_, int old, int diff)
{
  if (rotationQuantity != 0)
  {
    Serial.print(", roundf: ");
    Serial.print(roundf_);
    Serial.print(", old: ");
    Serial.print(old);
    Serial.print(", diff: ");
    if (dir == -1)
    {
      Serial.print("-");
    }
    Serial.print(diff);

    Serial.print(", rQuantity: ");
    if (dir == -1)
    {
      Serial.print("-");
    }
    Serial.print(rotationQuantity);

    /*Serial.print(", rSpeed: ");
      Serial.print(rotationSpeed);*/

    Serial.print(", total: ");
    Serial.print(rotationQuantity_total);

    /* Serial.print(", back: ");
      Serial.println(back);*/
    Serial.println();
  }
}

void printSteps_acc(int dir, double accX_absolute, int component)
{
  if (rotationQuantity != 0)
  {
    String str;

    if (component == 0) {
      str += "accX:";
    } else {
      str += "accZ:";
    }
    if (dir == -1)
    {
      str += "-";
    }

    str += String(accX_absolute) + " target: " + String(rotationQuantity) + " rQuantity_total:" + String(rotationQuantity_total) /*+ " rSpeed:" + String(rotationSpeed)*/;

    Serial.println(str);
  }
}







// 行った分戻る だと 正逆で回転量が違うとタチ悪いので，一旦使わず．ソースも未調整
//  回りっぱなしの軸をじわじわ中央へ戻す
void stepBack()
{
  // ステッパーを回すセンサ値になってるターンから最低1000msおいてから戻す
  //  じわじわ戻すために、戻す感覚は時間を空けたいが じわじわ戻すほど大変で最悪っぽいのでほどほどにする
  int diff = rotationQuantity_total - 0;
  if (rotationQuantity_total > 0)
  {
    Serial.print("back+ IN ");

    if (back == true)
    {
      Serial.print("差+");
      Serial.print(diff);

      turn_CW(diff, duty_max);

      Serial.print(" 戻した- total ");
      Serial.println(rotationQuantity_total);
    }
    else
    {
      delay(1000); // total==0でもここには入っているっぽいのはなぜだろう??気のせい
      Serial.print(" delayedOnly");
    }
    back = true;
    Serial.println(" Inside if block");  // 追加した行
  }
  else if (rotationQuantity_total < 0)
  {
    Serial.print("back- IN ");

    if (back == true)
    {
      Serial.print("差");
      Serial.print(diff);

      turn_CCW(diff, duty_max);

      Serial.print(" 戻した+ total ");
      Serial.println(rotationQuantity_total);
    }
    else
    {
      delay(1000);
      Serial.println(" delayedOnly");
    }
    back = true;
    Serial.println(" Inside else block");  // 追加した行
  }
  Serial.println(" Outside if-else block");  // 追加した行
}



// *--- センサー値 ---

// センサー値と現状に応じてステッパーを回す
void rotateWithSensorValue(int dir, int &rotationQuantity, int rotationSpeed)
{

  if (dir == 1) // CCW
  {
    // 振幅が既に最大値に至っている場合は回転しない
    if (rotationQuantity_total < rotationQuantity_total_max)
    {
      // ex 37<40
      // 回転量をそのまま足すと最大値を突破する場合は、最大値に収まる値を足す
      if (rotationQuantity_total + rotationQuantity > rotationQuantity_total_max)
      {
        // rotationQuantity_total + rotationQuantity <= rotationQuantity_total_max にしておきたいから
        // ex）37 + 5 > 42 これはオーバーなので補正する
        rotationQuantity = rotationQuantity_total_max - rotationQuantity_total;
        // 3 = 40 - 37
        // Serial.print("Corrected ");
      }

      turn_CCW(rotationQuantity, rotationSpeed);
      back = false;
    }
    else
    {
      //rotationQuantity = 0;

    }
  }
  else if (dir == -1) // CW
  {
    // 振幅が既に最小値に至っている場合は回転しない
    if (rotationQuantity_total > 0 - rotationQuantity_total_max)
    {
      // ex -37 > -40
      // 回転量をそのまま足すと最小値を突破する場合は、最小値に収まる値を足す
      if (rotationQuantity_total - rotationQuantity < 0 - rotationQuantity_total_max)
      {
        // rotationQuantity_total - rotationQuantity > 0 - rotationQuantity_total_max にしておきたいから
        // ex -37 -5 < -40
        rotationQuantity = rotationQuantity_total_max + rotationQuantity_total;
        // 3= 40+(-37)
        // Serial.print("Corrected ");
      }

      turn_CW(rotationQuantity, rotationSpeed);
      back = false;
    }
    else
    {
      //rotationQuantity = 0;
    }
  }
  else // dir==0
  {
    //rotationQuantity = 0;
  }
}

//  roll値の処理
void stepRoll()
{
  // 最新のroll値を確認
  roll_roundf = roundf( delayedSensVal/*roll*/); // rollの四捨五入値
  // どのくらい動かせば良いかを知るために、1つ前の角度との差を計算
  // targetAngle = calcDiff(roll_roundf, roll_old); // ここでroll_oldからrollに動く方向がが正負のいずれかを見る
  calculateDirAndAbsDiff(roll_roundf, roll_old, dir, roll_diff);
  // Serial.print(roll_roundf);
  //  roll_diffが一定以上の場合は回転する
  if (roll_diff > roll_diff_th_min)
  {
    // 必要な回転量（ステップ数）を計算
    // 差を絶対値になるようにしているため、roll値が-90~90度をとるところを0~180度で考えている
    // 例）1ステップ3.44の場合90度動くには 90/3.44 = 26.16ステップ
    rotationQuantity = roundf(logN(roll_diff, roll_logBase)); // 目標角度をステップ数に変換
    // rotationSpeed = roundf(map(roll_diff, roll_diff_th_min, 20, 100, 255));
    rotationSpeed = duty_max;                                    // 定数でよければ（仮）
    rotateWithSensorValue(dir, rotationQuantity, rotationSpeed); // 方向、回転量、スピード      // memo: 多分この3つはローカル変数にしておかないとごちゃごちゃになる
    // if (rotationQuantity != 0) {//コメントアウトしてあったけどする必要なくね？
    printSteps(dir, roll_roundf, roll_old, roll_diff);
    // }
    roll_old = roll_roundf; // 次のループに向けて古い値として保存 回転した時のみ更新
  }
  else
  {
    rotationQuantity = 0;
  }
}

// x軸方向の加速度値の処理
void stepAccX()
{

  accX = delayedSensVal;
  // 回転方向決める
  if (accX < 0)
  {
    dir = -1; // CW
  }
  else if (accX > 0)
  {
    dir = 1; // CCW
  }
  else
  {
    dir = 0;
    return;
  }

  // accX_absolute の計算
  float accX_absolute = abs(accX);

  // accX_absolute の値に応じて rotationQuantity, rotationSpeed を割り当てるコードを書く
  if (accX_absolute > accX_th_min)
  {

    if (accX_absolute > accX_th_max) // 上限を8m/s^2として、それ以上の扱いは8にまるめる
    {
      accX_absolute = accX_th_max;
    }
    rotationQuantity = roundf(logN(accX_absolute, accX_logBase));
    // rotationSpeed = roundf(map(accX_absolute, accX_th_min, accX_th_max, 100, 255));
    rotationSpeed = duty_max; // 定数でよければ（仮）

    rotateWithSensorValue(dir, rotationQuantity, rotationSpeed);

    if (rotationQuantity != 0)
    {
      // accXには回ってる間にかなりの確率で次の値が代入される。printをaccXですると回転した時の値とずれるので、accX_absoluteでする。
      printSteps_acc(dir, accX_absolute, 0);
    }
  }
  else
  {
    rotationQuantity = 0;
  }
}



// x軸方向の加速度値の処理
void stepAccZ()
{
  accZ = delayedSensVal;
  // 回転方向決める
  if (accZ < 0)
  {
    dir = -1; // CW
  }
  else if (accZ > 0)
  {
    dir = 1; // CCW
  }
  else
  {
    dir = 0;
    return;
  }

  // accZ_absolute の計算
  float accZ_absolute = abs(accZ);

  // accZ_absolute の値に応じて rotationQuantity, rotationSpeed を割り当てるコードを書く
  if (accZ_absolute > accZ_th_min)
  {

    if (accZ_absolute > accZ_th_max) // 上限を8m/s^2として、それ以上の扱いは8にまるめる
    {
      accZ_absolute = accZ_th_max;
    }
    rotationQuantity = roundf(logN(accZ_absolute, accZ_logBase));
    // rotationSpeed = roundf(map(accZ_absolute, accZ_th_min, accZ_th_max, 100, 255));
    rotationSpeed = duty_max; // 定数でよければ（仮）

    rotateWithSensorValue(dir, rotationQuantity, rotationSpeed);

    if (rotationQuantity != 0)
    {
      // accZには回ってる間にかなりの確率で次の値が代入される。printをaccZですると回転した時の値とずれるので、accZ_absoluteでする。
      printSteps_acc(dir, accZ_absolute, 1);
    }
  }
  else
  {
    rotationQuantity = 0;
  }
}


// 差分計算・回転方向確定 暫定roll用の関数
// memo：値がひっくり返ってしまった時に無視する処理を入れておきたいが、なくても大丈夫そうかも
void calculateDirAndAbsDiff(int roll_roundf, int roll_old, int &dir, int &roll_diff)
{
  // 値の範囲を -90 から 90 に制限
  roll_roundf = std::max(-90, std::min(90, roll_roundf));
  roll_old = std::max(-90, std::min(90, roll_old));

  if (roll_roundf > roll_old)
  {
    dir = 1;
    roll_diff = roll_roundf - roll_old;
  }
  else if (roll_roundf < roll_old)
  {
    dir = -1;
    roll_diff = roll_old - roll_roundf;
  }
  else
  {
    dir = 0;
    roll_diff = 0;
  }

  if (roll_diff > roll_diff_th_max)
  {
    roll_diff = roll_diff_th_max;
  }
}

// センサ値の取得
double printEvent(sensors_event_t *event, int receivedValue)
{
  double x = -1000000, y = -1000000, z = -1000000; // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER)
  {
    // Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    //accX = x;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION)
  {
    // Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    roll = y;
    // return y;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
  {
    // Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE)
  {
    // Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR)
  {
    // Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
  {
    // Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    accX = x;
    accZ = z;
    // return ;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY)
  {
    // Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    // accX = x;
  }
  else
  {
    Serial.print("Unk:");
  }

  String str = "X:" + String(x) + "," + "Y:" + String(y) + "," + "Z:" + String(z);
  // Serial.println(str);
  //String str = "accX:" + String(accX) + "," + "roll:" + String(roll);

  if (deviceConnected) { //接続されていたら
    // 送信する値

    /*uint8_t*/String valueToSend = String(str);

    // BLE通知を行う
    pNotifyCharacteristic->setValue(valueToSend);
    pNotifyCharacteristic->notify();

    //Serial.print("send");
    //Serial.println(valueToSend);
  }

 
    switch (SensingTarget)
    {
      case 0:
        //Serial.print("y");
        return y;
        break;

      case 1:
        //Serial.print("x");
        return x;
        break;

      case 2:
        //Serial.print("z");
        return z;
        break;

      default:
        //適度に待つ
        delay(100);
    }

}
