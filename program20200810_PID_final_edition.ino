#include <SoftwareSerial.h>                                             //引用程式庫
#include <SPI.h>
#include <MFRC522.h>
SoftwareSerial BT(0, 1);                                                //藍芽接收腳, 傳送腳

#define RST_PIN         9                                               //RFID-RST腳位
#define SS_PIN          10                                              //RFID-SS腳位
#define IRQ_PIN         2                                               //RFID-IRQ腳位

MFRC522 mfrc522(SS_PIN, RST_PIN);                                       //RFID-RST、RFID-SS腳位設定
extern byte U[12];
volatile bool Read = false;
byte regVal = 0x7F;
void activateRec(MFRC522 mfrc522);
void clearInt(MFRC522 mfrc522);

extern byte U[12];                                                      //由外部.cpp傳入卡號
int len;                                                                //命令的長度
int orderseq = 0;                                                       //指定目的時命令提取順序
String order = "";                                                      //手機傳送過來的資料
int RFIDnum;                                                            //RFID卡片號碼總和
String cardnum = "", oldcardnum = "";                                   //當前卡片與上次讀到的卡片
int transcheck = 0;                                                     //確認傳送成功
int controlmode = 0;                                                    //0->自走模式,1->選擇路目的模式
int readRFID = 0;                                                       //0->自走模式,1->讀卡模式
int forceturn = 0;
int arrived = 0;                                                        //arrived = 1時,車輛停止
int breakflag = 0;

int card1 = (0x8E << 8) + (0x22);
int card2 = (0x28 << 8) + (0x2D);
int card3 = (0xD1 << 8) + (0x2D);
int card4 = (0x65 << 8) + (0x22);
int card5 = (0xA6 << 8) + (0x2D);
int card6 = (0x16 << 8) + (0x2D);
int card7 = (0xCA << 8) + (0x2D);
int card8 = (0x0A << 8) + (0x2D);
int card9 = (0xC6 << 8) + (0x02);
int card10 = (0xA2 << 8) + (0x2D);
int card11 = (0xC8 << 8) + (0x2D);
int card12 = (0x6C << 8) + (0x2D);
int card13 = (0x4C << 8) + (0x2D);
int card14 = (0x88 << 8) + (0x2D);
int card15 = (0x45 << 8) + (0x22);
int card16 = (0xFC << 8) + (0x22);
int card17 = (0x5C << 8) + (0x32);
int card18 = (0x81 << 8) + (0x22);


int sensorRead1, sensorRead2, sensorRead3, sensorRead4;                 //2號感測器讀值,3號感測器讀值,4號感測器讀值,5號感測器讀值
int Tp;                                                                 //max = 245,min = 175
int total;                                                              //total = Kp * P + Ki * I + Kd * D
float error = 0;                                                        //目前車輛偏移量
float last_error = 0;                                                   //上次車輛偏移量
float P, D, I = 0;
int Kp = 110, Ki = 30, Kd = 2200;
float Ke = 0.15;
int errorcount = 0;                                                     //判斷車輛穩定,errorcount越大表示越穩定
int stopcount = 0;                                                      //當stopcount>7000時,車輛停止
int count = 0;                                                          //左右迴轉計時器
int left = 0, right = 0;                                                //左、右輪輸出PWM
int b, c, d, e;                                                         //正反轉控制
int leftflag = 0, rightflag = 0;                                        //紀錄最後讀到的線位置
int lefted = 0, righted = 0;                                            //紀錄發生迴轉方向
int lefting = 0, righting = 0;                                          //兩者互鎖,降低迴轉干擾
int leftfail = 0, rightfail = 0, failflag = 0, fail = 0;                //轉向錯誤旗幟,當再次遇到彎道時直接以相反方向前進
int voltage;                                                            //目前電池電壓
unsigned long starttime, nowtime;                                       //在一定時間內,加強馬達輸出幫助起步
int sflag = 0;

void setup() {
  pinMode(5, OUTPUT);                                                   //en A
  pinMode(4, OUTPUT);                                                   //motor A+
  pinMode(3, OUTPUT);                                                   //motor A-
  pinMode(6, OUTPUT);                                                   //en B
  pinMode(7, OUTPUT);                                                   //motor B+
  pinMode(8, OUTPUT);                                                   //motor B-
  BT.begin(38400);                                                      //藍芽連線包率
  SPI.begin();                                                          //MFRC522通訊設定
  mfrc522.PCD_Init();                                                   //MFRC522初始化
  byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  pinMode(IRQ_PIN, INPUT_PULLUP);
  regVal = 0xA0;
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);
  Read = false;
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);
  starttime = millis();
}

void loop() {
  nowtime = millis() - starttime;
  if (millis() % 80 == 0) {
    while (BT.available()) {                                            //列印手機傳送到藍芽的資料
      if (arrived == 0) {
        breaks();
        delay(100);
        order = "";
        orderseq = 0;
        len = 0;
        while (abs(analogRead(A1) + analogRead(A2) - analogRead(A3) + analogRead(A4)) > 50 && (analogRead(A2) > 400 || analogRead(A3) > 400)) {
          if (analogRead(A1) + analogRead(A2) > analogRead(A3) + analogRead(A4) || lefting) {
            leftuturn();
          }
          else if (analogRead(A3) + analogRead(A4) > analogRead(A1) + analogRead(A2) || righting) {
            rightuturn();
          }
        }
      }
      else if (arrived == 1) {
        order = "";
        orderseq = 0;
        len = 0;
      }
      cease();
      order += BT.readString();
      transcheck = 1;
    }
    if (transcheck == 1) {                                              //接收成功
      controlmode = 0;
      readRFID = 1;
      arrived = 0;
      orderseq = 0;
      transcheck = 0;
      starttime = millis();
    }
  }

  if (order == "w") {                                                   //停車
    arrived = 1;
    readRFID = 0;
    order = "";
  }
  else if (order == "e") {                                              //恢復自走模式
    arrived = 0;
    readRFID = 0;
    order = "";
    cardnum = "";
    oldcardnum = "";
    sflag = 0;
  }
  else if (order == "s") {                                              //尋找兩張卡片,開始定位
    if (oldcardnum != "" && oldcardnum != cardnum && cardnum != "14" && cardnum != "15") {
      arrived = 1;
      order = "";
      breakflag = 1;
    }
  }
  else if (order.length() >= 2 && controlmode == 0) {                   //指定目的地模式
    controlmode = 1;
  }

  if (arrived == 1) {
    if (breakflag == 1) {
      breaks();
      delay(100);
      breakflag = 0;
      while (abs(analogRead(A1) + analogRead(A2) - analogRead(A3) + analogRead(A4)) > 50 && (analogRead(A2) > 400 || analogRead(A3) > 400)) {
        if (analogRead(A1) + analogRead(A2) > analogRead(A3) + analogRead(A4) || lefting ) {
          leftuturn();
        }
        else if (analogRead(A3) + analogRead(A4) > analogRead(A1) + analogRead(A2) || righting) {
          rightuturn();
        }
      }
    }
    cease();
    readRFID = 0;
    order = "";
    orderseq = 0;
    len = 0;
    transcheck = 0;
    controlmode = 0;
  }

  if (controlmode == 1) {
    len = order.length();
    if (order[orderseq] == cardnum[0] && order[orderseq + 1] == cardnum[1]) {
      orderseq += 2;
      if (order[orderseq] == '3' && order[orderseq + 1] == '3') {       //左轉
        straight();
        delay(250);
        while (analogRead(A1) + analogRead(A2) + analogRead(A3) + analogRead(A4) > 30 && analogRead(A3) + analogRead(A4) > 50) {
          leftturn();
        }
        while (analogRead(A1) + analogRead(A2) + analogRead(A3) + analogRead(A4) < 30 && analogRead(A3) + analogRead(A4) > 50) {
          leftturn();
        }
        orderseq += 2;
        rightflag = 0;
        leftflag = 1;
      }
      else if (order[orderseq] == '3' && order[orderseq + 1] == '4') {  //右轉
        straight();
        delay(250);
        while (analogRead(A1) + analogRead(A2) + analogRead(A3) + analogRead(A4) > 30 && analogRead(A1) + analogRead(A2) > 50) {
          rightturn();
        }
        while (analogRead(A1) + analogRead(A2) + analogRead(A3) + analogRead(A4) < 30 && analogRead(A1) + analogRead(A2) > 50) {
          rightturn();
        }
        orderseq += 2;
        rightflag = 1;
        leftflag = 0;
      }
      else if (order[orderseq] == '3' && order[orderseq + 1] == '1') {  //直走
        if (rightflag == 0 && leftflag == 0) {
          if (analogRead(A1) + analogRead(A2) > analogRead(A3) + analogRead(A4)) {
            rightflag = 0;
            leftflag = 1;
          }
          else {
            rightflag = 1;
            leftflag = 0;
          }
        }
        orderseq += 2;
      }
      else if (order[orderseq] == '3' && order[orderseq + 1] == '2') {  //迴轉
        if ( (order[orderseq - 2] == '0' && (order[orderseq - 1] == '2' || order[orderseq - 1] == '3' || order[orderseq - 1] == '6' || order[orderseq - 1] == '8'))  || (order[orderseq - 2] == '1' && (order[orderseq - 1] == '1' || order[orderseq - 1] == '3' || order[orderseq - 1] == '6' || order[orderseq - 1] == '7' || order[orderseq - 1] == '8'))) {
          leftuturn();
          delay(300);
        }
        else {
          straight();
          delay(400);
          leftuturn();
          delay(600);
        }
        rightflag = 0;
        leftflag = 1;
        count = 0;
        orderseq += 2;
      }
    }
    if ((orderseq == len) || (order[len - 2] == cardnum[0] && order[len - 1] == cardnum[1])) {
      arrived = 1;
      order = "";
      orderseq = 0;
      len = 0;
      controlmode = 0;
      breakflag = 1;
    }
  }
  if (arrived == 0) {
    voltage = analogRead(A0);
    sensorRead1 = analogRead(A1);
    sensorRead2 = analogRead(A2);
    sensorRead3 = analogRead(A3);
    sensorRead4 = analogRead(A4);
    if (forceturn == 1 && (rightflag == 1 || leftflag == 1)) {
      full_turn(leftflag, rightflag, &count, &stopcount, sensorRead1, sensorRead2, sensorRead3, sensorRead4, &b, &c, &d, &e, &left, &right, &lefted, &righted, &leftfail, &rightfail, &failflag, &fail);
      forceturn = 0;
    }
    else if (sensorRead1 + sensorRead2 + sensorRead3 + sensorRead4 > 100) {
      forceturn = 0;
      count = 0;
      stopcount = 0;
      failflag = 0;
      b = 1;
      c = 0;
      d = 1;
      e = 0;
      if (sensorRead1 + sensorRead2 > sensorRead3 + sensorRead4) {        //#1234
        rightflag = 0;
        leftflag = 1;
        if ((sensorRead1 > sensorRead2) && sensorRead2 < 200 && righting == 0) { //#1
          error = 3.1 * Ke;
          lefting = 1;
          lefted = 1;
        }
        else if (sensorRead1 > sensorRead2) {                             //#2
          error = 2.1 * Ke;
          if (lefted == 1) {
            lefting = 0;
            lefted = 0;
          }
        }
        else if (sensorRead1 > sensorRead3) {                             //#3
          error = 1.1 * Ke;
          if (lefted == 1) {
            lefting = 0;
            lefted = 0;
          }
        }
        else if (sensorRead2 > sensorRead3 && sensorRead3 > sensorRead1 && sensorRead2 > sensorRead1) {      //#4
          error = 0.1 * Ke;
          if (lefted == 1) {
            lefting = 0;
            lefted = 0;
          }
          righting = 0;
          righted = 0;
        }
      }
      else if (sensorRead1 + sensorRead2 < sensorRead3 + sensorRead4) {   //#5678
        rightflag = 1;
        leftflag = 0;
        if ((sensorRead4 > sensorRead3) && sensorRead3 < 200 && lefting == 0) {  //#8
          error = -3.1 * Ke;
          righting = 1;
          righted = 1;
        }
        else if (sensorRead4 > sensorRead3) {                             //#7
          error = -2.1 * Ke;
          if (righted == 1) {
            righting = 0;
            righted = 0;
          }
        }
        else if (sensorRead4 > sensorRead2) {                             //#6
          error = -1.1 * Ke;
          if (righted == 1) {
            righting = 0;
            righted = 0;
          }
        }
        else if (sensorRead3 > sensorRead2 && sensorRead2 > sensorRead4 && sensorRead3 > sensorRead4) {       //#5
          error = -0.1 * Ke;
          if (righted == 1) {
            righting = 0;
            righted = 0;
          }
          lefting = 0;
          lefted = 0;
        }
      }
      PIDoutput(error, &I, &errorcount, &last_error, &left, &right, &Tp, &rightfail, &leftflag, &failflag, readRFID, voltage, nowtime, lefting, righting);
    }

    else if (rightfail == 1 || leftfail == 1) {
      stopcount += 1;
      rightflag = rightfail;
      leftflag = leftfail;
      rightfail = 0;
      leftfail = 0;
      failflag = 1;
      full_turn(leftflag, rightflag, &count, &stopcount, sensorRead1, sensorRead2, sensorRead3, sensorRead4, &b, &c, &d, &e, &left, &right, &lefted, &righted, &leftfail, &rightfail, &failflag, &fail);
    }
    else if (rightflag == 1 || leftflag == 1) {
      stopcount += 1;
      full_turn(leftflag, rightflag, &count, &stopcount, sensorRead1, sensorRead2, sensorRead3, sensorRead4, &b, &c, &d, &e, &left, &right, &lefted, &righted, &leftfail, &rightfail, &failflag, &fail);
    }

    digitalWrite(7, b);
    digitalWrite(8, c);
    digitalWrite(4, d);
    digitalWrite(3, e);
    int limitflag = 0;
    if (left == 0 || right == 0) {
      limitflag = 1;
    }
    analogWrite(5, constrain((left), 0, ((220 + limitflag * 25) * (voltage >= 500) + (230 + limitflag * 20) * (voltage < 500 && voltage >= 400) + 245 * (voltage < 400) + 0 * (nowtime < 1000))));
    analogWrite(6, constrain((right), 0, ((220 + limitflag * 25) * (voltage >= 500) + (230 + limitflag * 20) * (voltage < 500 && voltage >= 400) + 245 * (voltage < 400) + 0 * (nowtime < 1000))));
  }


  if (readRFID == 1 && Read == true) {
    if (!mfrc522.PICC_IsNewCardPresent()) {
      return;
    }
    if (!mfrc522.PICC_ReadCardSerial()) {
      return;
    }
    oldcardnum = cardnum;
    mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid));                 //列印卡片UID
    RFIDnum = (U[2] << 8) + (U[3]);                                   //將卡片UID組合併寫入RFIDnum
    findcard(RFIDnum, &cardnum);
    BT.print(cardnum);                                                //於手機上顯示目前讀取卡片
    clearInt(mfrc522);
    mfrc522.PICC_HaltA();
    Read = false;
    forceturn = 1;
    if (sflag == 1 && oldcardnum != cardnum) {
      arrived = 1;
      breakflag = 1;
    }
    else if (oldcardnum != "" && oldcardnum != cardnum && cardnum != "14" && cardnum != "15") {
      sflag = 1;
      arrived = 1;
      breakflag = 1;
    }
  }
  activateRec(mfrc522);

  if (stopcount >= 7000) {
    left = 0;
    right = 0;
  }
}


void full_turn(int leftflag, int rightflag, int *count, int *stopcount, int sensorRead1, int sensorRead2 , int sensorRead3, int sensorRead4, int *b, int *c, int*d, int *e, int *left, int *right, int *lefted, int *righted, int *leftfail, int *rightfail, int *failflag, int *fail) {
  *count += 1;
  if ((((*count > 0) && (*count < 1200)) || (*failflag == 1)) && (sensorRead1 + sensorRead2 + sensorRead3 + sensorRead4 < 80)) {
    *b = leftflag;
    *c = rightflag;
    *d = rightflag;
    *e = leftflag;
    *lefted = leftflag;
    *righted = rightflag;
    *left = 255 * rightflag + 255 * leftflag;
    *right = 255 * leftflag + 255 * rightflag;
  }
  else if (sensorRead1 + sensorRead2 + sensorRead3 + sensorRead4 < 80) {
    if ((*count >= 1200) && (*count < 1300)) {
      *b = rightflag;
      *c = leftflag;
      *d = leftflag;
      *e = rightflag;
      *left = 255 * rightflag + 255 * leftflag;
      *right = 255 * leftflag + 255 * rightflag;
    }
    else {
      *b = 0;
      *c = 1;
      *d = 0;
      *e = 1;
      *left = 100 * leftflag + 220 * rightflag;
      *right = 100 * rightflag + 220 * leftflag;
      *lefted = rightflag;
      *righted = leftflag;
    }
    *fail = 1;
  }
  else if ((*fail == 1) && sensorRead1 + sensorRead2 + sensorRead3 + sensorRead4 >= 80) {
    if (*failflag == 0) {
      *leftfail = rightflag;
      *rightfail = leftflag;
      *failflag = 1;
      *stopcount = 0;
      *count = 0;
    }
    else if (*failflag == 1 && (sensorRead1 + sensorRead2 + sensorRead3 + sensorRead4 >= 80)) {
      *failflag = 0;
      *fail = 0;
    }
    else {
      *failflag = 1;
    }
  }
}

void PIDoutput(float error, float * I, int *errorcount, float * last_error, int *left, int *right, int*Tp, int *rightfail, int *leftfail, int *failflag, int readRFID, int voltage, unsigned long nowtime, int lefting, int righting) {
  P = error;
  *I = *I * 0.6 + error;
  if (abs(error) > 0.3) {
    D = 0.4 * error / abs(error);
    *errorcount = 0;
  }
  else {
    D = error - *last_error;
    *errorcount += 1;
  }
  if (*errorcount > 500 || (nowtime < 200)) {
    *rightfail = 0;
    *leftfail = 0;
    *failflag = 0;
    *Tp = 190 * (voltage >= 500) + 200 * (voltage < 500 && voltage >= 400) + 210 * (voltage < 400) + 45 * (nowtime < 200) - 30 * (lefting || righting) * (nowtime < 200);
  }
  else if (abs(D) < 0.15) {
    *Tp = 180 * (voltage >= 500) + 190 * (voltage < 500 && voltage >= 400) + 200 * (voltage < 400);
  }
  else if (abs(D) >= 0.15 && abs(D) < 0.3) {
    *Tp = 160;
  }
  else if (abs(D) >= 0.3 || abs(error) >= 0.3) {
    *Tp = 60;
  }
  *last_error = error;
  total = Kp * P + Ki * (abs(*I) < 0.6) * *I + Kd * D;
  *left = constrain(*Tp - (290 - *Tp) / 4.5 - total, 0, 255);
  *right = constrain(*Tp + (290 - *Tp) / 4.5 + total, 0, 255);
}

void rightturn() {
  digitalWrite(7, 0);
  digitalWrite(8, 1);
  digitalWrite(4, 1);
  digitalWrite(3, 0);
  analogWrite(5, 220);
  analogWrite(6, 250);
}

void leftturn() {
  digitalWrite(7, 1);
  digitalWrite(8, 0);
  digitalWrite(4, 0);
  digitalWrite(3, 1);
  analogWrite(5, 250);
  analogWrite(6, 220);
}

void straight() {
  digitalWrite(7, 1);
  digitalWrite(8, 0);
  digitalWrite(4, 1);
  digitalWrite(3, 0);
  analogWrite(5, 235);
  analogWrite(6, 255);
}

void leftuturn() {
  digitalWrite(7, 1);
  digitalWrite(8, 0);
  digitalWrite(4, 0);
  digitalWrite(3, 1);
  analogWrite(5, 255);
  analogWrite(6, 255);
}

void rightuturn() {
  digitalWrite(7, 0);
  digitalWrite(8, 1);
  digitalWrite(4, 1);
  digitalWrite(3, 0);
  analogWrite(5, 255);
  analogWrite(6, 255);
}

void cease() {
  digitalWrite(7, 0);
  digitalWrite(8, 0);
  digitalWrite(4, 0);
  digitalWrite(3, 0);
  analogWrite(5, 0);
  analogWrite(6, 0);
}

void breaks() {
  digitalWrite(7, 0);
  digitalWrite(8, 1);
  digitalWrite(4, 0);
  digitalWrite(3, 1);
  analogWrite(5, 235);
  analogWrite(6, 255);
}

void readCard() {
  Read = true;
}

void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}

void findcard(int RFIDnum, String * cardnum) {
  if (RFIDnum == card1) {
    *cardnum = "01";
  }
  else if (RFIDnum == card2) {
    *cardnum = "02";
  }
  else if (RFIDnum == card3) {
    *cardnum = "03";
  }
  else if (RFIDnum == card4) {
    *cardnum = "04";
  }
  else if (RFIDnum == card5) {
    *cardnum = "05";
  }
  else if (RFIDnum == card6) {
    *cardnum = "06";
  }
  else if (RFIDnum == card7) {
    *cardnum = "07";
  }
  else if (RFIDnum == card8) {
    *cardnum = "08";
  }
  else if (RFIDnum == card9) {
    *cardnum = "09";
  }
  else if (RFIDnum == card10) {
    *cardnum = "10";
  }
  else if (RFIDnum == card11) {
    *cardnum = "11";
  }
  else if (RFIDnum == card12) {
    *cardnum = "12";
  }
  else if (RFIDnum == card13) {
    *cardnum = "13";
  }
  else if (RFIDnum == card14) {
    *cardnum = "14";
  }
  else if (RFIDnum == card15) {
    *cardnum = "15";
  }
  else if (RFIDnum == card16) {
    *cardnum = "16";
  }
  else if (RFIDnum == card17) {
    *cardnum = "17";
  }
  else if (RFIDnum == card18) {
    *cardnum = "18";
  }
}
