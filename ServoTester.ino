#include <Servo.h>
#include <String.h>
#include <Arduino.h>
#include <ILI9341_t3.h>
#include <tbParam.h>
#include <textBox.h>
#include <ioChan.h>

//---HW Pin Defines---
#define TFT_DC  9
#define TFT_CS 10
#define TFT_RST 8

#define STAT_OUT_PIN    LED_BUILTIN /* 13 */
#define SVO_IN_PIN      A0          /* 15 */
#define BTN_L_PIN       A1          /* 21 */
#define BTN_M_PIN       A2          /* 17 */
#define BTN_R_PIN       A3          /* 16 */
#define SVO_OUT_0_PIN   A4          /* 18 */
#define SVO_OUT_1_PIN   A9          /* 23 */
#define SVO_OUT_2_PIN   A8          /* 22 */
#define SVO_OUT_3_PIN   A7          /* 21 NC */
#define SVO_OUT_4_PIN   A6          /* 20 */
#define SVO_OUT_5_PIN   A5          /* 19 */
#define SVO_OUT_6_PIN   A10         /*  NC */
#define SVO_OUT_7_PIN   A11         /*  NC */

//---Non HW defines---
#define SER_DELAY 3



#define TB_OAT_X        0
#define TB_OAT_Y        0


#define TB_MODE_X        FONT_S2_W * 5
#define TB_MODE_Y        S2_TB_H


#define TB_BTN_L_X      0
#define TB_BTN_L_Y      S2_TB_H * 2

#define TB_BTN_M_X      (FONT_S2_W * 6)
#define TB_BTN_M_Y      TB_BTN_L_Y

#define TB_BTN_R_X      FONT_S2_W * 12
#define TB_BTN_R_Y      TB_BTN_L_Y

#define TB_BTN_L_TXT_X  0
#define TB_BTN_L_TXT_Y  (S2_TB_H * 3)

#define TB_BTN_M_TXT_X  FONT_S3_W * 4
#define TB_BTN_M_TXT_Y  TB_BTN_L_TXT_Y

#define TB_BTN_R_TXT_X  FONT_S3_W * 8
#define TB_BTN_R_TXT_Y  TB_BTN_L_TXT_Y

#define TB_SVO0_POS_X    95
#define TB_SVO0_POS_Y    (S2_TB_H * 3) + S3_TB_H

#define TB_SVO1_POS_X    95
#define TB_SVO1_POS_Y    (S2_TB_H * 3) + (S3_TB_H *2)

#define TB_SVO2_POS_X    95
#define TB_SVO2_POS_Y    (S2_TB_H * 3) + (S3_TB_H *3)

#define TB_SVO3_POS_X    95
#define TB_SVO3_POS_Y    (S2_TB_H * 3) + (S3_TB_H *4)

#define TB_SVO4_POS_X    95
#define TB_SVO4_POS_Y    (S2_TB_H * 3) + (S3_TB_H *5)

#define BTN_THRES       50

#define RETRACT_MIN     400
#define RETRACT_CENT    900
#define RETRACT_MAX     1600

enum SvoAction {
  SVO_ACT_NA,                 //---Trivial servo state while in boot
  SVO_ACT_PWRUP,              //---Trivial servo state while in power up
  SVO_ACT_INC_IDLE,           //---Servo idle state---
  SVO_ACT_DEC_R,              //---Servo moving right (pos decrementing)---
  SVO_ACT_CENT,               //---Servo centering---
  SVO_ACT_INC_L,              //---Servo moving left (pos incrementing)---
  SVO_ACT_R_MAX,              //---Servo right max state---
  SVO_ACT_L_MIN,              //---Servo left max state---
  SVO_ACT_ANALOG_SLAVE,       //---Servo slaved to analog servo input cmd
  SVO_ACT_RETRACT_IDLE,
  SVO_ACT_RETRACT_IDLE_UP,
  SVO_ACT_RETRACT_IDLE_DWN,
  SVO_ACT_RETRACT_IDLE_CENT,
  SVO_ACT_RETRACT_UP,         //---Retract pos up
  SVO_ACT_RETRACT_DOWN,       //---Retract pos down
  SVO_ACT_RETRACT_CENT,       //---Retract center
  SVO_ACT_RETRACT_TO_DWN,  //---Retract transitions from up to down
  SVO_ACT_RETRACT_TO_UP,  //---Retract transitions from down to up
  SVO_ACT_RETRACT_TO_CENTER,
  NUM_SVO_ACTION
};

String svo_ActionStr[NUM_SVO_ACTION] = {
  "SVO_NA",
  "SVO_ACT_PWRUP",
  "SVN_INC_IDLE",
  "SVO_DEC_R",
  "SVO_CENT",
  "SVO_INC_L",
  "SVO_R_MAX",
  "SVO_L_MIN",
  "SVO_AN_SLAVE",
  "SVO_RET_IDLE",
  "SVO_RET_IDLE_UP",
  "SVO_RET_IDLE_DWN",
  "SVO_RET_IDLE_CENT",
  "SVO_RET_UP",
  "SVO_RET_DOWN",
  "SVO_RET_CENT",
  "SVO_RET_2_D",
  "SVO_RET_2_U",
  "SVO_RED_2_C"
};
enum TaskType {
  TASK_STAT_LED,
  TASK_POWER_UP,
  TASK_SAWTOOTH,
  TASK_VOLTAGE_MON,
  TASK_BTN_READ,
  TASK_OP_MODE,
  TASK_SERIAL,
  TASK_SERVO,
  TASK_TFT,
  NUM_TASKS
};

enum opModes {
  OP_MD_NA,
  OP_MD_SERIAL_WAIT,
  OP_MD_BOOT,
  OP_MD_PWRUP,
  OP_MD_ANALOG_PAN,
  OP_MD_INC_TO_ANALOG_TRANS,
  OP_MD_ANALOG_TO_INC_TRANS,
  OP_MD_INC_PAN,
  OP_MD_COMPASS_TRACK,
  OP_MD_RETRACT,
  OP_MD_NGR,        //---for testing a retractable nose gear: two channels
                    //   1 for steering servo, 1 for retract
  NUM_OP_MODES
};


String opMdStr[NUM_OP_MODES] = {
  "NA",
  "Serial Wait",
  "Boot",
  "Powerup",
  "Analog Pan",
  "Inc 2 An",
  "An 2 Inc",
  "Inc Pan",
  "Compass Track",
  "Retract",
  "Nose Gear Retract"
};

String taskStr[NUM_TASKS] = {
  "TASK_STAT_LED",
  "TASK_POWER_UP",
  "TASK_SAWTOOTH",
  "TASK_VOLTAGE_MON",
  "TASK_BTN_READ",
  "TASK_OP_MODE",
  "TASK_SERIAL",
  "TASK_SERVO",
  "TASK_TFT"
};

enum powerUpSteps {
  PWRUP_NA,
  PWRUP_0,    //---All LEDs off
  PWRUP_1,    //---LF on
  PWRUP_2,    //---LF, RF on
  PWRUP_3,    //---LF, RF, RR on
  PWRUP_4,    //---LF, RF, RR, LR on
  PWRUP_5,    //---LF, RF, RR on
  PWRUP_6,    //---LF, RF on
  PWRUP_7,    //---LF, on
  PWRUP_DONE, //---All LEDs off
  NUM_PWRUP_STEPS
};

String pwrUpStr[NUM_PWRUP_STEPS] = {
  "PWRUP_NA",
  "PWRUP_0",
  "PWRUP_1",
  "PWRUP_2",
  "PWRUP_3",
  "PWRUP_4",
  "PWRUP_5",
  "PWRUP_6",
  "PWRUP_7",
  "PWRUP_DONE"
};

unsigned int opMode;
unsigned int pwrUpStep;
unsigned int statLedState;
unsigned int svoAction;


// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

//---Boolean vars for button state---
char btnL;
char btnM;
char btnR;

long OATVar;
long POTVar;
long btnLVar;
long btnMVar;
long btnRVar;
long SvoInVar;
long SvoErr;
int SvoDone;
long SvoOutMaster;
long SvoOutNWS;       //---Nose wheel steering
long SvoOutNGR;       //---Nose gear retract
long SvoOutLR;        //---Left main retract
long SvoOutRR;        //---Right main retract
long SvoOut0Var;
long SvoOut1Var;
long SvoOut2Var;
long SvoOut3Var;
long SvoOut4Var;
long SvoOut5Var;
long SvoOut6Var;
long SvoOut7Var;

String ModeStr;
String btnLStr;
String btnMStr;
String btnRStr;
String AppTitle;
String AppVersion;

ioChannel OATChan;
ioChannel btnLChan;
ioChannel btnMChan;
ioChannel btnRChan;
ioChannel SvoInChan;
ioChannel SvoOut0Chan;
ioChannel SvoOut1Chan;
ioChannel SvoOut2Chan;
ioChannel SvoOut3Chan;
ioChannel SvoOut4Chan;
ioChannel SvoOut5Chan;
ioChannel SvoOut6Chan;
ioChannel SvoOut7Chan;

textBoxObj* tboCH0;
textBoxObj* tboCH1;
textBoxObj* tboCH2;
textBoxObj* tboCH3;
textBoxObj* tboCH4;
textBoxObj* tboCH5;
textBoxObj* tboCH6;
textBoxObj* tboCH7;

textBoxObj* tbMode;   //STYLE_CUSTOM w/ ioChan source

textBoxObj* tbOAT;   //STYLE_CUSTOM w/ ioChan source

textBoxObj* tbBtnL;   //STYLE_CUSTOM w/ tbParam source
textBoxObj* tbBtnM;   //STYLE_CUSTOM w/ fix integer source source
textBoxObj* tbBtnR;   //STYLE_DEFAULT w/ ioChan source

textBoxObj* tboSvoPos;   //STYLE_DEFAULT w/ tbParam source
textBoxObj* tboSvoNWSPos;   //STYLE_DEFAULT w/ tbParam source
textBoxObj* tboSvoNGRPos;   //STYLE_DEFAULT w/ tbParam source
textBoxObj* tboSvoLRPos;   //STYLE_DEFAULT w/ tbParam source
textBoxObj* tboSvoRRPos;   //STYLE_DEFAULT w/ tbParam source

textBoxObj* tbBtnLTxt;   //STYLE_DEFAULT w/ fix integer source source

textBoxObj* tbBtnMTxt;   //STYLE_BOUNDED_BOX w/ ioChan source
textBoxObj* tbBtnRTxt;   //STYLE_BOUNDED_BOX w/ tbParam source


/*-----------------------------------------------------------------------------
 *
 *
 *
 *---------------------------------------------------------------------------*/
void setupTFT() {
  AppTitle = "TFT Servo Tester";
  AppVersion = "v0.2";


  Serial.println("Serial Ready");
  Serial.println("Start TFT");

  tft.begin();
  delay(250);
  tft.fillScreen(ILI9341_BLACK);

  delay(250);
  tft.fillScreen(ILI9341_RED);

  // read diagnostics (optional but can help debug problems)
  int x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);

  Serial.print("TFT Width:");
  Serial.println(tft.width());

  Serial.print("TFT Height:");
  Serial.println(tft.height());

  delay(250);
  tft.fillScreen(ILI9341_BLUE);


  delay(250);
  tft.fillScreen(ILI9341_GREEN);

  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(2);
  delay(250);
  tft.fillScreen(ILI9341_BLACK);
  tft.println(AppTitle);
  tft.println(AppVersion);



}
/*-----------------------------------------------------------------------------
 *
 *
 *
 *---------------------------------------------------------------------------*/
void setup() {
  int tmr = 0;
  int toggle = 0;
  int serialOk = 0;

  Serial.begin(115200);

  tmr = SER_DELAY;
  while(tmr > 0 && serialOk == 0) {

    if(Serial)
      serialOk = 1;
    delay(1000);

    toggle = !toggle;

    digitalWrite(STAT_OUT_PIN, toggle);
    tmr--;
  }

  opMode = OP_MD_NA;
  pwrUpStep = 0;
  svoAction = SVO_ACT_NA;

  btnLStr = "OFF";
  btnMStr = "OFF";
  btnRStr = "OFF";
  btnL = 0;
  btnM = 0;
  btnR = 0;
  SvoInVar = SERVO_POS_MID;
  SvoDone = -1;
  SvoErr = -1;
  SvoOutMaster = SERVO_POS_MID;
  SvoOutNWS = SERVO_POS_MID;
  SvoOutNGR = SERVO_POS_MID;
  SvoOutLR = SERVO_POS_MID;
  SvoOutRR = SERVO_POS_MID;
  SvoOut0Var = SERVO_POS_MID;
  SvoOut1Var = SERVO_POS_MID;
  SvoOut2Var = SERVO_POS_MID;
  SvoOut3Var = SERVO_POS_MID;
  SvoOut4Var = SERVO_POS_MID;
  SvoOut5Var = SERVO_POS_MID;
  SvoOut6Var = SERVO_POS_MID;
  SvoOut7Var = SERVO_POS_MID;

  setupTFT();


  Serial.println(AppTitle);
  Serial.println(AppVersion);

  Serial.println("Init ioChannels");
//  OATChan = ioChannel(IO_TYPE_AIN_LM35_3V3, SVO_OUT_0_PIN, &OATVar);
  SvoInChan = ioChannel(IO_TYPE_AIN_3V3_1800, SVO_IN_PIN, &SvoInVar);
  SvoInChan.ioFilter = IO_FILT_WEIGHTED_AVG;

  btnLChan = ioChannel(IO_TYPE_AIN_RAW, BTN_L_PIN, &btnLVar);
  btnLChan.ioFilter = IO_FILT_WEIGHTED_AVG;

  btnMChan = ioChannel(IO_TYPE_AIN_RAW, BTN_M_PIN, &btnMVar);
  btnMChan.ioFilter = IO_FILT_WEIGHTED_AVG;

  btnRChan = ioChannel(IO_TYPE_AIN_RAW, BTN_R_PIN, &btnRVar);
  btnRChan.ioFilter = IO_FILT_WEIGHTED_AVG;

  SvoOut0Chan = ioChannel(IO_TYPE_DOUT_SERVO_180, SVO_OUT_0_PIN, &SvoOut0Var);
  SvoOut1Chan = ioChannel(IO_TYPE_DOUT_SERVO_180, SVO_OUT_1_PIN, &SvoOut1Var);
  SvoOut2Chan = ioChannel(IO_TYPE_DOUT_SERVO_180, SVO_OUT_2_PIN, &SvoOut2Var);
  SvoOut3Chan = ioChannel(IO_TYPE_DOUT_SERVO_180, SVO_OUT_3_PIN, &SvoOut3Var);
  SvoOut4Chan = ioChannel(IO_TYPE_DOUT_SERVO_180, SVO_OUT_4_PIN, &SvoOut4Var);
  SvoOut5Chan = ioChannel(IO_TYPE_DOUT_SERVO_180, SVO_OUT_5_PIN, &SvoOut5Var);
  SvoOut6Chan = ioChannel(IO_TYPE_DOUT_SERVO_180, SVO_OUT_6_PIN, &SvoOut6Var);
  SvoOut7Chan = ioChannel(IO_TYPE_DOUT_SERVO_180, SVO_OUT_7_PIN, &SvoOut7Var);


  Serial.println("Init Textboxes0");
  //textBoxObj(tft, str, x, y, size, style, fontClr, bgClr);
  int tempX = 60;
  int tempInc = 15;
  int tempY = 0;
  tboCH7 = new textBoxObj(&tft, "8", tempX, tempY, (sizeType)SZ_1, STYLE_BOUNDED_BOX_STATIC, tbRD, tbBLK);
  tempX += tempInc;
  tboCH6 = new textBoxObj(&tft, "7", tempX, tempY, (sizeType)SZ_1, STYLE_BOUNDED_BOX_STATIC, tbORG, tbBLK);
  tempX += tempInc;
  tboCH5 = new textBoxObj(&tft, "6", tempX, tempY, (sizeType)SZ_1, STYLE_BOUNDED_BOX_STATIC, tbYLW, tbBLK);
  tempX += tempInc;
  tboCH4 = new textBoxObj(&tft, "5", tempX, tempY, (sizeType)SZ_1, STYLE_BOUNDED_BOX_STATIC, tbMedGRN, tbBLK);
  tempX += tempInc;
  tboCH3 = new textBoxObj(&tft, "4", tempX, tempY, (sizeType)SZ_1, STYLE_BOUNDED_BOX_STATIC, tbGRN, tbBLK);
  tempX += tempInc;
  tboCH2 = new textBoxObj(&tft, "3", tempX, tempY, (sizeType)SZ_1, STYLE_BOUNDED_BOX_STATIC, tbBL, tbBLK);
  tempX += tempInc;
  tboCH1 = new textBoxObj(&tft, "2", tempX, tempY, (sizeType)SZ_1, STYLE_BOUNDED_BOX_STATIC, tbInigo, tbBLK);
  tempX += tempInc;
  tboCH0 = new textBoxObj(&tft, "1", tempX, tempY, (sizeType)SZ_1, STYLE_BOUNDED_BOX_STATIC, tbViolet, tbBLK);



  Serial.println("Init Textboxes1");
  tbMode = new textBoxObj(&tft, "Mode", TB_MODE_X, TB_MODE_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX, tbRD, tbBLK);

  Serial.println("Init Textboxes2");
  tbBtnL = new textBoxObj(&tft, &btnLChan, TB_BTN_L_X, TB_BTN_L_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX, tbORG, tbBLK);

  Serial.println("Init Textboxes3");
  tbBtnM = new textBoxObj(&tft, &btnMChan, TB_BTN_M_X, TB_BTN_M_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX, tbYLW, tbBLK);

  Serial.println("Init Textboxes4");
  tbBtnR = new textBoxObj(&tft, &btnRChan, TB_BTN_R_X, TB_BTN_R_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX, tbYLW, tbBLK);

  Serial.println("Init Textboxes5");
  tbBtnLTxt = new textBoxObj(&tft, "ButtonL OFF", TB_BTN_L_TXT_X, TB_BTN_L_TXT_Y, (sizeType)SZ_3, STYLE_BOUNDED_BOX, tbRD, tbBLK);

  Serial.println("Init Textboxes6");
  tbBtnMTxt = new textBoxObj(&tft, "ButtonM OFF", TB_BTN_M_TXT_X, TB_BTN_M_TXT_Y, (sizeType)SZ_3, STYLE_BOUNDED_BOX, tbWHT, tbBLK);

  Serial.println("Init Textboxes7");
  tbBtnRTxt = new textBoxObj(&tft, "ButtonR OFF", TB_BTN_R_TXT_X, TB_BTN_R_TXT_Y, (sizeType)SZ_3, STYLE_BOUNDED_BOX, tbGRN, tbBLK);


  Serial.println("Init Textboxes8");
  tboSvoPos = new textBoxObj(&tft, &SvoOut0Chan, TB_SVO0_POS_X, TB_SVO0_POS_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX_DYNAMIC, tbCY, tbBLK);
  tboSvoNWSPos = new textBoxObj(&tft, &SvoOut1Chan, TB_SVO1_POS_X, TB_SVO1_POS_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX_DYNAMIC, tbGRN, tbBLK);
  tboSvoNGRPos = new textBoxObj(&tft, &SvoOut2Chan, TB_SVO2_POS_X, TB_SVO2_POS_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX_DYNAMIC, tbYLW, tbBLK);
  tboSvoLRPos = new textBoxObj(&tft, &SvoOut3Chan, TB_SVO3_POS_X, TB_SVO3_POS_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX_DYNAMIC, tbORG, tbBLK);
  tboSvoRRPos = new textBoxObj(&tft, &SvoOut4Chan, TB_SVO4_POS_X, TB_SVO4_POS_Y, (sizeType)SZ_2, STYLE_BOUNDED_BOX_DYNAMIC, tbCY, tbBLK);

  Serial.println("Init Textboxes9");
  tft.fillScreen(ILI9341_BLACK);
  Serial.println("Init Textboxes Done");

  tboCH0->tboStale = 1;
  tboCH1->tboStale = 1;
  tboCH2->tboStale = 1;
  tboCH3->tboStale = 1;
  tboCH4->tboStale = 1;
  tboCH5->tboStale = 1;
  tboCH6->tboStale = 1;
  tboCH7->tboStale = 1;

  tboCH0->tboRedraw();
  tboCH1->tboRedraw();
  tboCH2->tboRedraw();
  tboCH3->tboRedraw();
  tboCH4->tboRedraw();
  tboCH5->tboRedraw();
  tboCH6->tboRedraw();
  tboCH7->tboRedraw();

}
//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
//void SerialPrintVar(String str, void *var ) {
//  String tmp = " " + str + ":";
//  Serial.print(tmp);
//  Serial.print((String)var);
//}
//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
int taskPowerUp(void) {
  static int pwrUpCnt = 0;

  pwrUpCnt++;

  switch(pwrUpStep) {
    default:
    case PWRUP_NA:
      pwrUpStep = PWRUP_0;
      break;

    case PWRUP_0:
      pwrUpStep = PWRUP_DONE;
      break;

    case PWRUP_DONE:
      break;

  }

  return pwrUpStep;
}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskStatLED(void) {
  static int cnt= 0;
  static int ledOn = 0;

  switch(opMode) {
    default:
    case OP_MD_NA:
      break;

    case OP_MD_SERIAL_WAIT:
    case OP_MD_BOOT:
      ledOn = !ledOn;
      break;

    case OP_MD_PWRUP:
      cnt++;
      if(cnt < 15)
        ledOn = true;
      else if(cnt < 30)
        ledOn = false;
      else
        cnt = 0;
      break;
  }

  digitalWrite(STAT_OUT_PIN, ledOn);
}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskTFT() {
  static String btnLStrShdw = btnLStr;
  static String btnMStrShdw = btnMStr;
  static String btnRStrShdw = btnRStr;
  static int firstTime = 1;

  tbMode->tboSetString(opMdStr[opMode]);

  tbBtnL->tboRedraw();
  tbBtnM->tboRedraw();
  tbBtnR->tboRedraw();
  tboSvoPos->tboRedraw();
  tboSvoNWSPos->tboRedraw();

  if(firstTime == 1) {
    tboCH0->tboRedraw();
    tboCH1->tboRedraw();
    tboCH2->tboRedraw();
    tboCH3->tboRedraw();
    tboCH4->tboRedraw();
    tboCH5->tboRedraw();
    tboCH6->tboRedraw();
    tboCH7->tboRedraw();
    firstTime = 0;
  }

  tbBtnLTxt->tboSetString(btnLStr);
  tbBtnMTxt->tboSetString(btnMStr);
  tbBtnRTxt->tboSetString(btnRStr);


  btnLStrShdw = btnLStr;
  btnMStrShdw = btnMStr;
  btnRStrShdw = btnRStr;
}


//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskSerial() {


  Serial.print(" OpMd:");
  Serial.print(opMode);
  Serial.print(":");
  Serial.print(opMdStr[opMode]);


//  Serial.print(" OAT:");
//  Serial.print(OATChan.getDispStr());

  Serial.print(" SvoIn:");
  Serial.print(String(SvoInVar));
//  Serial.print(String(SvoInChan.ioEngVal));
  Serial.print(":");
  Serial.print(SvoInChan.getDispStr());

  Serial.print(" SvoOut:");
  Serial.print(String(SvoOut1Chan.ioEngVal));
  Serial.print(":");
  Serial.print(SvoOut1Chan.getDispStr());

  Serial.print(" SvoDone:");
  Serial.print(SvoDone);

  Serial.print(" NWSPos:");
  Serial.print(tboSvoNWSPos->tboX);

  Serial.print(" SvoOutMaster:");
  Serial.print(SvoOutMaster);

  Serial.print(" svoA:");
  Serial.print(svo_ActionStr[svoAction]);

  Serial.print(" SvoOutNWS:");
  Serial.print(SvoOutNWS);

  Serial.print(" SvoOutNGR:");
  Serial.print(SvoOutNGR);

  Serial.print(" SvoOutLR:");
  Serial.print(SvoOutLR);

  Serial.print(" SvoOutRR:");
  Serial.print(SvoOutRR);




//  Serial.print(" DY:");
//  Serial.print(tboSvoPos->tboDy);

//  Serial.print(" WMx:");
//  Serial.print(tboSvoPos->tboWMax);

//  Serial.print(" tboString:");
//  Serial.print(tboCH7->tboString);

//  Serial.print(" tboW:");
//  Serial.print(tboCH7->tboW);
//
//  Serial.print(" tboWPad:");
//  Serial.print(tboCH7->tboWPad);
//
//  Serial.print(" tboCharW:");
//  Serial.print(tboCH7->tboCharW);
//
//  Serial.print(" tboCharCount:");
//  Serial.print(tboCH7->tboCharCount);

//  Serial.print(" btnLStr:");
//  Serial.print(btnLStr);
//
//  Serial.print(" btnMStr:");
//  Serial.print(btnMStr);
//
//  Serial.print(" btnRStr:");
//  Serial.print(btnRStr);

//  Serial.print(" SvoAct:");
//  Serial.print(svo_ActionStr[svoAction]);


  Serial.print("\n");

}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskBtn() {
  //---Left Button---
  if(btnLVar > BTN_THRES){
    btnLStr = "OFF";
    btnL = 0;
  }
  else {
    btnLStr = "ON";
    btnL = 1;
  }

  //---Middle Button---
  if(btnMVar > BTN_THRES) {
    btnMStr = "OFF";
    btnM = 0;

  }
  else {
    btnMStr = "ON";
    btnM = 1;

  }


  //---Right Button---
  if(btnRVar > BTN_THRES){
    btnRStr = "OFF";
    btnR = 0;

  }
  else {
    btnRStr = "ON";
    btnR = 1;

  }
}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskServo(void) {
  switch(svoAction) {
    default:
    case SVO_ACT_NA:

      break;

    case SVO_ACT_INC_IDLE:

      break;

    case SVO_ACT_DEC_R:

      break;

    case SVO_ACT_CENT:

      break;

    case SVO_ACT_INC_L:

      break;


    case SVO_ACT_R_MAX:

      break;

    case SVO_ACT_L_MIN:

      break;

    case SVO_ACT_ANALOG_SLAVE:

      break;

    case SVO_ACT_RETRACT_UP:

      break;

    case SVO_ACT_RETRACT_DOWN:

      break;
  }
}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void handleIncBtn(void) {
  //---Right button decrements servo (towards 0)
  if(!btnL && btnR) {
    SvoOutMaster -= 10;
    svoAction = SVO_ACT_DEC_R;
  }
  //---Left button increments servo (towards 1180)
  else if(btnL && !btnR) {
    SvoOutMaster += 10;
    svoAction = SVO_ACT_INC_L;
  }
  else if(btnL && btnR) {
    SvoOutMaster = SERVO_POS_MID;
    svoAction = SVO_ACT_CENT;
  }
  else {
    //---Both buttons off: do nothing---
    svoAction = SVO_ACT_INC_IDLE;
  }
}

//-----------------------------------------------------------------tempInc
//
//-----------------------------------------------------------------
void handleRetractBtn(void) {
  //---Right button: transit to retract min position
  if(!btnL && btnR ) {
    if(SvoOutMaster < RETRACT_MAX)
      svoAction = SVO_ACT_RETRACT_TO_DWN;
    else
      svoAction = SVO_ACT_RETRACT_IDLE_DWN;
  }
  //---Left button transit to retract max position
  else if(btnL && !btnR) {
    if(SvoOutMaster > RETRACT_MIN)
      svoAction = SVO_ACT_RETRACT_TO_UP;
    else
      svoAction = SVO_ACT_RETRACT_IDLE_UP;
  }
  //---Both buttons, go to center: effective idle position for retracts
  else if(btnL && btnR) {
    if(SvoOutMaster < (RETRACT_CENT - 100) || SvoOutMaster > (RETRACT_CENT + 100)) {
      svoAction = SVO_ACT_RETRACT_TO_CENTER;
    }
    else
      svoAction = SVO_ACT_RETRACT_IDLE_CENT;
  }
  else {
    //---Both buttons off: do nothing---
  }


}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskRetract(void) {

  switch(svoAction) {
    default:
    case SVO_ACT_RETRACT_IDLE:
    case SVO_ACT_RETRACT_IDLE_UP:
    case SVO_ACT_RETRACT_IDLE_DWN:
    case SVO_ACT_RETRACT_IDLE_CENT:
    case SVO_ACT_RETRACT_UP:
    case SVO_ACT_RETRACT_DOWN:
    case SVO_ACT_RETRACT_CENT:
      break;

    case SVO_ACT_RETRACT_TO_CENTER:
      //---Trans is done---
      SvoDone = taskSvoPosTrans(&SvoOutMaster, (int)RETRACT_CENT, (int)10);
      if(SvoDone) {
        SvoOutMaster = RETRACT_CENT;
        svoAction = SVO_ACT_RETRACT_CENT;
      }
      break;


    case SVO_ACT_RETRACT_TO_DWN:
      //---Trans is done---
      SvoDone = taskSvoPosTrans(&SvoOutMaster, (int)RETRACT_MAX, (int)10);
      if(SvoDone) {
        SvoOutMaster = RETRACT_MAX;
        svoAction = SVO_ACT_RETRACT_DOWN;
      }
      break;

    case SVO_ACT_RETRACT_TO_UP:
      //---Trans is done---
      SvoDone = taskSvoPosTrans(&SvoOutMaster, (int)RETRACT_MIN, (int)10);
      if(SvoDone) {
        SvoOutMaster = RETRACT_MIN;
        svoAction = SVO_ACT_RETRACT_UP;
      }
      break;

  }

}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void distSvoCmd(void) {
  SvoOut0Var = SvoOutMaster;
  SvoOut1Var = SvoOutMaster;
  SvoOut2Var = SvoOutMaster;
  SvoOut3Var = SvoOutMaster;
  SvoOut4Var = SvoOutMaster;
  SvoOut5Var = SvoOutMaster;
  SvoOut6Var = SvoOutMaster;
  SvoOut7Var = SvoOutMaster;

  tboSvoPos->tboX = SvoOutMaster/10;
  tboSvoNWSPos->tboX = SvoOutMaster/10;

}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void distRetractCmd(void) {
  SvoOut0Var = SvoOutMaster; //===Servo port 6 (from right)
  SvoOut1Var = SvoOutMaster; //===Servo port 1 (from right)
  SvoOut2Var = SvoOutMaster; //===Servo port 2 (from right)
//  SvoOut3Var = SERVO_POS_MID; //===Not connected: TODO: Assign diff pin to this channel
  SvoOut4Var = SvoOutMaster; //===Servo port 4 (from right)
  SvoOut5Var = SvoOutMaster; //===Servo port 5 (Right most)
//  SvoOut6Var = SERVO_POS_MID; //===Not connected: TODO: Assign diff pin to this channel
//  SvoOut7Var = SERVO_POS_MID; //===Not connected: TODO: Assign diff pin to this channel

  tboSvoPos->tboX = SvoOutMaster/10;
  tboSvoNWSPos->tboX = SvoOutMaster/10;
}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void distNGRSvoCmd(void) {
  SvoOutLR = SvoOutMaster;
  SvoOutRR = SvoOutMaster;
  SvoOutNGR = SvoOutMaster;

  SvoOutNWS = SvoInVar;

  SvoOut0Var = SvoOutNWS;
  SvoOut1Var = SvoOutNWS;
  SvoOut2Var = SvoOutNWS;
  SvoOut3Var = SvoOutNWS;
  SvoOut4Var = SvoOutNWS;
  SvoOut5Var = SvoOutNWS;
//  SvoOut1Var = SvoOutRR;
//  SvoOut2Var = SERVO_POS_MID;
//  SvoOut3Var = SERVO_POS_MID;
//  SvoOut4Var = SvoOutLR;
//  SvoOut5Var = SvoOutNGR;
//  SvoOut6Var = SERVO_POS_MID;
//  SvoOut7Var = SERVO_POS_MID;

  tboSvoPos->tboX = SvoInVar/10;
  tboSvoNWSPos->tboX = SvoOutMaster/10;
//  tboSvoNGRPos->tboX = SvoInVar/10;
//  tboSvoLRPos->tboX = SvoOutLR/10;
//  tboSvoRRPos->tboX = SvoOutRR/10;
}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void handleModeBtn(void) {
  static char btnMShadow = 0;
  if(btnM != btnMShadow && !btnM)
    opMode++;

//  Serial.print(" handleModeBtn:");
  if(opMode > (NUM_OP_MODES - 1))
    opMode = OP_MD_ANALOG_PAN;

  btnMShadow = btnM;

}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskOpMode(void) {
  static int icount = 0;

  switch(opMode) {
    default:
    case OP_MD_NA:
      //--Go directly to BOOT mode---
      opMode = OP_MD_BOOT;
      svoAction = SVO_ACT_NA;
      break;

    case OP_MD_BOOT:
      //---Stay in BOOT mode for 100 ticks---
      if(icount < 5)
        opMode = OP_MD_BOOT;
      else
        opMode = OP_MD_PWRUP;
      svoAction = SVO_ACT_NA;
      break;

    case OP_MD_PWRUP:
      //---Stay in PWRUP mode until it's done---
      if(taskPowerUp() == PWRUP_DONE)
        opMode = OP_MD_ANALOG_PAN;
      else
        opMode = OP_MD_PWRUP;
      handleModeBtn();
      svoAction = SVO_ACT_PWRUP;
      break;

    case OP_MD_ANALOG_PAN:
      tboSvoPos->tboEnabled = true;
      tboSvoNWSPos->tboEnabled = false;

      SvoOutMaster = SvoInVar;
      handleModeBtn();
      distSvoCmd();
      svoAction = SVO_ACT_ANALOG_SLAVE;
      break;

    case OP_MD_INC_TO_ANALOG_TRANS:
      distSvoCmd();
//      handleModeBtn();
      opMode = OP_MD_ANALOG_TO_INC_TRANS;
      break;

    case OP_MD_ANALOG_TO_INC_TRANS:
      distSvoCmd();
//      handleModeBtn();
      opMode = OP_MD_INC_PAN;
      break;

    case OP_MD_INC_PAN:
      handleIncBtn();
      handleModeBtn();
      distSvoCmd();
      break;

    case OP_MD_COMPASS_TRACK:
//      SvoOutMaster = hdgVarS * 10;
      svoAction = SVO_ACT_NA;
      handleModeBtn();
      break;

    case OP_MD_RETRACT:
//      SvoOutMaster = SvoInVar;
      tboSvoPos->tboEnabled = true;
      tboSvoNWSPos->tboEnabled = true;
      handleRetractBtn();
      taskRetract();
      handleModeBtn();
      distRetractCmd();
      break;

    case OP_MD_NGR:
//      SvoOutMaster = SvoInVar;
      tboSvoPos->tboEnabled = true;
      tboSvoNWSPos->tboEnabled = true;
      handleRetractBtn();
      taskRetract();
      handleModeBtn();
      distNGRSvoCmd();
      break;

  }

  icount++;
}

//-----------------------------------------------------------------
// FUNCTION: taskSvoPosTrans
// DESCRIPTION: Provides smooth transition between to servo positions
// ARGS:
//      curPosRef - pointer to current servo position in deg
//      endPos - ending position in deg
//      delta - servo speed in deg per cycle
//      dur - duration of transfer in cycles - not implemented
//      TODO: Normalize cycles to a fixed amount of time in seconds
//-----------------------------------------------------------------
int taskSvoPosTrans(long *curPosPtr, int endPos, int delta) {
  int done = 0;
  static long currPos = 0;
  static int dist = 0;
  static int dir = 0;

  currPos = *curPosPtr;
  if(delta <= 0)
    delta = 1;

  //Calc the servo transition distance in deg * 10
  dist = endPos - currPos;
  Serial.print(" dist:");
  Serial.print(dist);

  SvoErr = dist;

  //---Neg distance, decrement---
  if(dist < 0)
    dir = -1;
  //---Pos distance, increment---
  else if(dist > 0)
    dir = 1;
  //---Zero distance, we're done---
  else {
    dir = 0;
  }
  Serial.print(" dir:");
  Serial.print(dir);

  if(abs(dist) < 15)
    done = 1;
  else
    done = 0;

  if(!done)
    currPos += dir * delta;
  //---Return -1 when done---
  else {
    dist = 0;
    dir = 0;
  }
  *curPosPtr = currPos;
  Serial.print(" currPos:");
  Serial.print(currPos);
  return done;
}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskManager(void) {

  static int icount = 0;
  static int taskIdx = 0;


  icount++;
//  Serial.print("taskManager1");

  switch(taskIdx) {

    default:
      break;

    case TASK_STAT_LED:
      taskStatLED();
      break;

    case TASK_SAWTOOTH:

      break;

    case TASK_VOLTAGE_MON:

      break;

    //---Analog to momentary switch
    case TASK_BTN_READ:
      //---TODO: generalize to pass button structure/class as arg---
      taskBtn();

      break;

    //---Op mode state engine task---
    case TASK_OP_MODE:
      taskOpMode();
      break;

    case TASK_SERVO:
      taskServo();
      break;

    case TASK_SERIAL:
      taskSerial();
      break;

    case TASK_TFT:
      taskTFT();
      taskIdx = NUM_TASKS;
      break;
  }

  taskIdx++;
  if(taskIdx > NUM_TASKS)
    taskIdx = 0;

}


/*-----------------------------------------------------------------------------
 *
 *
 *
 *---------------------------------------------------------------------------*/
void loop(void) {
  //  OATChan.procInChan();
  //  Serial.print("loop1");
  SvoInChan.procInChan();
  btnLChan.procInChan();
  btnMChan.procInChan();
  btnRChan.procInChan();

  //  Serial.print("loop2");
  taskManager();

  //  Serial.print("loop3");
  SvoOut0Chan.procOutChan();
  SvoOut1Chan.procOutChan();
  SvoOut2Chan.procOutChan();
  SvoOut3Chan.procOutChan();
  SvoOut4Chan.procOutChan();
  SvoOut5Chan.procOutChan();
  SvoOut6Chan.procOutChan();
  SvoOut7Chan.procOutChan();

  //  Serial.print("loop4");
  delay(5);
}
