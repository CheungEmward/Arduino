# Arduino
Arduino programme with voltage detection and serial communication 

#include <Servo.h>
#include <Wire.h>

Servo right_drive;
Servo left_drive;

//-----引脚定义-----//
int rPWM_Out = 9;
int lPWM_Out = 10;
int Signal_Change_Info = 2;
//int Signal_Change_Info = 1;

//-----模式判断用变量-----//
int Signal_mode = 1;//1,控制板控制螺旋桨；0，则遥控器控制螺旋桨

//-----字符缓存-----//
String comdata;
char rg_Info, rg1, rg2, rg3, rg4;
char rr_Info, rr1, rr2, rr3;

//-----推力计算相关变量声明-----//
double rec_power, rec_rudder;
double rec_lPower = 0;
double rec_rPower = 0;
int rec_lSignal = 0;
int rec_rSignal = 0;
int lSignal_Store = 1500;//上一循环时刻高电平时长记录
int rSignal_Store = 1500;
int lSignal_Change; //当前循环中高电平时长改变值
int rSignal_Change;
int Det_Tick = 10;  //每次循环高电平时长改变值

//-----船型参数给定-----//
double B = 0.4;
double Lcg = 0.4;

//-----推力与信号对应曲线斜率-----//
double k1 = 0.1; //k1为右桨正转推力与信号关系曲线斜率
double k2 = 0.1; //k2为右桨反转推力与信号关系曲线斜率
double k3 = 0.1; //k3为左桨正转推力与信号关系曲线斜率
double k4 = 0.1; //k4为左桨反转推力与信号关系曲线斜率

//-----函数声明-----//
void Mode_Identify();

void setup() 
{
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  right_drive.attach(rPWM_Out);
  left_drive.attach(lPWM_Out);
  right_drive.writeMicroseconds(1500);//高电平时长设置为中值
  left_drive.writeMicroseconds(1500); //高电平时长设置为中值
  Serial.begin(9600);
  //检测：初始化2引脚为高电平//
  digitalWrite(Signal_Change_Info, HIGH);
  //pinMode(rPWM_Out,OUTPUT);//
  //pinMode(lPWM_Out,OUTPUT)//

}

void loop() {
  //-----控制模式判断-----//
//  Mode_Identify();

  if (Signal_mode == 1)
  {
    //-----串口信息接收-----//
    while (Serial.available() > 0)
    {
      delay(10);
      comdata += char(Serial.read());
//      Serial.println(comdata);
//      Serial.println(comdata.length());
    }
    
    //-----串口信息解析-----//
    if ( comdata.length() > 0 )
    {
//      Serial.println("250250250");
     if ((int(char(comdata.charAt(0))) == 35) && (int(char(comdata.charAt(1))) == 67) && (int(char(comdata.charAt(2))) == 65) && (int(char(comdata.charAt(21))) == 42))
      {

        //Serial.println('1');//
        rg_Info = char(comdata.charAt(3));
//        Serial.println(rg1);
        rg1 = char(comdata.charAt(4));
        rg2 = char(comdata.charAt(5));
        rg3 = char(comdata.charAt(6));
        rg4 = char(comdata.charAt(7));
//        Serial.println(comdata.charAt(4));
//        Serial.println(comdata.charAt(5));
//        Serial.println(comdata.charAt(6));
//        Serial.println(comdata.charAt(7));
        if (rg_Info == '0')
        {
          rec_power = -((int(rg1) - 48) * 100 + (int(rg2) - 48) * 10 + (int(rg3) - 48) + (int(rg4) - 48)*0.1);
        }
        else if (rg_Info == '1')
        {
          rec_power = (int(rg1) - 48) * 100 + (int(rg2) - 48) * 10 + (int(rg3) - 48) + (int(rg4) - 48)*0.1;
        }
        rec_power = constrain(rec_power, -100.0, 100.0);
//        Serial.println(rec_power);

        rr_Info = char(comdata.charAt(13));
        rr1 = char(comdata.charAt(14));
        rr2 = char(comdata.charAt(15));
        rr3 = char(comdata.charAt(16));
        if (rr_Info == '0')
        rec_rudder = -((int(rr1) - 48) * 10 + (int(rr2) - 48) + (int(rr3) - 48)*0.1);
        else if (rr_Info == '1')
        rec_rudder = (int(rr1) - 48) * 10 + (int(rr2) - 48) + (int(rr3) - 48)*0.1;
        rec_rudder = constrain(rec_rudder, -30.0, 30.0);
//        Serial.println(rec_rudder);
     }
   }
//    else
//    {
//      Serial.print("123454989");
//    }

    //-----动力分配-----//
    rec_rPower = (rec_power * cos(rec_rudder) + 2 * rec_power * sin(rec_rudder) * Lcg / B) / 2;
    rec_lPower = (rec_power * cos(rec_rudder) - 2 * rec_power * sin(rec_rudder) * Lcg / B) / 2;


//    Serial.print("rec_rPower = ");
//    Serial.println(rec_rPower);
//    Serial .print("rec_lPower = ");
//    Serial.println(rec_lPower);

    //-----推力与信号映射-----//
    if (rec_rPower > 0)//右桨
    {
      rec_rSignal = 1500 + (int)rec_rPower / k1; //k1为右桨正转推力与信号关系曲线斜率
    }
    else if (rec_rPower < 0)
    {
      rec_rSignal = 1500 + (int)rec_rPower / k2; //k2为右桨反转推力与信号关系曲线斜率
    }
    else
    {
      rec_rSignal = 1500;
    }
    rec_rSignal = constrain(rec_rSignal, 1000, 2000);
//    Serial.print("rec_rSignal = ");
//    Serial.println(rec_rSignal);
    


    if (rec_lPower > 0)//左桨
    {
      rec_lSignal = 1500 + (int)rec_lPower / k3; //k3为右桨正转推力与信号关系曲线斜率
    }
    else if (rec_lPower < 0)
    {
      rec_lSignal = 1500 + (int)rec_lPower / k4; //k4为右桨反转推力与信号关系曲线斜率
    }
    else
    {
      rec_lSignal = 1500;
    }
    rec_lSignal = constrain(rec_lSignal, 1000, 2000);
//    Serial.print("rec_lSignal = ");
//    Serial.println(rec_lSignal);
    

    
    //-----当前推力与期望推力差值计算-----//
    lSignal_Change = rec_lSignal - lSignal_Store;
    rSignal_Change = rec_rSignal - rSignal_Store;

//    //-----左桨推进高电平梯度增加-----//
    int Output_lSignal;
//    if (lSignal_Change > 0)
//    {
//      Output_lSignal = lSignal_Store + Det_Tick;
//      Output_lSignal = constrain(Output_lSignal, lSignal_Store, rec_lSignal);
//    }
//    else if (lSignal_Change < 0)
//    {
//      Output_lSignal = lSignal_Store - Det_Tick;
//      Output_lSignal = constrain(Output_lSignal, rec_lSignal, lSignal_Store);
//    }
//    else
      Output_lSignal = rec_lSignal;

    Output_lSignal = constrain(Output_lSignal, 1000, 2000);
    lSignal_Store = Output_lSignal;
    left_drive.writeMicroseconds(Output_lSignal);

//    Serial.print("Output_lSignal = ");
//    Serial.println(Output_lSignal);

    //-----右桨推进高电平梯度增加-----//
    int Output_rSignal;
//    if (rSignal_Change > 0)
//    {
//      Output_rSignal = rSignal_Store + Det_Tick;
//      Output_rSignal = constrain(Output_rSignal, rSignal_Store, rec_rSignal);
//    }
//    else if (rSignal_Change < 0)
//    {
//      Output_rSignal = rSignal_Store - Det_Tick;
//      Output_rSignal = constrain(Output_rSignal, rec_rSignal, rSignal_Store);
//    }
//    else
      Output_rSignal = rec_rSignal;

    Output_rSignal = constrain(Output_rSignal, 1000, 2000);
    rSignal_Store = Output_rSignal;
    right_drive.writeMicroseconds(Output_rSignal);

//    Serial.print("Output_rSignal = ");
//    Serial.println(Output_rSignal);

    //------电压检测模块------//
    int val1;
    int val2;
    int val11;
    int val22;
    val1 = analogRead(A2);
//    Serial.print("val1 = ");
//    Serial.print(val1);
//    Serial.print("Power supply voltage = ");
    val11 = int(val1 / 0.464333333);
    val11 = constrain(val11, 0, 9999);
//    Serial.print(val11);
//    Serial.println("V");

    val2 = analogRead(A3);
//    Serial.print("Control supply voltage = ");
    val22 = int( val2 / 0.464333333);
    val22 = constrain(val22, 0, 9999);
//    Serial.print(val22);
//    Serial.println("V");

    String ling = "0000";
    String leftvoltage;
    String rightvoltage;
    String leftfinalvoltage;
    String rightfinalvoltage;
    leftvoltage = ling + String(val11);
    rightvoltage = ling + String(val22);
    leftfinalvoltage = leftvoltage.substring(leftvoltage.length()-4, leftvoltage.length());
    rightfinalvoltage = rightvoltage.substring(rightvoltage.length()-4, rightvoltage.length());
    
    
    //------给航空板的数值------//
    String Info_send;
    Info_send = "#AC" + String(Output_rSignal) + String(Output_lSignal) + String(leftfinalvoltage) + String(rightfinalvoltage) + String(Signal_mode) + "*";
//    Info_send = "#AC" + String(rg1) + String(rg2) + String(rg3) + String(rg4) + String(rr_Info) + String(rr1) + String(rr2) + String(rr3) + String(leftfinalvoltage) + String(rightfinalvoltage) + String(Signal_mode) + "*";
    delay(100);
    Serial.println(Info_send);
      //-----清空缓存字符串-----//
    comdata = "";
  }

  //-----清空缓存字符串-----//
  comdata = "";
}

void Mode_Identify()//判断最终的螺旋桨输入信号的来源
{

  if (digitalRead(Signal_Change_Info) == HIGH)
    Signal_mode = 0;
  else
    Signal_mode = 1;

}
