#include    <Servo.h>

// `Pino Servo Motor
#define MOTOR_PIN           9
// Pinos PWM
#define PWM1_PIN            5
#define PWM2_PIN            6
// 74HCT595N Pinos Registrador 8 bits
#define SHCP_PIN            2                               // The displacement of the clock
#define EN_PIN              7                               // Habilitação/Sentido
#define DATA_PIN            8                               // Dados Serial
#define STCP_PIN            4                               // Clock        
#define Trig_PIN            12
#define Echo_PIN            13
// Entradas Analogicas Sensores seguidor de faixa
#define LEFT_LINE_TRACJING      A0
#define CENTER_LINE_TRACJING    A1
#define right_LINE_TRACJING     A2

Servo MOTORservo;


//Definir Ponto de destino
const int Dest_X=-1400;
const int Dest_Y=-1000;

int Passo=0;

const int Forward       = 92;                               // Frente
const int Backward      = 163;                              // Ré
const int Turn_Left     = 149;                              // Virar Eesquerda
const int Turn_Right    = 106;                              // Virar p/ Direita
const int Top_Left      = 20;                               // Diagonal Esquerda-Frente
const int Bottom_Left   = 129;                              // Diagonal Esquerda-Ré
const int Top_Right     = 72;                               // Diagonal Direita-Frente
const int Bottom_Right  = 34;                               // Diagonal Direita-Re
const int Stop          = 0;                                // Parado
const int Contrarotate  = 172;                              // Giro Sentido Antihorario
const int Clockwise     = 83;                               // Giro Sentido Horario
const int Moedl1        = 25;                               // model1
const int Moedl2        = 26;                               // model2
const int Moedl3        = 27;                               // model3
const int Moedl4        = 28;                               // model4
const int MotorLeft     = 230;                              // servo virar para esquerda
const int MotorRight    = 231;                              // servo virar para direita

int Left_Tra_Value;
int Center_Tra_Value;
int Right_Tra_Value;
int Black_Line = 400;

int leftDistance = 0;
int middleDistance = 0;
int rightDistance = 0;

byte RX_package[3] = {0};
uint16_t angle = 90;
byte order = Stop;
char model_var = 0;
int UT_distance = 0;

void setup()
{
    Serial.setTimeout(10);
    Serial.begin(115200);

    MOTORservo.attach(MOTOR_PIN);

    pinMode(SHCP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(STCP_PIN, OUTPUT);
    pinMode(PWM1_PIN, OUTPUT);
    pinMode(PWM2_PIN, OUTPUT);

    pinMode(Trig_PIN, OUTPUT);
    pinMode(Echo_PIN, INPUT);

    pinMode(LEFT_LINE_TRACJING, INPUT);
    pinMode(CENTER_LINE_TRACJING, INPUT);
    pinMode(right_LINE_TRACJING, INPUT);

    MOTORservo.write(angle);

}


void loop() {


  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);

    if (commaIndex1 != -1 && commaIndex2 != -1) {
      String value1Str = data.substring(0, commaIndex1);
      String value2Str = data.substring(commaIndex1 + 1, commaIndex2);
      String value3Str = data.substring(commaIndex2 + 1);

      int coordenadaX = value1Str.toInt();
      int coordenadaY = value2Str.toInt();
      int angulo = value3Str.toInt();

      switch (Passo) 
      {
      case 0:
        if ((angulo >=95)||(angulo <=85))
        {
          Motor(Clockwise, 100);
        } else{
          Passo=1;
        }
      case 1:
      //Desloca até o ponto X
        if (coordenadaX>= Dest_X+100)
        {
          Motor(Forward, 100);
        } else if  (coordenadaX>= Dest_X-100){
          Motor(Backward, 100);
          } else
            {
              Passo=2;
            }
      case 2:
      //Gira o carro para direção em Y
      if (coordenadaY<= Dest_Y){
        if ((angulo >=10)&&(angulo <=0))
          {
            Motor(Top_Left, 100);
          } 
          else
          {
            Passo=3;
          }
        }
      if (coordenadaY>= Dest_Y){
        if ((angulo >=10)&&(angulo <=0))
          {
            Motor(Top_Right, 100);
          } 
          else
          {
            Passo=3;
          }
      }
      case 3:
      //Desloca até o ponto X
        if (coordenadaY>= Dest_Y+100)
        {
          Motor(Forward, 100);
        } else if  (coordenadaY>= Dest_Y-100){
          Motor(Backward, 100);
          } else
            {
              Passo=4;
            }
      default:
        Motor(Stop, 0);
      }
      




  }
    
  }
}

void model1_func(byte orders) //Movimentos em manual
{
    switch (orders)
    {
    case Stop:
        Motor(Stop, 0);
        break;
    case Forward:
        Motor(Forward, 250);
        break;
    case Backward:
        Motor(Backward, 250);
        break;
    case Turn_Left:
        Motor(Turn_Left, 250);
        break;
    case Turn_Right:
        Motor(Turn_Right, 250);
        break;
    case Top_Left:
        Motor(Top_Left, 250);
        break;
    case Top_Right:
        Motor(Top_Right, 250);
        break;
    case Bottom_Left:
        Motor(Bottom_Left, 250);
        break;
    case Bottom_Right:
        Motor(Bottom_Right, 250);
        break;
    case Clockwise:
        Motor(Clockwise, 250);
        break;
    case MotorLeft:
        motorleft();
        break;
    case MotorRight:
        motorright();
        break;
    default:
        // Serial.println(".");
        order = 0;
        Motor(Stop, 0);
        break;
    }
}

void model2_func()      // Seguidor de corredor, desvia obstaculo
{
    MOTORservo.write(90);
    UT_distance = SR04(Trig_PIN, Echo_PIN);
    Serial.println(UT_distance);
    middleDistance = UT_distance;

    if (middleDistance <= 25) 
    {
        Motor(Stop, 0);
        for(int i = 0;i < 500;i++){
          delay(1);
          RXpack_func();
          if(model_var != 1)
            return ;
        }
        MOTORservo.write(10);
        for(int i = 0;i < 300;i++){
          delay(1);
          RXpack_func();
          if(model_var != 1)
            return ;
        }
        rightDistance = SR04(Trig_PIN, Echo_PIN);//SR04();
        Serial.print("rightDistance:  ");
        Serial.println(rightDistance);
        MOTORservo.write(90);
        for(int i = 0;i < 300;i++){
          delay(1);
          RXpack_func();
          if(model_var != 1)
            return ;
        }
        MOTORservo.write(170);
        for(int i = 0;i < 300;i++){
          delay(1);
          RXpack_func();
          if(model_var != 1)
            return ;
        }
        leftDistance = SR04(Trig_PIN, Echo_PIN);//SR04();
        Serial.print("leftDistance:  ");
        Serial.println(leftDistance);
        MOTORservo.write(90);
        if((rightDistance < 20) && (leftDistance < 20)){

            Motor(Backward, 180);
            for(int i = 0;i < 1000;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Contrarotate, 250); 
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
        }
        else if(rightDistance < leftDistance) {
            Motor(Stop, 0);
            for(int i = 0;i < 100;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Backward, 180);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Contrarotate, 250);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
        }//turn right
        else if(rightDistance > leftDistance){
            Motor(Stop, 0);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Backward, 180);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Clockwise, 250); 
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
        }
        else{
            Motor(Backward, 180);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Clockwise, 250); 
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
        }
    }
    else 
    {
        Motor(Forward, 250);
    }
}

void model3_func()      // Seguidor fila
{
    MOTORservo.write(90);  
    UT_distance = SR04(Trig_PIN, Echo_PIN);
    Serial.println(UT_distance);
    if (UT_distance < 15)
    {
        Motor(Backward, 200);
    }
    else if (15 <= UT_distance && UT_distance <= 20)
    {
        Motor(Stop, 0);
    }
    else if (20 <= UT_distance && UT_distance <= 25)
    {
        Motor(Forward, 180);
    }
    else if (25 <= UT_distance && UT_distance <= 50)
    {
        Motor(Forward, 220);
    }
    else
    {
        Motor(Stop, 0);
    }
}

void model4_func()      // Seguidor de trilha
{
    MOTORservo.write(90);
    Left_Tra_Value = analogRead(LEFT_LINE_TRACJING);
    Center_Tra_Value = analogRead(CENTER_LINE_TRACJING);
    Right_Tra_Value = analogRead(right_LINE_TRACJING);
    if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Forward, 250);
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Contrarotate, 220);
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Contrarotate, 250);
    }
    else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Clockwise, 250);
    }
    else if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Clockwise, 220);
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Stop, 0);
    }
}
void motorleft()  //servo
{
    MOTORservo.write(angle);
    angle+=1;
    if(angle >= 180) angle = 180;
    delay(10);
}
void motorright() //servo
{
    MOTORservo.write(angle);
    angle-=1;
    if(angle <= 1) angle = 1;
    delay(10);
}

void Motor(int Dir, int Speed)      // motor drive
{
    digitalWrite(EN_PIN, LOW);
    analogWrite(PWM1_PIN, Speed);
    analogWrite(PWM2_PIN, Speed);

    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
    digitalWrite(STCP_PIN, HIGH);
}

float SR04(int Trig, int Echo)      // Medicao sensor ultrasonico
{
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    float distance = pulseIn(Echo, HIGH) / 58.00;
    delay(10);
    
    return distance;
}

void RXpack_func()  //Receive data
{
    if(Serial.available() > 0)
    {
        delay(1);                                           // delay 1MS
        if(Serial.readBytes(RX_package, 6))
        {
            if (RX_package[0] == 0xA5 && RX_package[2] == 0x5A)     // Verifica Inicio e fim do pacote
            {
                order = RX_package[1];
                if(order == Moedl1) 
                {
                    model_var = 0;
                }
                else if (order == Moedl2)
                {
                    model_var = 1;
                }
                else if (order == Moedl3)
                {
                    model_var = 2;
                }
                else if (order == Moedl4)
                {
                    model_var = 3;
                }

            }
        }
    }
}
