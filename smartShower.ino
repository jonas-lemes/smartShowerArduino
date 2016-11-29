
#include <OneWire.h>
#include <DallasTemperature.h>  
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#define pSensor A1
#define pControle 3

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);

float tempMin = 999;
float tempMax = 0;
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1;

double setPoint = 0;
long lastProcess =0;
int controlePwm = 0; //desligado
double erro = 0; // valor do erro
double PID;
double lastTemperatura;
//double temperatura;
double P=0,I=0,D=0;
double kp=3.55,ki=128,kd = -0.007;

LiquidCrystal_I2C lcd(0x20,16,2);  

byte statusLed    = 13;
byte sensorInterrupt = 0;  
byte sensorPin       = 2;


float calibrationFactor = 4.5;

volatile byte pulseCount;  

int pwmSum;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

unsigned long oldTime;
boolean enviaDado = false;

const int potenciometer = 0;
int maped_value;
int valuePotenciometer = 0;

void setup()
{

  Serial.begin(9600);
  
  sensors.begin(); 
   if (!sensors.getAddress(sensor1, 0)) 
   Serial.println("Sensor nao encontrados !");
    
  lcd.init();                 
  lcd.backlight();           
  lcd.print("Ligue o chuveiro..."); 
  
  
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;

  
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    pinMode(pSensor, INPUT);
    pinMode(pControle, OUTPUT);
}

/**
 * Main program loop
 */
void loop(){
     
     if((millis() - oldTime) > 1000){  
      pwmSum = 0;    
      pulseCount= 0;
      flowRate= 0.0;
      flowMilliLitres= 0;
      totalMilliLitres= 0;
      oldTime= 0;
      flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
      
      if(flowRate>0){ 
        
      valuePotenciometer = analogRead(potenciometer);
      maped_value = map(valuePotenciometer,0,1023,20,40);

      lcd.clear();
      lcd.print(String("Set point: ") + String(maped_value)); 
      lcd.setCursor(0, 1); 
      sensors.requestTemperatures();
      float tempC = sensors.getTempC(sensor1);
      lcd.print(String("Temp.: ") + String(tempC)+ (char)223 + String("C")); 
      //PID
      setPoint = maped_value;
      erro = setPoint - tempC;
      float deltaTime = (millis() - lastProcess)/1000.0;//tf=0.0151
      lastProcess= millis();
      // float deltaTime = millis();
      //implementando o P
      P = erro*kp;
      
      //implementando I
      if(P > 255){
        P = 255;
      }
      
      if( P < -255){
        P = -255;
      }
      
      I += (erro*ki)*deltaTime;
      
      //implementando D
      if(I > 255){
        I = 255;
      }

      if( I < -255){
        I = -255;
      }

      D = (lastTemperatura - tempC)*kd/deltaTime;//verificar
   
    if(D > 255){
      D = 255;
    }
    if( D < -255){
      D = -255;
    }
    PID= P+I+D; 
    //converte para Controle
       controlePwm=(10);//verificar 
    // saida do controle
    if(controlePwm > 255){
      controlePwm = 255;
    }
    if(controlePwm < 0){
      controlePwm = 0;
    }
    
    analogWrite(pControle, controlePwm);
    
     pwmSum += controlePwm;
     
    detachInterrupt(sensorInterrupt);     
    oldTime = millis();      
    flowMilliLitres = (flowRate / 60) * 1000; 
    totalMilliLitres += flowMilliLitres;
        
    unsigned int frac;
   
      frac = (flowRate - int(flowRate)) * 10;
    
      pulseCount = 0;
     
      attachInterrupt(sensorInterrupt, pulseCounter, FALLING);      
    
      enviaDado = true;
      }else{
        if(enviaDado){
          
        lcd.clear();        
        lcd.print("Enviando dados");
        lcd.setCursor(0, 1); 
        delay(1000);
        lcd.print(".");
        delay(1000);
        lcd.print(".");
        delay(1000);
        lcd.print("."); 
        delay(1000);
        lcd.print("!"); 
        delay(1000);
        int consumo =0;
        Serial.write("CLA");
        consumo = (pwmSum*5500) / 3600;
        totalMilliLitres;
        enviaDado = false;
        lcd.clear();
        lcd.print("Ligue o chuveiro...");
        }
      }
     }
      
}

void pulseCounter()
{
  
  pulseCount++;
}

