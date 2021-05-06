#include <Arduino.h>
#include <math.h>

#define LedPin 2
#define NUM_READS 20    // Number of sensor reads for filtering
int filtro[NUM_READS];

const long knownResistor = 4625;  // Constant value of known resistor in Ohms _Kpa)ohm _Kpa)ohm 
const long default_TempC=24;
const long open_resistance=-100; //check the open resistance value by replacing sensor with an open and replace the value here...this value might vary slightly with circuit components 
const long short_resistance=0; // similarly check short resistance by shorting the sensor terminals and replace the value here.
const long short_CB=0;
const long open_CB=199;

String IncomingString;

unsigned long lastTrans;

const int interval = 1000;

int CB;
int resistencia;


int activeDigitalPin = 6;         // 6 or 7 interchangeably
int supplyVoltageAnalogPin;       // 6-ON: A0, 7-ON: A1
int sensorVoltageAnalogPin;       // 6-ON: A1, 7-ON: A0

int supplyVoltage;                // Measured supply voltage
int sensorVoltage;                // Measured sensor voltage

        // Calculated moisture percentages and resistances to be sorted and filtered
                        // Simple index variable

void setupCurrentPath() {
  if ( activeDigitalPin == 6 ) {
    activeDigitalPin = 7;
    supplyVoltageAnalogPin = A1;
    sensorVoltageAnalogPin = A0;
  }
  else {
    activeDigitalPin = 6;
    supplyVoltageAnalogPin = A0;
    sensorVoltageAnalogPin = A1;
  }
}

// Selection sort algorithm
void filtrarDatos() {
  int j;
  int temporal;
  for(int i=0; i<NUM_READS-1; i++)
    for(j=i+1; j<NUM_READS; j++)
      if ( filtro[i] > filtro[j] ) {
        temporal = filtro[i];
        filtro[i] = filtro[j];
        filtro[j] = temporal;
      }

      resistencia= filtro[NUM_READS/2];
}

void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(115200); 

  pinMode(LedPin, OUTPUT);

  // initialize the digital pin as an output.
  // Pin 6 is sense resistor voltage supply 1
  pinMode(6, OUTPUT);    

  // initialize the digital pin as an output.
  // Pin 7 is sense resistor voltage supply 2
  pinMode(7, OUTPUT);   

  delay(500);   
}

void loop() {



  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter

for(int i = 0; i < NUM_READS; i++)
{
      setupCurrentPath();      // Prepare the digital and analog pin values

    // Read 1 pair of voltage values
    digitalWrite(activeDigitalPin, HIGH);                 // set the voltage supply on
    //delay(3);
    supplyVoltage = analogRead(supplyVoltageAnalogPin);   // read the supply voltage
    sensorVoltage = analogRead(sensorVoltageAnalogPin);   // read the sensor voltage
    digitalWrite(activeDigitalPin, LOW);                  // set the voltage supply off  
    //delay(3); 

    // Calculate resistance and moisture percentage without overshooting 100
    // the 0.5 add-term is used to round to the nearest integer
    // Tip: no need to transform 0-1023 voltage value to 0-5 range, due to following fraction
    filtro[i] = long( float(knownResistor)* ( supplyVoltage - sensorVoltage ) / sensorVoltage);
    //Serial.println(WM1_Resisten_Kpa = ((3.213*(resis/1000))+4.093)/(1-(0.009733*(resis/1000))-(0.01205*tempC));
//  valueOf[i].moisture = min( int( pow( valueOf[i].resistance/331.55 , 1.0/-1.695 ) * 100 + 0.5 ) , 100 );
   
}
//convert WM1 Reading to Centibars or KiloPascal
    // The trailing 00 for numbers are very critical in the formula below eg: 1 is written as 1.00, else code messes math due to int vs double for numbers
    
    filtrarDatos();
    
    if (resistencia>550.00) 
    {
      if(resistencia>8000.00)
      {
      CB=abs(-2.246-5.239*(resistencia/1000.00)*(1+.018*(default_TempC-24.00))-.06756*(resistencia/1000.00)*(resistencia/1000.00)*((1.00+0.018*(default_TempC-24.00))*(1.00+0.018*(default_TempC-24.00)))); 
      //Serial.print("WM1 >8000 \n");
      } 
     if (resistencia>1000.00) 
      {
        if (resistencia<8000)
        {
      CB=abs((-3.213*(resistencia/1000.00)-4.093)/(1-0.009733*(resistencia/1000.00)-0.01205*(default_TempC))) ;
    // Serial.print("WM1 >1000 \n");
        }
      }
      if (resistencia<1000.00)
        {
        if (resistencia>550.00)
        {
        CB=abs(-20.00*((resistencia/1000.00)*(1.00+0.018*(default_TempC-24.00))-0.55));
       // Serial.print("WM1>550 \n");
        }
        }
     }
    if(resistencia<550.00)
      {
        if(resistencia>300.00)
        {
        CB=0.00;
       // Serial.print("550<WM1>0 \n");
        }
        if(resistencia<300.00)
        {
          if(resistencia>=short_resistance)
          {   
            CB=199;
          //Serial.print("Cb demasiado bajos WM1 \n");
          }
         
        }
        if(resistencia<=open_resistance)
        {
        CB=open_CB; //255 is a fault code for open circuit or sensor not present 
       // Serial.print("Sensor WM1 Error\n");
        }
      }
      // Serial.print("Resistencia =  ");
     // Serial.println(resistencia);
     // Serial.print("Kpa =  ");
     // Serial.println(CB);
      delay(1000);   
      
      // ---SendData---

      if(millis()> lastTrans+interval){

        String Dat = (String)CB;

        Serial.println("AT+SEND=0,3,"+Dat);
        digitalWrite(LedPin, HIGH);
        delay(100);
        digitalWrite(LedPin, LOW);
        lastTrans = millis();

      }
      
      
      
      
      
      
      
      
      
      
      
      
      }
  





