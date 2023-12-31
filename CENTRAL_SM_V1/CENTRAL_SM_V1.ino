#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial SerialPort(1); // use UART0
//HardwareSerial Sender(0);
const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;
const int Rdir = 23;
const int Ldir = 32;
const int Rpwm = 19;
const int Lpwm = 33;

const int freq = 8000;
const int Lchannel = 0;
const int Rchannel = 1;
const int channel1 = 2;
const int channel2 = 3;
const int channel3 = 4;
const int resolution = 8;
const int freqsm = 5000;

const int auger_rot_pwm = 27;
const int auger_rot_dir = 12;
const int auger_pwm = 18;
const int auger_dir = 4;


int x = 0, y = 0;
float M = 1.0;
char c;
char data[16] = "000000000000000";
void Stop(void) {
  Serial.println("Stop");
  digitalWrite(Ldir, LOW);
  digitalWrite(Rdir, LOW);
  ledcWrite(Lchannel, 0);
  ledcWrite(Rchannel, 0);
  Serial.write('0');
  digitalWrite(auger_dir, LOW);
  digitalWrite(auger_rot_dir, LOW);
  ledcWrite(channel1, 0);
  ledcWrite(channel2, 0);
}


void MotorCode(int x, int y, int M, int s, int r)
{
  if (r == 1) {
//    Sender.write('A');
//    ESP.restart();
  }
  // STOP
  if (abs(x) < 100 && abs(y) < 100)
  {
    Serial.println("Stop");
    digitalWrite(Ldir, LOW);
    digitalWrite(Rdir, LOW);
    ledcWrite(Lchannel, 0);
    ledcWrite(Rchannel, 0);
  }

  // FORWARD MAX
  else if (abs(x) < 100 && y > 100)
  {
    Serial.println("FM");

    digitalWrite(Ldir, HIGH);
    digitalWrite(Rdir, HIGH);
    int i = map(abs(y) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(y) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // BACKWARD MAX
  else if (abs(x) < 100 && y < 100)
  {
    Serial.println("BM");

    digitalWrite(Ldir, LOW);
    digitalWrite(Rdir, LOW);
    int i = map(abs(y) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(y) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // SPOT LEFT
  else if (x < 100 && abs(y) <= 100)
  {
    Serial.println("SL");

    digitalWrite(Ldir, LOW);
    digitalWrite(Rdir, HIGH);
    int i = map(abs(x) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(x) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // SPOT RIGHT
  else if (x > 100 && abs(y) <= 100)
  {
    Serial.println("SR");

    digitalWrite(Ldir, HIGH);
    digitalWrite(Rdir, LOW);
    int i = map(abs(x) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(x) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // OCTET 1
  else if (x > 100 && y > 100 && x > y)
  {
    Serial.println("O1");

    digitalWrite(Ldir, HIGH);
    digitalWrite(Rdir, LOW);
    int i = map(abs(abs(x) - abs(y)) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(x) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // OCTET 2
  else if (x > 100 && y > 100 && x < y)
  {
    Serial.println("O2");

    digitalWrite(Ldir, HIGH);
    digitalWrite(Rdir, HIGH);
    int i = map(abs(abs(x) - abs(y)) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(y) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // OCTET 3
  else if (x < 100 && y > 100 && abs(x) < y)
  {
    Serial.println("O3");

    digitalWrite(Ldir, HIGH);
    digitalWrite(Rdir, HIGH);
    int i = map(abs(y) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(abs(x) - abs(y)) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // OCTET 4
  else if (x < 100 && y > 100 && abs(x) >= y)
  {
    Serial.println("O4");

    digitalWrite(Ldir, LOW);
    digitalWrite(Rdir, HIGH);
    int i = map(abs(x) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(abs(x) - abs(y)) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }
  // OCTET 5
  else if (x < 100 && y < 100 && abs(x) > abs(y))
  {
    Serial.println("O5");

    digitalWrite(Ldir, LOW);
    digitalWrite(Rdir, HIGH);
    int i = map(abs(x) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(abs(x) - abs(y)) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // OCTET 6
  else if (x < 100 && y < 100 && abs(x) < abs(y))
  {
    Serial.println("O6");

    digitalWrite(Ldir, LOW);
    digitalWrite(Rdir, LOW);
    int i = map(abs(y) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(abs(x) - abs(y)) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // OCTET 7
  else if (x > 100 && y < 100 && abs(x) < abs(y))
  {
    Serial.println("O7");

    digitalWrite(Ldir, LOW);
    digitalWrite(Rdir, LOW);
    int i = map(abs(abs(x) - abs(y)) * (M * 0.1), 100, 1023, 0, 255);
    int j = map(abs(y) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }

  // OCTET 8
  else if (x > 100 && y < 100 && abs(x) > abs(y))
  {
    Serial.println("O8");

    digitalWrite(Ldir, HIGH);
    digitalWrite(Rdir, LOW);
    int i = map(abs(abs(x) - abs(y)), 100, 1023, 0, 255);
    int j = map(abs(x) * (M * 0.1), 100, 1023, 0, 255);
    ledcWrite(Lchannel, i);
    ledcWrite(Rchannel, j);
  }
  switch (s)
  {
    case 0: // safety
//      Sender.write('0');
      Stop();
      //      Serial.println("0");
      break;

    case 1: // safety
//      Sender.write('1');
      Serial.println("1");
      break;

    case 2: // safety
//      Sender.write('2');
      //      Serial.println("2");
      break;

    case 3: // safety
//      Sender.write('3');
      break;

    case 4: // safety
//      Sender.write('4');
      break;

    case 5: // safety
//      Sender.write('5');
      Serial.println("Hey its 500 mo-fkers");
      for (int i = 0; i < 500; i++) {


        digitalWrite(auger_dir, HIGH);
        digitalWrite(auger_rot_dir, HIGH);
        ledcWrite(channel1, 255);
        ledcWrite(channel2, 255);   
         }  
      break;

    case 6: // safety
//      Sender.write('6');
      for (int i = 0; i < 500; i++)
        {
          digitalWrite(auger_dir, LOW);
          digitalWrite(auger_rot_dir, LOW);
          ledcWrite(channel1, 0);
          ledcWrite(channel2, 255);
        }
      break;

    case 7: // safety
//      Sender.write('7');
      for (int i = 0; i < 500; i++)
        {

          digitalWrite(auger_dir, LOW);
          digitalWrite(auger_rot_dir, LOW);
          ledcWrite(channel1, 255);
          ledcWrite(channel2, 0);
        }
      break;

    case 8: // safety
//      Sender.write('8');

      break;

    case 9: // safety
//      Sender.write('9');
      break;

    default:
//      Sender.write('0');
      break;
  }
}

void setup()
{
  Serial.begin(115200);
//  Sender.begin(115200) ;   // Use default serial for debug output
  SerialPort.begin(115200, SERIAL_8N1, 2, 0);//(baud rate,protocol,Tx,Rx)

  ledcSetup(Lchannel, freq, resolution);
  ledcSetup(Rchannel, freq, resolution);
  ledcAttachPin(Lpwm, Lchannel);
  ledcAttachPin(Rpwm, Rchannel);
  pinMode(Ldir, OUTPUT);
  pinMode(Rdir, OUTPUT);
  pinMode(auger_dir, OUTPUT);       // auger actuation direction
  pinMode(auger_rot_dir, OUTPUT);   // auger rotation direction
  ledcSetup(channel1, freqsm, resolution);
  ledcSetup(channel2, freqsm, resolution);
  ledcWrite(channel1, 0);
  ledcWrite(channel2, 0);
  ledcAttachPin(auger_pwm, channel1);       // auger actuation PWM
  ledcAttachPin(auger_rot_pwm, channel2);   // auger rotation PWM
}

void loop()
{
  if (SerialPort.available())
  {
    // read the data into the buffer
    while (SerialPort.available())
    {

      rxBuffer[bufferIndex] = (char)SerialPort.read();
      bufferIndex++;
      // Make sure we don't overflow the buffer
      if (bufferIndex >= BUFFER_SIZE)
        bufferIndex = 0;
    }

    // Find the positions of the "M","X", "Y", "S" ,"R" and "E" characters in the buffer

    char *M_index = strchr(rxBuffer, 'M');
    char *x_index = strchr(rxBuffer, 'X');
    char *y_index = strchr(rxBuffer, 'Y');
    char *S_index = strchr(rxBuffer, 'S');
    char *R_index = strchr(rxBuffer, 'R');
    char *E_index = strchr(rxBuffer, 'E');
    
    // Print the received packet in character format
    Serial.print("Received Packet: ");
    for (int i = 0; i < bufferIndex; i++) {
      Serial.print(rxBuffer[i]);
    }
    Serial.println(); // Print a newline to separate packets
    
    if (M_index != NULL && x_index != NULL && y_index != NULL && S_index != NULL && R_index != NULL && E_index != NULL)
    {
      // Extract the values from the packet
      char m = *(M_index + 1);
      int M = m - '0';
      int x = atoi(x_index + 1);
      int y = atoi(y_index + 1);
      int s = atoi(S_index + 1);
      int r = atoi(R_index + 1);
      MotorCode(x, y, M, s, r);
      delay(10);
    }
    else
    {
      Serial.println("Invalid Packet received");
    }
  }
  else
  {
    Stop();
  }

  bufferIndex = 0;
}
