const bool verbose = true;
const int LED_pin = 10;
const int photoresistor_pin = A0;
bool done = false;
String cmd_buffer = "";
double setPoint = 100;
int timestep;
double light_buffer = 0;
int measurement_counter = 0;

class PID_controller {
  private:
    double Kp; double Ki; double Kd;
    double accumulator;
    double prev_e;
    double u_min;
    double u_max;

    double saturation(double u){
      if(u<u_min){
        return u_min;
      }
      if(u>u_max){
        return u_max;
      }
      return u;
    }

    void antiwindup(u){
      // to be wrote
    }
    
  public:
    PID_controller(double h = 1000.0, double kp = 1, double ki = 0, double kd = 0, double minoutput = 0; double maxoutput = 255){
      /* Its expected that the timestep h is given in ms */
      Kp = kp*h/1000.0; Ki = ki*h/1000.0; Kd = kd*h/1000.0;
      accumulator = 0.0;
      prev_e = 0.0;
      u_min = minoutput;
      u_max = maxoutput;
    }
    double control(double setPoint, double y) {
      double e = setPoint - y;
      accumulator += e;
      double u = Kp*e + Ki*accumulator + Kd*(e-prev_e);
      prev_e = e;
      
      u = saturation(u);
      
      if(verbose){
        Serial.print("Accu:");
        Serial.print(accumulator);
        Serial.print(", e:");
        Serial.print(e);
        Serial.print(", u: ");
        Serial.println(u);
      }
      return u;
    }
} PID;


void setup(){
  pinMode(LED_pin, OUTPUT);
  pinMode(photoresistor_pin, INPUT);
  
  setPoint = 100.0;
  timestep = 500; //ms
  PID = PID_controller(timestep,0.5,1,0);
  
  Serial.begin(9600); 
  Serial.println("Arduino setup complete.");
}

void loop(){
  int light = readLight();
  double u = PID.control(setPoint, light);
  setLEDBrightness(u);
  if(verbose){
    Serial.print("u: ");
    Serial.println(light);
  }
  delay(timestep);
}


void setLEDBrightness(int duty){
  analogWrite(LED_pin, duty);
}

int readLight(){
  /*
  double voltage = analogRead(photoresistor_pin)*5.0/1024.0;
  double R = 50/voltage - 10;
  double light = (light_buffer + convertToLux(R)) /2;
  light_buffer = light;
  */
  return map(analogRead(photoresistor_pin),675, 1000, 0, 255);
  
}

double convertToLux(double R){
  return pow((R/60.0), (-1/0.6505));
}

/* 
 *  Serialevent runs automatically at the end of every loop(),
 *  Messages will update setpoint.
 */
void serialEvent(){
  while(Serial.available()){
    if(!done){
      char ch = (char)Serial.read();
      if(ch == '\n') {
        done = true;
      }
      else {
        cmd_buffer += ch;
      }
    }
  }
  setPoint = cmd_buffer.toInt();
  done = false;
  String cmd_buffer = "";
  
  if(verbose){
    Serial.print("Setpoint is now ");
    Serial.println(setPoint);
  }
}


