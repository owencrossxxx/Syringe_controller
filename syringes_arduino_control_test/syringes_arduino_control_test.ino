//To-do
// PID test
//counter
//python script


#define STEPS_PER_REVOLUTION 200*16
#define ISR_FREQ 25000
#define MAX_MOTOR_SPEED_STEPS 25000 // 25000 Steps/s
#define MAX_MOTOR_SPEED 10000
#define MAX_MOTOR_TICKS 1000000;
#define SPEED2TICKS(target_speed)((unsigned int)((float)ISR_FREQ/target_speed))
#define INVERT_DIRECTION false
#define FORWARD (!INVERT_DIRECTION ? HIGH : LOW)
#define BACKWARD (!FORWARD)

//#define PRESSURE_SENSORS ADP5101 42.1316
#define PRESSURE_FROM_RAW(value)((float)value*0.0048828125*42.1316) // TODO: put sensor calibration

#define INIT_COMMUNICATION 1
#define STOP_CMD 255
#define RESTART_CMD 254
#define HOME 253
#define SET_CMD 2
#define TARGET_PRESSURE 3
#define PID_GAINS 4


const unsigned int n_motors = 2;
unsigned long cnt[n_motors];
unsigned long mtr_ticks[n_motors];
bool motor_enabled[n_motors];

float offset_p[n_motors]; // offset pressure
float p_d[n_motors]; // target pressure
float p[n_motors]; // pressure
float sp[n_motors] = {0,0}; // smoothed pressure

int steps[n_motors] = {0, 0};
float e[n_motors] = {0, 0};
float eI[n_motors] = {0, 0};
float e_prev[n_motors] = {0, 0};
float u[n_motors] = {0, 0};
float kp[n_motors] = {700.0, 1.0};
//float ki[n_motors] = {20.0, 0.0};
float ki[n_motors] = {0.0, 0.0};
float kd[n_motors] = {0.0, 0.0};
const float ss_percentage = 5 / 100;
const float dt = 1.0 / 100.0; // Must match the Timer 2 frequency

bool steady_state = true;

bool stop_control_system = true;

bool gohome = false;
bool stopnow = false;



const int stepPins[n_motors] = {3, 5};
const int dirPins[n_motors] = {4, 6};
const int analogPins[n_motors] = {A3, A4};
const int valvePin = 9;
const int buttonPins[n_motors] = {7, 8};

String message, out;

void setup()
{
  // Stop interrupts
  cli();

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  //OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536) // 1hz increments with 1024 prescaler
  OCR1A = 9; // = (16*10^6) / (25000*64) - 1 (must be <65536) // 25 Khz increments with 64 prescaler
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS01) | (1 << CS00); // This should be the 64 prescaler
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 100hz increments
  OCR2A = 156;// = (16*10^6) / (100*1025) - 1 (must be < 256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 1024 prescaler
  TCCR2B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  pinMode(valvePin, OUTPUT);
  
  for (int i = 0; i < n_motors; i++)
  {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    
    cnt[i] = 0;
    mtr_ticks[i] = 10000;
    motor_enabled[i] = true;
    //p_d[i] = p[i]; // set error equal to zero ad the beginning
    //e[i] = e_prev[i] = p_d[i] - p[i];
    e[i] = e_prev[i] = 0;
    delay(1);
  }

  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(1);

  // get offset
  for (int i = 0; i < n_motors; i++) {
    offset_p[i] = analogRead(analogPins[i]) * 0.0048828125 * 42.1316; //measure offset
    //delay(1);
  }

  for (int i = 0; i < n_motors; i++)
  {
    // TODO: use port manipulation
    digitalWrite(dirPins[i], LOW);
  }

  digitalWrite(valvePin, LOW);

  // Enable interrupts
  sei();
}

ISR(TIMER1_COMPA_vect)
{
  if (gohome) {
    for (int i = 0; i < n_motors; i++)
    {
      steps[i] = 0;
    }
  }

  if (stop_control_system)
  {
    for (int i = 0; i < n_motors; i++)
    {
      eI[i] = 0;
      cnt[i] = 0;
    }
    return;
  }

  for (int i = 0; i < n_motors; i++)
  {
    if (!motor_enabled[i])
    {
      eI[i] = 0;
      continue;
    }
    cnt[i]++;
    if (cnt[i] >= mtr_ticks[i])
    {
      // TODO: use port manipulation
      digitalWrite(stepPins[i], HIGH);
      digitalWrite(stepPins[i], LOW);
      cnt[i] = 0;
      if (digitalRead(dirPins[i] == FORWARD)) {
        steps[i]++;
      }
      else {
        steps[i]--;
      }
    }
  }
}


ISR(TIMER2_COMPA_vect)
{
  if (stopnow) {
    stop_control_system = true;
    gohome = false;
    digitalWrite(valvePin, LOW);
  }
  else {
    stop_control_system = false;
    for (int i = 0; i < n_motors; i++)
    {
      //Home check
      if (gohome == true && digitalRead(buttonPins[0]) == 1) {
        digitalWrite(dirPins[i], BACKWARD);
        mtr_ticks[i] = SPEED2TICKS((unsigned int)abs(5000));
        digitalWrite(valvePin, HIGH);
        //Serial.println("valve is on");
        motor_enabled[i] = true;
        //Serial.println("hi");
      }
      else if (gohome == true && digitalRead(buttonPins[0]) == 0) {
        digitalWrite(valvePin, LOW);
        motor_enabled[i] = false;
        gohome = false;
        p_d[i] = 0;
        p[i] = 0;
      }
      else {
        digitalWrite(valvePin, LOW);
        // PID
        // REMARK: put (eventually) analog read here
        e[i] = (p_d[i] - sp[i]);
        //Serial.println(e[0]);
        if (e[i] > 0) {
          digitalWrite(dirPins[i], FORWARD);
          //Serial.println("Forward");
        }
        else {
          digitalWrite(dirPins[i], BACKWARD);
          //Serial.println("Backward");
        }
        eI[i] += e[i] * dt;
        u[i] =
          kp[i] * e[i] // Proportional
          + ki[i] * eI[i] // Integral
          + kd[i] * (e[i] - e_prev[i]) / dt // Derivative TODO: consider to derive p instead of e to avoid spikes in case of sharp transition in the setpoint
          ;
        //if( p_d[i] -  p_d[i]*ss_percentage < p[i] && p[i] > p_d[i] + p_d[i]*ss_percentage )
        if (abs(e[i]) < 1)
        {
          steady_state = true;
          eI[i] = 0;
          mtr_ticks[i] = MAX_MOTOR_TICKS;
          motor_enabled[i] = false;
        }
        else
        {
          steady_state = false;
          if (abs(u[i]) > MAX_MOTOR_SPEED)
            u[i] = MAX_MOTOR_SPEED;
          mtr_ticks[i] = SPEED2TICKS((unsigned int)abs(u[i]));
          motor_enabled[i] = true;
        }

        //Safety stop
        if (digitalRead(dirPins[0]) == BACKWARD && digitalRead(buttonPins[0]) == 0) {
          //mtr_ticks[i] = SPEED2TICKS((unsigned int)abs(0));
          motor_enabled[i] = false;
          //Serial.println("Emergency");
        }
        else {
          motor_enabled[i] = true;
        }
      }
    }
  }
}

void loop()
{
  // REMARK: Analog read is extremely slow and it mess with the motor command
  // if put in the ISR2. The resulting command produces higher noise from the stepper.
  // Leave it here for the moment.

  for (int i = 0; i < n_motors; i++){
    p[i] = analogRead(analogPins[i]) * 0.0048828125 * 42.1316 - offset_p[i];
    sp[i] = 0.1*p[i]+0.9*sp[i];
  }
  //  ////////////////////
  //
  //  //p[1] = p[0]; // TODO: Remove
 


  message = Serial.readStringUntil(';');

  switch (message.toInt())
  {
    case STOP_CMD:
      //      stop_control_system = true;
      //      gohome = false;
      //      digitalWrite(valvePin, LOW);
      stopnow = true;
      return;
    case RESTART_CMD:
      stop_control_system = false;
      gohome = false;
      digitalWrite(valvePin, LOW);
      return;
    case TARGET_PRESSURE:
      stopnow = false;
      message = Serial.readStringUntil(';');
      //Serial.print("Setting Target Pressure to: ");
      //Serial.println(message.toFloat());
      // TODO: Change
      p_d[0] = message.toFloat();
      //p_d[1] = message.toFloat();
      motor_enabled[0] = true;
      //motor_enabled[1] = true;
      stop_control_system = false;
      gohome = false;
      digitalWrite(valvePin, LOW);
      /////
      return;
    case HOME:
      stopnow = false;
      stop_control_system = false;
      gohome = true;
      return;
    default:
      break;
  }



  //out = String(p[0]) + ";" + String(p_d[0]);
    Serial.print(steps[0]);
    Serial.print(";");
    Serial.print(sp[0]);
    Serial.print(";");
    Serial.println(p_d[0]);

    //Serial.println(sp[0]);

}

void split_string(String s, float out[])
{
  ;
}
