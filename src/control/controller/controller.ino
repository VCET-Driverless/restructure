//sudo chmod a+rw /dev/ttyACM0
#define DIR_PIN          3
#define STEP_PIN         2       // MOTOR PINS
#define ENABLE_PIN       4

#define ADIR_PIN          5//8
#define ASTEP_PIN         6//9      //ACTUATOR
#define AENABLE_PIN       7//10

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

#define G 5.18*3.5
#define R -5/9

void serialFlush(){
  while(Serial.available() > 0){
    char t = Serial.read();
  }
}

void (* resetFunc) (void) = 0;

//#define PIN_SWITCH_1     13
int t;
// RemoteXY select connection mode and include library
/* #define REMOTEXY_MODE__ESP8266_HARDSERIAL_POINT

  #include <RemoteXY.h>

  // RemoteXY connection settings
  #define REMOTEXY_SERIAL Serial
  #define REMOTEXY_SERIAL_SPEED 115200
  #define REMOTEXY_WIFI_SSID "RemoteXY"
  #define REMOTEXY_WIFI_PASSWORD "12345678"
  #define REMOTEXY_SERVER_PORT 6377


  // RemoteXY configurate
  #pragma pack(push, 1)
  uint8_t RemoteXY_CONF[] =
  { 255,1,0,0,0,26,0,8,13,0,
  1,0,42,21,12,12,2,31,66,82,
  69,65,75,32,33,33,33,33,33,33,
  33,33,0 };

  // this structure defines all the variables of your control interface
  struct {

    // input variable
  uint8_t button_1; // =1 if button pressed, else =0

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

  } RemoteXY;
  #pragma pack(pop) */


unsigned int c0;

void setup()
{
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  pinMode(ASTEP_PIN,   OUTPUT);
  pinMode(ADIR_PIN,    OUTPUT);
  pinMode(AENABLE_PIN, OUTPUT);
  pinMode(11, OUTPUT);


  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(AENABLE_PIN, LOW);
  Serial.begin(115200);
  // Serial.begin(9600);
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 1000;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= ((1 << CS11) | (1 << CS10));
  interrupts();

  c0 = 1600; // was 2000 * sqrt( 2 * angle / accel )
  //  RemoteXY_Init ();

  //pinMode (PIN_SWITCH_1, OUTPUT);
}
int act = 0;
volatile int dir = 0;
volatile unsigned int maxSpeed = 800;
volatile unsigned long n = 0;
volatile float d;
volatile unsigned long stepCount = 0;
volatile unsigned long rampUpStepCount = 0;
volatile unsigned long totalSteps = 0;
volatile int stepPosition = 0;

volatile bool movementDone = false;

ISR(TIMER1_COMPA_vect)
{
  if ( stepCount < totalSteps ) {
    if (act == 1) {
      digitalWrite(ASTEP_PIN, HIGH);      /////ACTUATOR RUN
      digitalWrite(ASTEP_PIN, LOW);
    }
    else
    {
      digitalWrite(STEP_PIN, HIGH);       /////MOTOR RUN
      digitalWrite(STEP_PIN, LOW);
    }

    stepCount++;
    stepPosition += dir;
  }
  else {
    movementDone = true;
    TIMER1_INTERRUPTS_OFF
  }

  if ( rampUpStepCount == 0 ) { // ramp up phase
    n++;
    d = d - (2 * d) / (4 * n + 1);
    if ( d <= maxSpeed ) { // reached max speed
      d = maxSpeed;
      rampUpStepCount = stepCount;
    }
    if ( stepCount >= totalSteps / 2 ) { // reached halfway point
      rampUpStepCount = stepCount;
    }
  }
  else if ( stepCount >= totalSteps - rampUpStepCount ) { // ramp down phase
    n--;
    d = (d * (4 * n + 1)) / (4 * n + 1 - 2);
  }

  OCR1A = d;
}

void moveNSteps(long steps) {

  digitalWrite(DIR_PIN, steps < 0 ? HIGH : LOW);

  digitalWrite(ADIR_PIN, steps < 0 ? HIGH : LOW);
  dir = steps > 0 ? 1 : -1;
  totalSteps = abs(steps);
  d = c0;
  OCR1A = d;
  stepCount = 0;
  n = 0;
  rampUpStepCount = 0;
  movementDone = false;

  TIMER1_INTERRUPTS_ON
}

void moveToPosition(long p, bool wait = true) {
  Serial.println(p);
  moveNSteps(p - stepPosition);
  while ( wait && ! movementDone );
}

void loop() {
    if (Serial.available() > 0) {                  ////////FOR MOTOR
    int a = Serial.read();
    
    Serial.println(a);
    digitalWrite(ENABLE_PIN, 1);
    digitalWrite(AENABLE_PIN, 1);

    maxSpeed = 200;                   ///////////////////FOR MOTOR SPEED

    act = 0;                              ///FOR RUNNING MOTOR 0 else 1 in actuator
    switch (a) {
      case '0':
        moveToPosition(-60 * G * R);
        break;

        case '1':
        moveToPosition( -45 * G * R );
        break;

        case '2':
        moveToPosition( -30 * G * R );
        break;

        case '3':
        moveToPosition( -15 * G * R );
        break;

        case '4':
        moveToPosition( 0 * G * R );
        break;

        case '5':
        moveToPosition( 15 * G * R );
        break;

        case '6':
        moveToPosition( 30 * G * R );
        break;

        case '7':
        moveToPosition( 45 * G * R );
        break;

        case '8':
        moveToPosition( 60 * G * R );
        break;

      /*case '0':
        moveToPosition(-60 * G * R);
        //to take turn for some time
        break;

      case '1':
        moveToPosition( -25 * G * R );
        break;

      case '2':
        moveToPosition( 0 * G * R );
        break;

      case '3':
        moveToPosition( 25 * G * R );
        break;

      case '4':
        moveToPosition( 60 * G * R );
        
        break;*/

      case 'a':
        analogWrite(11, 72);
        digitalWrite(AENABLE_PIN, LOW);
        break;

      case 'b':
        digitalWrite(11, 0);
        digitalWrite(AENABLE_PIN, LOW);
        
        break;

      case 'c':
        analogWrite(11, 0);
        digitalWrite(AENABLE_PIN, LOW);
        act = 1;
        moveToPosition(0);
        maxSpeed = 150;
        digitalWrite(AENABLE_PIN, HIGH);
        moveToPosition(2500);
        delay(500);
        moveToPosition(0);
        digitalWrite(AENABLE_PIN, LOW);
        act = 0;

        break;

      case 'r':
        resetFunc();
        break;

      case 's':
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(AENABLE_PIN, HIGH);
        act = 0;
        moveToPosition(0);
        act = 1;
        moveToPosition(2900);
        act = 0;
        moveToPosition(0);
        digitalWrite(AENABLE_PIN, LOW);

    }
    digitalWrite(ENABLE_PIN, 0);
    digitalWrite(AENABLE_PIN, 0);
    serialFlush();
  }
}
