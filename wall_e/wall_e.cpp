#include <Wire.h>
#include <ZumoShield.h>

#define LOG_SERIAL // write log output to serial port

#define LED 13
Pushbutton button(ZUMO_BUTTON);

#define RA_SIZE 3
#define XY_ACCELERATION_THRESHOLD 15000

ZumoMotors motors;

#define REVERSE_SPEED     100 // 0 is stopped, 400 is full speed
#define TURN_SPEED        100
#define SEARCH_SPEED      100
#define SUSTAINED_SPEED   200 // switches to SUSTAINED_SPEED from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        100
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms

#define RIGHT 1
#define LEFT -1



// Sound Effects
ZumoBuzzer buzzer;
const char sound_effect[]
PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody

unsigned long loop_start_time;
unsigned long last_turn_time;
unsigned long contact_made_time;
#define MIN_DELAY_AFTER_TURN          50  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   100  // ms = min delay between detecting new contact event


// sonar

int trigPin = 11;    // Trigger
int echoPin = 2;     // Echo
long duration, distance_cm;

long loop_count = 0;

// RunningAverage class
// based on RunningAverage library for Arduino
// source:  https://playground.arduino.cc/Main/RunningAverage
template <typename T>
class RunningAverage
{
public:
    RunningAverage(void);
    RunningAverage(int);
    ~RunningAverage();
    void clear();
    void addValue(T);
    T getAverage() const;
    void fillValue(T, int);
protected:
    int _size;
    int _cnt;
    int _idx;
    T _sum;
    T * _ar;
    static T zero;
};

class Accelerometer : public LSM303
{
    typedef struct acc_data_xy
    {
        unsigned long timestamp;
        int x;
        int y;
        float dir;
    } acc_data_xy;

public:
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE) {};
    ~Accelerometer() {};
    void enable(void);
    void getLogHeader(void);
    void readAcceleration(unsigned long timestamp);
    float len_xy() const;
    float dir_xy() const;
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
    float dir_xy_avg(void) const;
private:
    acc_data_xy last;
    RunningAverage<int> ra_x;
    RunningAverage<int> ra_y;
};


void Accelerometer::enable(void)
{
    writeAccReg(LSM303::CTRL_REG1_A, 0x27);

    if (getDeviceType() == LSM303::device_DLHC)
        writeAccReg(LSM303::CTRL_REG4_A, 0x08); // DLHC: enable high resolution mode
}

void Accelerometer::getLogHeader(void)
{
    Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
    Serial.println();
}

void Accelerometer::readAcceleration(unsigned long timestamp)
{
    readAcc();
    if (a.x == last.x && a.y == last.y) return;

    last.timestamp = timestamp;
    last.x = a.x;
    last.y = a.y;

    ra_x.addValue(last.x);
    ra_y.addValue(last.y);

#ifdef LOG_SERIAL
    Serial.print(last.timestamp);
    Serial.print("  ");
    Serial.print(last.x);
    Serial.print("  ");
    Serial.print(last.y);
    Serial.print("  ");
    Serial.print(len_xy());
    Serial.print("  ");
    Serial.print(dir_xy());
    Serial.print("  |  ");
    Serial.print(sqrt(static_cast<float>(ss_xy_avg())));
    Serial.print("  ");
    Serial.print(dir_xy_avg());
    Serial.println();
#endif
}

float Accelerometer::len_xy() const
{
    return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
    return atan2(last.x, last.y) * 180.0 / M_PI;
}

int Accelerometer::x_avg(void) const
{
    return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
    return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
    long x_avg_long = static_cast<long>(x_avg());
    long y_avg_long = static_cast<long>(y_avg());
    return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

float Accelerometer::dir_xy_avg(void) const
{
    return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}




Accelerometer lsm303;
boolean in_contact;  // set when accelerometer detects contact with opposing robot


// setup

void setup() {

#ifdef LOG_SERIAL
    Serial.begin(9600);
#endif

    Wire.begin();
    lsm303.init();
    lsm303.enable();
    randomSeed((unsigned int) millis());

    // uncomment if necessary to correct motor directions
    //motors.flipLeftMotor(true);
    motors.flipRightMotor(true);

    // init sonar
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(LED, OUTPUT);
}

void loop(){
    loop_test();
}


void loop_test(){
    move_forward();
    delay(2000);
    stop();
    delay(3000);
    move_back();
    delay(2000);
    stop();
    delay(3000);
}
void move_back(){
    motors.setSpeeds(-SUSTAINED_SPEED, SUSTAINED_SPEED);
}
void move_forward(){
    motors.setSpeeds(SUSTAINED_SPEED, -SUSTAINED_SPEED);
}

// 135 667 844
void loop_contact(){
    loop_start_time = millis();
    lsm303.readAcceleration(loop_start_time);
    Serial.print("ss_xy_avg=");
    Serial.println(lsm303.ss_xy_avg());
    if(check_for_contact()){
        digitalWrite(LED, HIGH);
        delay(5000);
        digitalWrite(LED, LOW);
        Serial.println("Contact");
    }
    delay(100);
}

void loop_lego(){
    move_back();
    delay(300);
    turnLeft();
    delay(300);
    move_forward();
    delay(400);
    turnLeft();
    delay(300);

    digitalWrite(LED,HIGH);
    delay(1000);
    digitalWrite(LED,LOW);

}

void loop_explore() {
    loop_start_time = millis();

    long direction = random(2);
    while(getDistance() < 30) {
        if(direction == 0) {
            turnLeft();
        } else {
            turnRight();
        }
    }
    if(getDistance > 30){
        move_forward();
        delay(100);
    }
    if(loop_count%30==0){
        move_back();
        delay(500);
        if(direction == 0) {
            turnLeft();
            turnLeft();
        } else {
            turnRight();
            turnRight();
        }
    }
    loop_count++;
}



bool check_for_contact(){
    static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
    return (lsm303.ss_xy_avg() >  threshold_squared);
}
void turnRight(){
    motors.setSpeeds(SUSTAINED_SPEED, -SUSTAINED_SPEED);
    delay(100);
    stop();
}
void turnLeft(){
    motors.setSpeeds(-SUSTAINED_SPEED,SUSTAINED_SPEED);
    delay(100);
    stop();
}
void stop(){
    motors.setSpeeds(0,0);
}

long getDistance(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    return (duration/2) / 29.1;
}
