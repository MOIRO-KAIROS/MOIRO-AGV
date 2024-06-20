#include <Encoder.h> // Encoder Library

// Motor 제어 핀 설정
#define PWMA 12    // A모터(Left Front) 속도 제어 핀
#define DIRA1 34 
#define DIRA2 35  // A모터 방향 제어 핀
#define PWMB 8    // B모터(Right Front) 속도 제어 핀
#define DIRB1 37 
#define DIRB2 36  // B모터 방향 제어 핀
#define PWMC 6   // C모터(Right Front) 속도 제어 핀
#define DIRC1 43 
#define DIRC2 42  // C모터 방향 제어 핀
#define PWMD 5    // D모터(Right Front) 속도 제어 핀
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  // D모터 방향 제어 핀

// PWM 값 설정
#define MAX_PWM   200
#define MIN_PWM   130
int Motor_PWM = 130; // 모터 초기 속도

// --- SPD Motor ---

class SPDMotor {
  public:
  SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );

  /// Set the PWM speed and direction pins.
  /// pwm = 0, stop (no active control)
  /// pwm = 1 to 255, proportion of CCW rotation
  /// pwm = -1 to -255, proportion of CW rotation
  void speed( int pwm );

  /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
  void hardStop();

  /// Get the current speed.
  int getSpeed();

  /// Get the current rotation position from the encoder.
  long getEncoderPosition();

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;

    // Current speed setting.
    int _speed;
};

SPDMotor::SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 ) {
  _encoder = new Encoder(encoderA, encoderB);
  _encoderReversed = encoderReversed;

  _motorPWM = motorPWM;
  pinMode( _motorPWM, OUTPUT );
  _motorDir1 = motorDir1;
  pinMode( _motorDir1, OUTPUT );
  _motorDir2 = motorDir2;
  pinMode( _motorDir2, OUTPUT );
}

/// Set the PWM speed and direction pins.
/// pwm = 0, stop (no active control)
/// pwm = 1 to 255, proportion of CCW rotation
/// pwm = -1 to -255, proportion of CW rotation
void SPDMotor::speed( int speedPWM ) {
  _speed = speedPWM;
  if( speedPWM == 0 ) {
    digitalWrite(_motorDir1,LOW);
    digitalWrite(_motorDir2,LOW);
    analogWrite( _motorPWM, 255);
  } else if( speedPWM > 0 ) {
    digitalWrite(_motorDir1, LOW );
    digitalWrite(_motorDir2, HIGH );
    analogWrite( _motorPWM, speedPWM < 255 ? speedPWM : 255);
  } else if( speedPWM < 0 ) {
    digitalWrite(_motorDir1, HIGH );
    digitalWrite(_motorDir2, LOW );
    analogWrite( _motorPWM, (-speedPWM) < 255 ? (-speedPWM): 255);
  }
}

/// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
void SPDMotor::hardStop() {
    _speed = 0;
    digitalWrite(_motorDir1,HIGH);
    digitalWrite(_motorDir2,HIGH);
    analogWrite( _motorPWM, 0);
}

/// Get the current speed.
int SPDMotor::getSpeed() {
    return _speed;
}

/// Get the current rotation position from the encoder.
long SPDMotor::getEncoderPosition() {
  long position = _encoder->read();
  return _encoderReversed ? -position : position;
}

SPDMotor *motorLF = new SPDMotor(18, 31, true, PWMA, DIRA1, DIRA2); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, PWMB, DIRB2, DIRB1); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor( 3, 49, true,  PWMC, DIRC1, DIRC2); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 2, A1, false, PWMD, DIRD2, DIRD1); // <- NOTE: Motor Dir pins reversed for opposite operation

void setup()
{
  Serial.begin(9600);
  HC06.begin(9600);
  delay(300) ;//added delay to give wireless ps2 module some time to startup, before configuring it
  Serial.println("SPDMotor control via Bluetooth");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any trailing newline characters

    if (command.startsWith("F")) { // 전진
      motorLF->speed(Motor_PWM);
      motorRF->speed(Motor_PWM);
      motorLR->speed(Motor_PWM);
      motorRR->speed(Motor_PWM);
    } else if (command.startsWith("B")) { // 후진
      motorLF->speed(-Motor_PWM);
      motorRF->speed(-Motor_PWM);
      motorLR->speed(-Motor_PWM);
      motorRR->speed(-Motor_PWM);
    } else if (command.startsWith("S")){ // 정지
      motorLF->speed(0);
      motorRF->speed(0);
      motorLR->speed(0);
      motorRR->speed(0);
    } else if (command.startsWith("L")){ // 왼쪽 이동
      motorLF->speed(-Motor_PWM);
      motorRF->speed(Motor_PWM);
      motorLR->speed(Motor_PWM);
      motorRR->speed(-Motor_PWM);
    } else if (command.startsWith("R")){ // 오른쪽 이동
      motorLF->speed(Motor_PWM);
      motorRF->speed(-Motor_PWM);
      motorLR->speed(-Motor_PWM);
      motorRR->speed(Motor_PWM);
    } else if (command.startsWith("T")){ // 시계방향 회전
      motorLF->speed(Motor_PWM);
      motorRF->speed(-Motor_PWM);
      motorLR->speed(Motor_PWM);
      motorRR->speed(-Motor_PWM);
    }

    
    Serial.print("Command received: ");
    Serial.println(command);
    Serial.print("LF: ");
    Serial.print(motorLF->getEncoderPosition());
    Serial.print(" RF: ");
    Serial.print(motorRF->getEncoderPosition());
    Serial.print(" LR: ");
    Serial.print(motorLR->getEncoderPosition());
    Serial.print(" RR: ");
    Serial.println(motorRR->getEncoderPosition());
  }
}
