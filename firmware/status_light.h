#ifndef __STATUS_LIGHT_H__
#define __STATUS_LIGHT_H__
class StatusLed{
  private:
  bool is_blinking = false;
  unsigned long next_status_change;

  public:
  StatusLed(): next_status_change(0){}


  void init(){
    pinMode(LED_BUILTIN, OUTPUT);
  }
  void off(){
    is_blinking = false;
    digitalWrite(LED_BUILTIN, LOW);
  }

  void on(){
    is_blinking = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }

  void blink(){
    is_blinking = true;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    next_status_change = millis() + 500;
  }

  void update(){
    if (is_blinking && millis() > next_status_change){
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      next_status_change = millis() + 500;
    }
  }


};
#endif
