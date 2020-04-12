

#define pinSplit 2
#define pinPause 3
#define pinLED 5

//milliseconds
#define timeConnectivityPoll 1000
#define timeEnded 250
#define timePaused 1000
#define timeFade 50  //Results in ~2,5s fades (50ms * (255 / 5) = 2550ms)

bool trigSplit = false;
bool trigPause = false;

bool split = false;
bool pause = false;

char message;

unsigned long timerLED = 0;
bool fadeState = false; //false = fade rising, true = fade falling
int fadePhase = 0; //0 - 255
bool stateLED = false;

bool newRun = false;
int newRunBlink = 0;

unsigned long timerLastPoll = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(pinSplit, INPUT_PULLUP);
  pinMode(pinPause, INPUT_PULLUP);

  pinMode(pinLED, OUTPUT);
}


void loop()
{
  split = F_TRIG(trigSplit, pinSplit);
  pause = F_TRIG(trigPause, pinPause);

  buttonHandler();
  runStatus();
  
  delay(5);

  connectionPoll();

  delay(5);
}


void connectionPoll()
{
  if ((millis() - timerLastPoll) >= timeConnectivityPoll)
  {
    Serial.write(">");
    timerLastPoll = millis();
  }
}


void buttonHandler()
{
  if (split)
  {
    Serial.write("1");
  }
  
  if (pause)
  {
    Serial.write("2");
  }

  if (Serial.available() > 0)
  {
    message = Serial.read();

    if (message == '1')
    {
      newRun = true;
    }
  }
}


void runStatus()
{
  //1 = running, 2 = paused, 3 = run ended 4 = not running (resetted)
  switch(message)
  {
    case '1':
      if (newRun)
      {
        blinkLED(timeEnded);
      }
      else
      {
        stateLED = true;
        digitalWrite(pinLED, stateLED);
      }
      break;

    case '2':
      blinkLED(timePaused);
      break;

    case '3':
      blinkLED(timeEnded);
      break;

    case '4':
      fadeLED(timeFade);
      break;

    default:
      stateLED = false;
      break;
  }
}


void blinkLED(int timeMS)
{
  //Check if time has passed. If it has, flip the variable to turn the led on/off and reset timer
  if ((millis() - timerLED) >= timeMS)
  {
    stateLED = !stateLED;
    timerLED = millis();

    digitalWrite(pinLED, stateLED);


    //Blink LED a few times at the start of a new run
    if (newRun)
    {
      newRunBlink++;

      if (newRunBlink > 4)
      {
        newRun = false;
        newRunBlink = 0;
      }
    }
  }
}


void fadeLED(int timeFadeMS)
{
  //Check if time has passed. If it has, increase the pwm duty cycle and reset the timer
  if ((millis() - timerLED) >= timeFadeMS)
  {
    if (fadeState == false)
    {
      fadePhase = fadePhase + 5;

      if (fadePhase >= 255)
      {
        fadePhase = 255;
        fadeState = true;
      }
    }

    if (fadeState == true)
    {
      fadePhase = fadePhase - 5;

      if (fadePhase <= 0)
      {
        fadePhase = 0;
        fadeState = false;
      }
    }
    
    timerLED = millis();

    analogWrite(pinLED, fadePhase);
  }
}

//INPUT_PULLUP is NC (=1)
bool F_TRIG(bool &trig, int pin)
{
  bool pinState;

  pinState = digitalRead(pin);

  //Falling edge of pin (button pressed)
  if (pinState == false && trig == false)
  {
    trig = true;

    return true;
  }

  //Button is kept pressed
  if (pinState == false && trig == true)
  {
    return false;
  }

  //Rising edge of pin, reset trigger
  if (pinState == true && trig == true)
  {
    trig = false;
    
    return false;
  }

  return false;   //Default
}

