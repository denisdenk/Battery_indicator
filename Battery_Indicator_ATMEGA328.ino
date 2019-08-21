/*
 * FUSES: HIGH 0xDE; LOW 0xFF (16MHz External Crystal)
 */


uint16_t counter = 0;
int timer = 0;
int i = 0;

bool longPush = false;
bool run = false;

// Function prototypes
void ShowVoltage(void);
float mapping(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  Serial.begin(115200);
  // LED`s
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  // POWER
  pinMode(9, OUTPUT);
  // Button
  pinMode(2, INPUT_PULLUP);

  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(5, !digitalRead(5));
    digitalWrite(6, !digitalRead(6));
    digitalWrite(7, !digitalRead(7));
    digitalWrite(10, !digitalRead(10));
    digitalWrite(11, !digitalRead(11));
    delay(100);
  }

  analogReference(EXTERNAL);
  analogRead(A0);
  Serial.println("Init OK!");
}

void loop() {
  while (!digitalRead(PD2)) {
    delay(5);
    counter++;
    //Serial.println(counter);
  }
  
  // Если длительное нажатие
  if (counter > 50) {
    // Кнопка была долго нажата, выставляем флаг
    longPush = true;
    counter = 0;
    Serial.println("Long push!\r\n");
  }
  // Если короткое нажатие
  if (counter > 7 && counter < 40) {
    Serial.println("Short push!");
    // Если перед ним было длинное - подключаем/отключаем питание
    if (longPush) {
      // Если работало - отключаем и наоборот
      if (!run) {
        run = true;
        digitalWrite(9, HIGH);
        Serial.println("*******");
        Serial.println("*Work!*");
        Serial.println("*******");
      }
      else {
        digitalWrite(9, LOW);
        run = false;
        Serial.println("************");
        Serial.println("*Work done!*");
        Serial.println("************");
      }
      // Сбрасываем флаг длинного нажатия
      longPush = false;
    }
    // Однократное, короткое нажатие, показываем напряжение
    else {
      counter = 0;
      ShowVoltage();
    }
  }
}

void ShowVoltage(void) {
  /* Получение данных с АЦП */
  int ADCdata[5] = {0};
  int ADC_ExpVal = 0;
  // Считаем напряжение 4 раза и считаем среднее для точности
  for (uint8_t i = 0; i < 4; i++) {
    // Считываем  ADC
    ADCdata[i] = analogRead(A0);
  }

  ADC_ExpVal = (ADCdata[0] + ADCdata[1] + ADCdata[2] + ADCdata[3]) / 4;
  // Опорное напряжение 5В, значение регистра до 1024 => коэффициент = 5/1024=0.0048828
  float voltage = ADC_ExpVal * 0.0048828;
  // Приводим к виду 25.2V.
  voltage = mapping(voltage, 0, 5, 0, 25.2);
  Serial.print("Voltage = ");
  Serial.println(voltage);
  Serial.println();

  /* 100% заряженный аккумулятор = 25,2В.
    0% = 19,8В.
    Разница 5,4В. => 20% это 1.08В.
  */

  // 100%
  if (voltage >= 24.12) {
    PORTD |= (1 << PD5) | (1 << PD6) | (1 << PD7);
    PORTB |= (1 << PB2) | (1 << PB3);

    _delay_ms(500);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);
    PORTD &= ~(1 << PD5);
    PORTD &= ~(1 << PD6);
    PORTD &= ~(1 << PD7);
  }
  // 80%
  else if (voltage >= 23.04 && voltage < 24.12) {
    PORTD |= (1 << PD5) | (1 << PD6) | (1 << PD7);
    PORTB |= (1 << PB2);
    PORTB &= ~(1 << PB3);

    _delay_ms(500);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);
    PORTD &= ~(1 << PD5);
    PORTD &= ~(1 << PD6);
    PORTD &= ~(1 << PD7);
  }
  // 60%
  else if (voltage >= 21.96 && voltage < 23.04) {
    PORTD |= (1 << PD5) | (1 << PD6) | (1 << PD7);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);

    _delay_ms(500);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);
    PORTD &= ~(1 << PD5);
    PORTD &= ~(1 << PD6);
    PORTD &= ~(1 << PD7);
  }
  // 40%
  else if (voltage >= 20.88 && voltage < 21.96) {
    PORTD |= (1 << PD5) | (1 << PD6);
    PORTD &= ~(1 << PD7);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);

    _delay_ms(500);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);
    PORTD &= ~(1 << PD5);
    PORTD &= ~(1 << PD6);
    PORTD &= ~(1 << PD7);
  }
  // 20%
  else if (voltage >= 19.8 && voltage < 20.88) {
    PORTD |= (1 << PD5);
    PORTD &= ~(1 << PD6);
    PORTD &= ~(1 << PD7);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);

    _delay_ms(500);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);
    PORTD &= ~(1 << PD5);
    PORTD &= ~(1 << PD6);
    PORTD &= ~(1 << PD7);
  }
}

float mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
