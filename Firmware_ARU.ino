#include <LCD_1602_RUS.h>
LCD_1602_RUS lcd(0x27, 16, 2);

#include "GyverEncoder.h"
#define SMOOTH_ALGORITHM
#include "GyverStepper2.h"

// Определение пинов для TB6560AHQ
#define STEP_PIN 2    // Пин для сигнала STEP
#define ENABLE_PIN 3  // Пин для сигнала ENABLE
#define DIR_PIN 4     // Пин для сигнала DIR

#define SET_P A2       // Rонцевик 
#define RESET_P A3     // Сброс позиции 
GStepper2<STEPPER2WIRE> stepper(1600, STEP_PIN, DIR_PIN, ENABLE_PIN); 

// Определение пинов для энкодера
#define DT 5  // S1
#define CLK 6 // S2
#define SW 7  // KEY
Encoder enc1(CLK, DT, SW);

int32_t prevSETPOS = 0;  // Предыдущее значение SETPOS
int32_t SETPOS; // Новая переменная для результата
float platformAngle = 0;       // Текущий угол платформы (от 0 до 360)
float value = 1;               // Текущее значение угла, Начальное значение 1
float previousValue = 1;       // Предыдущее значение угла шага
bool editMode = false;         // Режим редактирования (false - нельзя менять, true - можно менять)

void setup()
{
  // тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый.
  stepper.reverse(true);
  enc1.setType(TYPE2);
  
  // Настройки шагового двигателя
  stepper.setMaxSpeed(9000);       // Скорость движения к цели Макс. скорость: 37000 шаг/с на полной, 14000 шаг/с на разгоне
  stepper.setAcceleration(7000);   // Ускорение
  
  // пуллапим. Кнопки замыкают на GND
  pinMode(SET_P, INPUT_PULLUP);
  pinMode(RESET_P, INPUT_PULLUP);
  
  Serial.begin(9600);

  lcd.init();                  // Инициализация LCD
  lcd.backlight();             // Включение подсветки дисплея
  lcd.setCursor(0, 0);
  lcd.print("Position:       ");
  lcd.setCursor(0, 1);         // Устанавливаем курсор
  lcd.print("Angle:");
  lcd.setCursor(6, 1);         // Устанавливаем курсор
  lcd.print(value, 1);         // Вывод шага
  lcd.write(byte(223));        // Вывод символа градуса

  homing();                    // Возврат в нулевое положение


}



void loop()
{
  enc1.tick();
  stepper.tick();   // Двигатель асинхронно крутится

  // Сброс позиции по нажатию кнопки RESET_P
  if (digitalRead(RESET_P) == LOW) { // Кнопка нажата
    resetPosition();  // Вызываем функцию сброса позиции
  }

  // Проверяем нажатие кнопки энкодера
  if (enc1.isClick()) {
    editMode = !editMode;  // Переключаем режим редактирования
    lcd.setCursor(0, 0);  // Переключение отображения режима на экране
    lcd.print(editMode ? "Edit Mode      " : "Position:     ");  // Показ режима на экране
  }


  if (editMode) {

    // В режиме редактирования обрабатываем изменение значения
    handleEditMode();

  } else {
    // Вне режима редактирования
    if (enc1.isRight()) {
      platformAngle += value;
      if (platformAngle >= 360) platformAngle -= 360;
      LCDInfo();  // Обновляем значения на экране
    }

    if (enc1.isLeft()) {
      platformAngle -= value;
      if (platformAngle < 0) platformAngle += 360;

      LCDInfo();    // Обновляем значения на экране
    }
    Steppers();     // Движение до заданной позиции
  }

}



// Установка шага для режима "EditMode"
void handleEditMode() {
  // Прибавляем к шагу 1
  if (enc1.isRight()) value++;
  if(enc1.isLeft()) value--;
  // Прибавляем к шагу 0.1
  if (enc1.isRightH()) value += 0.1;
  if(enc1.isLeftH()) value -= 0.1;
  // Ограничиваем диапазон значений от 1 до 30
  value = constrain(value, 0.1, 30);

  // Проверяем, изменилось ли значение
  if (value != previousValue) {
    lcd.setCursor(6, 1);    // Устанавливаем курсор
    lcd.print("     ");     // Стираем предыдущее значение
    lcd.setCursor(6, 1);    // Устанавливаем курсор
    lcd.print(value,1);     // Выводим новое значение
    lcd.write(byte(223));   // Символ градуса
    previousValue = value;  // Обновляем предыдущее значение
  }

}


// Возврат в нулевое положение
// Возврат в нулевое положение с поворотом на 90 градусов
void homing() {
  if (digitalRead(SET_P)) {       // если концевик X не нажат
    stepper.setSpeed(-4000);      // двигаем в сторону концевика
    while (digitalRead(SET_P)) {  // пока кнопка не нажата
      stepper.tick();             // крутим
    }
    stepper.brake();              // останавливаем двигатель
    delay(2000);
  }
  
  // Сбрасываем координаты в 0
  stepper.reset();
  platformAngle = 0;  

  // Двигаем мотор на 90 градусов (перевод в шаги)
  int32_t steps90 = round(1600L * 60 / 4);  // 90 градусов = 1/4 оборота
  stepper.setTarget(steps90);               // задаем цель
  while (!stepper.ready()) {                 // ждем завершения движения
    stepper.tick();
  }

  // Снова сбрасываем координаты в 0
  stepper.reset();
  platformAngle = 0;  
  LCDInfo();            // Обновляем значения на экране
}


// Вывод информации на дисплей
void LCDInfo() {
  lcd.setCursor(9, 0);          // Устанавливаем курсор на позицию
  lcd.print("       ");         // Стираем предыдущее значение
  lcd.setCursor(9, 0);          // Устанавливаем курсор на позицию
  lcd.print(platformAngle, 1);  // Вывод значения угла с одним символом после запятой
  lcd.write(byte(223));         // Вывод символа градуса
}

// Сброс позиции
void resetPosition() {
  stepper.brake();      // Останавливаем двигатель
  stepper.reset();      // Сбрасываем координаты в 0
  platformAngle = 0;    // Сбрасываем угол платформы
  LCDInfo();            // Обновляем значения на экране
}


void Steppers() {
    stepper.tick();
    
    // Перевод угла платформы в шаги двигателя
    int32_t targetSteps = round(platformAngle * (1600L * 60 / 360.0));

    // Текущая позиция в шагах
    int32_t currentSteps = stepper.getCurrent();

    // Рассчитываем разницу и выбираем кратчайший путь
    int32_t delta = targetSteps - currentSteps;

    // Учет круговой системы (половина и полный круг в шагах)
    int32_t halfCircleSteps = (1600L * 60) / 2;
    int32_t fullCircleSteps = 1600L * 60;

    if (delta > halfCircleSteps) delta -= fullCircleSteps;
    if (delta < -halfCircleSteps) delta += fullCircleSteps;

    // Устанавливаем новую цель
    stepper.setTarget(currentSteps + delta);
      while (!stepper.ready()) {                 // ждем завершения движения
    stepper.tick();
  }
}
