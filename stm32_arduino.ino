#include <Arduino.h>
#include <EEPROM.h>

// 定义 LED 引脚
#define LED_PIN_C13 PC13
#define PULSE_PIN PA2
#define DIRECTION_PIN PA4

// 定义限位开关和方向控制引脚
#define LIMIT_SWITCH_LEFT PA6  //red
#define LIMIT_SWITCH_RIGHT PA7

#define ADC_PIN PA0


#define EEPROM_ADDR 0  // 存储频率值的 EEPROM 地址

HardwareTimer *MyTim = new HardwareTimer(TIM1);

volatile bool motorRunning = true; // 用于指示电机是否正在运行
volatile bool toggleA0 = false;
volatile bool toggleC13 = false;
volatile unsigned int counter = 0; // 计数器，用于 PC13 频率控制


// 定时器中断处理函数
void TimerHandler() {
  if (!motorRunning) {
    return; // 如果电机不运行，直接返回
  }

  toggleA0 = !toggleA0;
  digitalWrite(PULSE_PIN, toggleA0);

  counter++;
  if (counter % 100 == 0) {
    toggleC13 = !toggleC13;
    digitalWrite(LED_PIN_C13, toggleC13);
  }
}

// 更新定时器频率
void updateTimerFrequency(float newFrequency) {
  // 停止定时器以进行配置
  MyTim->pause();
  // 设置新频率
  MyTim->setOverflow(newFrequency, HERTZ_FORMAT);
  // 附加中断处理函数
  MyTim->attachInterrupt(TimerHandler);
  // 启动定时器
  MyTim->resume();
  motorRunning = true;
}

// 停止电机
void stopMotor() {
  motorRunning = false;
  MyTim->pause(); // 停止定时器
  digitalWrite(LED_PIN_C13, LOW);
  digitalWrite(PULSE_PIN, LOW);
}

// 设置电机方向
void setMotorDirection(bool direction) {
  digitalWrite(DIRECTION_PIN, direction);
  Serial.print(counter);
  Serial.print(" c ");
  if(direction==HIGH){
    Serial.println("Left red limit: set HIGH");
  }
  if(direction==LOW){
    Serial.println("Right green limit: set LOW");
  }  


  counter = 0;
}

// 从 EEPROM 读取频率值
float readFrequencyFromEEPROM() {
  float frequency;
  EEPROM.get(EEPROM_ADDR, frequency);
  return frequency;
}

// 将频率值写入 EEPROM
void writeFrequencyToEEPROM(float frequency) {
  EEPROM.put(EEPROM_ADDR, frequency);
}

void setup() {
  // 初始化串口通信，波特率为 115200
  Serial.begin(115200);

  // 初始化 LED 引脚为输出模式
  pinMode(LED_PIN_C13, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);

  // 初始化限位开关和方向控制引脚
  pinMode(LIMIT_SWITCH_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_RIGHT, INPUT_PULLUP);

  pinMode(DIRECTION_PIN, OUTPUT);
  setMotorDirection(HIGH);

  // 初始化 ADC
  analogReadResolution(12); // 设置 ADC 分辨率为 12 位
  pinMode(ADC_PIN, INPUT_ANALOG); 

  // 从 EEPROM 读取频率值并设置定时器周期
  float storedFrequency = readFrequencyFromEEPROM();
  if (storedFrequency > 0) {
    updateTimerFrequency(storedFrequency);
    Serial.print("Loaded stored frequency: ");
    Serial.println(storedFrequency);
  } else {
    updateTimerFrequency(1.1);
    Serial.println("No valid stored frequency. Using default: 1.1 Hz");
  }
  
  // 打印编译时间
  Serial.print("Compiled on: ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);

  Serial.println("Timer started");
}


void loop() {
  static unsigned long lastSerialTime = 0;
  static unsigned long lastAdcTime = 0;
  static int lastAdcValue = 0;

  unsigned long currentTime = millis();

 
  // 检查是否有串口输入
  if (Serial.available() > 0) {
    float newFrequency = Serial.parseFloat(); // 读取串口输入的新频率值
    if (newFrequency > 0) {
      updateTimerFrequency(newFrequency); // 更新定时器频率
      writeFrequencyToEEPROM(newFrequency); // 将新频率值写入 EEPROM
      Serial.print("Updated frequency to: ");
      Serial.println(newFrequency);
    } else {
      Serial.println("Invalid frequency. Please enter a positive number.");
    }

  
  }

  
if (currentTime - lastAdcTime >= 100) {
    lastAdcTime = currentTime;
    int adcValue = analogRead(ADC_PIN); // 读取 ADC_PIN 的 ADC 值

    Serial.print(" ADC:");
    Serial.print(adcValue);

    if (abs(adcValue - lastAdcValue) > 20) {
      float newFrequency = adcValue * 12; // 将 ADC 值转换为频率范围 0-50000

      if (adcValue > 4000) {
        newFrequency = 100; // 如果 ADC 超过  设置为 100
        Serial.print("[ADC>4000, 100hz]");
      }

      updateTimerFrequency(newFrequency); // 更新定时器频率
      //writeFrequencyToEEPROM(newFrequency); // 将新频率值写入 EEPROM  疯了吗，持续写入芯片损坏

      Serial.print(" Update_frequency: ");
      Serial.print(newFrequency);
      lastAdcValue = adcValue; // 更新上一次的 ADC 值
    }
  }




  // 检查限位开关状态
  bool leftSwitch = digitalRead(LIMIT_SWITCH_LEFT) == LOW;
  bool rightSwitch = digitalRead(LIMIT_SWITCH_RIGHT) == LOW;

  Serial.print(",limit:");
  Serial.print(leftSwitch);
  Serial.print(rightSwitch);
  if (leftSwitch && rightSwitch) {
    // 如果两个限位开关同时按下，急停
    Serial.print(counter);
    Serial.println(" Error: Both limit active. Stopping motor.");
    stopMotor();
  } else if (leftSwitch) {
    setMotorDirection(HIGH);
  } else if (rightSwitch) {
    setMotorDirection(LOW);
  }

   // 打印时间信息
  if (currentTime - lastSerialTime >= 1000) {
    lastSerialTime = currentTime;
    Serial.print(" ,Time: ");
    Serial.println(currentTime / 1000); // 打印秒数
  }


  delay(200); // 防抖延迟
}