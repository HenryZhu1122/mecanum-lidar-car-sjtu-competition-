#include <Arduino.h>
#include <RPLidar.h>
#include "MecanumDriver.h"//引入驱动

#define DEBUG_SERIAL 1  //进入输出调试状态

#if DEBUG_SERIAL
  #define DBG_BEGIN(baud) Serial.begin(baud)
  #define DBG_PRINT(x) Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_BEGIN(baud)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif   //类似于别名

#define MOTOR1_PIN1 9
#define MOTOR1_PIN2 8
#define MOTOR2_PIN1 12
#define MOTOR2_PIN2 13
#define MOTOR3_PIN1 11
#define MOTOR3_PIN2 10
#define MOTOR4_PIN1 46
#define MOTOR4_PIN2 21 //引脚定义

const int F_ANGLE = 180;
const int L_ANGLE = 110;
const int R_ANGLE = 250; //侧前方的数据更有价值

const float BASE_SPEED = 60.0;     //直行速度
const float CORRECT_SPEED = 20.0;  //横向速度
const float TURN_SPEED = 75.0;     //转弯速度

const float FRONT_TURN_THRESH = 270;  //前方最小
const float SIDE_NEAR_THRESH = 170;   //侧方最小
const float SIDE_DIFF_MARGIN = 100;    //纠偏最小

const float VALID_MIN = 0;           // 搞个下界就够了

const int AVG_RANGE = 20;              // 取中心角左右
const unsigned long TURN_DURATION = 870;  // 转弯持续时间
const unsigned long POST_TURN_RUN = 400;  // 转后强制直行时间

#define MODE_RUN         0
#define MODE_TURN_LEFT   1
#define MODE_TURN_RIGHT  2
#define MODE_POST_TURN   3

RPLidar lidar;

MecanumDriver mecanum(
  MOTOR1_PIN1, MOTOR1_PIN2,
  MOTOR2_PIN1, MOTOR2_PIN2,
  MOTOR3_PIN1, MOTOR3_PIN2,
  MOTOR4_PIN1, MOTOR4_PIN2
);

float lidarDistances[360] = {0};

int currentMode = MODE_RUN;
unsigned long modeStartTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastGoodLidarTime = 0;
unsigned long lastLidarRestartTime = 0;//等待后续优化


void driveFixed(float fl, float fr, float bl, float br) {
  const float FL_TRIM = 1.00;
  const float FR_TRIM = 1.00;  
  const float BL_TRIM = 1.00;
  const float BR_TRIM = 1.00;//万一有的轮子不听话呢

  fl *= FL_TRIM;
  fr *= FR_TRIM;
  bl *= BL_TRIM;
  br *= BR_TRIM;

  mecanum.driveAllMotor(
    constrain(-bl, -255, 255),  // 左后
    constrain(-br, -255, 255),  // 右后
    constrain(fr, -255, 255),   // 右前
    constrain(fl, -255, 255)    // 左前
  );
}

void setSpeed(float speedX, float speedY, float angularW) {
  float fl = speedX + speedY + angularW;
  float fr = speedX - speedY - angularW;
  float bl = speedX - speedY + angularW;
  float br = speedX + speedY - angularW;//对速度进行叠加

  float maxVal = max(max(fabs(fl), fabs(fr)), max(fabs(bl), fabs(br)));

  if (maxVal > 255) {
    float scale = 255.0 / maxVal;
    fl *= scale;
    fr *= scale;
    bl *= scale;
    br *= scale;
  }

  driveFixed(fl, fr, bl, br);
}

void forward(float speed) {
  setSpeed(speed, 0, 0);
}

void back(float speed) {
  setSpeed(-speed, 0, 0);
}

void moveLeft(float speed) {
  setSpeed(0, -speed, 0);
}

void moveRight(float speed) {
  setSpeed(0, speed, 0);
}

void turnLeft(float speed) {
  setSpeed(0, 0, -speed);
}

void turnRight(float speed) {
  setSpeed(0, 0, speed);
}

void stopCar() {
  setSpeed(0, 0, 0);//后续还要加一种倒车状态
}

bool isValidDistance(float d) {
  return d > VALID_MIN;
}

float getAvgDistance(int centerAngle, int range) {
  float sum = 0;
  int count = 0;

  for (int i = -range; i <= range; i++) {
    int angle = (centerAngle + i + 360) % 360;
    float d = lidarDistances[angle];

    if (isValidDistance(d)) {
      sum += d;
      count++;//剔除无效数据
    }
  }

  if (count == 0) {
    return -1;
  }

  return sum / count;//算一定范围的平均值
}

void startTurnLeft() {
  currentMode = MODE_TURN_LEFT;
  modeStartTime = millis();
  //后续重点提升部分，这个拐角速度，时间都需要进行进一步优化
  DBG_PRINTLN("开始左转");
}

void startTurnRight() {
  currentMode = MODE_TURN_RIGHT;
  modeStartTime = millis();
  DBG_PRINTLN("开始右转");
}

void handleRun() {
  float front = getAvgDistance(F_ANGLE, AVG_RANGE);
  float left  = getAvgDistance(L_ANGLE, AVG_RANGE);
  float right = getAvgDistance(R_ANGLE, AVG_RANGE);

  if (millis() - lastPrintTime > 300) {
    lastPrintTime = millis();

    DBG_PRINT("前方距离是");
    DBG_PRINTLN(front);

    DBG_PRINT("左方距离是");
    DBG_PRINTLN(left);

    DBG_PRINT("右方距离是");
    DBG_PRINTLN(right);
  }


  if (front < 0 && left < 0 && right < 0) {
    stopCar();
    DBG_PRINTLN("雷达暂无有效数据，停车保护");
    return;
  }

  if (front > 0 && front < FRONT_TURN_THRESH) {
    if (left > 0 && right > 0) {
      if (left > right) {
        startTurnLeft();
      } else {
        startTurnRight();
      }
    }
    else if (left > 0) {
      startTurnLeft();
    }
    else if (right > 0) {
      startTurnRight();
    }
    else {
      startTurnLeft();
    }

    return;
  }

  float lateralSpeed = 0;

  if (left > 0 && right > 0) {
    if (left < SIDE_NEAR_THRESH && right >= SIDE_NEAR_THRESH) {
      lateralSpeed = CORRECT_SPEED;
      DBG_PRINTLN("开始向右纠正偏移");
    }
    else if (right < SIDE_NEAR_THRESH && left >= SIDE_NEAR_THRESH) {
      lateralSpeed = -CORRECT_SPEED;
      DBG_PRINTLN("开始向左纠正偏移");
    }
    else if (left < SIDE_NEAR_THRESH && right < SIDE_NEAR_THRESH) {
      if (left + SIDE_DIFF_MARGIN < right) {
        lateralSpeed = CORRECT_SPEED;
        DBG_PRINTLN("开始向右纠正偏移");
      } else if (right + SIDE_DIFF_MARGIN < left) {
        lateralSpeed = -CORRECT_SPEED;
        DBG_PRINTLN("开始向左纠正偏移");
      }
    }
  }
  else if (left > 0) {
    if (left < SIDE_NEAR_THRESH) {
      lateralSpeed = CORRECT_SPEED;
      DBG_PRINTLN("开始向右纠正偏移");
    }
  }
  else if (right > 0) {
    if (right < SIDE_NEAR_THRESH) {
      lateralSpeed = -CORRECT_SPEED;
      DBG_PRINTLN("开始向左纠正偏移");
    }
  }
//此部分后续需要进行重点优化，偏移的时间和速度应该与两端的距离差相关
  setSpeed(BASE_SPEED, lateralSpeed, 0);
}

//下面是转弯模式喵

void handleTurnLeft() {
  if (millis() - modeStartTime < TURN_DURATION) {
    turnLeft(TURN_SPEED);
  } else {
    stopCar();
    currentMode = MODE_POST_TURN;
    modeStartTime = millis();
    DBG_PRINTLN("左转完了，进入转弯后直行");
  }
}

void handleTurnRight() {
  if (millis() - modeStartTime < TURN_DURATION) {
    turnRight(TURN_SPEED);
  } else {
    stopCar();
    currentMode = MODE_POST_TURN;
    modeStartTime = millis();
    DBG_PRINTLN("右转结束，进入转弯后直行");
  }
}
void handlePostTurn() {
  setSpeed(BASE_SPEED, 0, 0);

  if (millis() - modeStartTime >= POST_TURN_RUN) {
    stopCar();
    currentMode = MODE_RUN;
    modeStartTime = millis();
    DBG_PRINTLN("转弯后直行结束，回到正常模式");
  }
}

//启动

void setup() {
  DBG_BEGIN(115200);

  lidar.begin(Serial2);
  lidar.startScan();

  mecanum.begin();

  currentMode = MODE_RUN;
  modeStartTime = millis();
  lastControlTime = millis();
  lastGoodLidarTime = millis();
  lastLidarRestartTime = millis();

  DBG_PRINTLN("小车启动");
}

//主循环
void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float dist = lidar.getCurrentPoint().distance;
    int angle = (int)round(lidar.getCurrentPoint().angle) % 360;

    if (angle >= 0 && angle < 360) {
      lidarDistances[angle] = dist;
      lastGoodLidarTime = millis();//读取雷达数据
    }
  } else {

    if (millis() - lastGoodLidarTime > 1500 && millis() - lastLidarRestartTime > 1500) {
      stopCar();
      lidar.startScan();
      lastLidarRestartTime = millis();
      DBG_PRINTLN("雷达长时间无有效点，重启扫描");
    }
  }

  // 直接每 50ms 执行一次喵，反正肯定可以读完喵
  if (millis() - lastControlTime >= 50) {
    lastControlTime = millis();

    switch (currentMode) {
      case MODE_RUN:
        handleRun();
        break;

      case MODE_TURN_LEFT:
        handleTurnLeft();
        break;

      case MODE_TURN_RIGHT:
        handleTurnRight();
        break;
      
       case MODE_POST_TURN:
        handlePostTurn();
        break;
    }
  }
}
