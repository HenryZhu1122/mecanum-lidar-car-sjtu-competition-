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
const int R_ANGLE = 250;

const float BASE_SPEED = 60.0;     //直行速度
const float CORRECT_SPEED = 20.0;  //横向速度
const float TURN_SPEED = 75.0;     //转弯速度

const float FRONT_TURN_THRESH = 270;  //前方最小
const float SIDE_NEAR_THRESH = 170;   //侧方最小
const float SIDE_DIFF_MARGIN = 100;    //纠偏最小

const float VALID_MIN = 0;           // 搞个下界就够了

const int AVG_RANGE = 20;              // 取中心角左右
const unsigned long TURN_DURATION = 900;  // 转弯持续时间

#define MODE_RUN         0
#define MODE_TURN_LEFT   1
#define MODE_TURN_RIGHT  2

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
unsigned long lastLidarRestartTime = 0;

// 这里的 fl/fr/bl/br 都按照“正值=该轮向前转”理解
// 根据你实测：
// 第1路：左后，正值后转，负值前转
// 第2路：右后，正值后转，负值前转
// 第3路：右前，正值前转，负值后转
// 第4路：左前，正值前转，负值后转
void driveFixed(float fl, float fr, float bl, float br) {
  const float FL_TRIM = 1.00;
  const float FR_TRIM = 1.00;  // 右前轮补偿，先加到1.30
  const float BL_TRIM = 1.00;
  const float BR_TRIM = 1.00;

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
  float br = speedX + speedY - angularW;

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
  setSpeed(0, 0, 0);
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
      count++;
    }
  }

  if (count == 0) {
    return -1;
  }

  return sum / count;
}

void startTurnLeft() {
  currentMode = MODE_TURN_LEFT;
  modeStartTime = millis();
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

  // 如果三个方向都没有数据，不要盲目前进，先停车
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

  setSpeed(BASE_SPEED, lateralSpeed, 0);
}

//下面是转弯模式喵

void handleTurnLeft() {
  if (millis() - modeStartTime < TURN_DURATION) {
    turnLeft(TURN_SPEED);
  } else {
    stopCar();
    currentMode = MODE_RUN;
    modeStartTime = millis();
    DBG_PRINTLN("左转完了，重新直行");
  }
}

void handleTurnRight() {
  if (millis() - modeStartTime < TURN_DURATION) {
    turnRight(TURN_SPEED);
  } else {
    stopCar();
    currentMode = MODE_RUN;
    modeStartTime = millis();
    DBG_PRINTLN("右转结束，回到直行");
  }
}

//进入状态

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
      lastGoodLidarTime = millis();
    }
  } else {
    // 偶尔读取失败不用管，只有长时间没有成功读到点才重启
    if (millis() - lastGoodLidarTime > 1500 && millis() - lastLidarRestartTime > 1500) {
      stopCar();
      lidar.startScan();
      lastLidarRestartTime = millis();
      DBG_PRINTLN("雷达长时间无有效点，重启扫描");
    }
  }

  // 控制逻辑不再依赖 isNewScan，每 50ms 执行一次
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
    }
  }
}