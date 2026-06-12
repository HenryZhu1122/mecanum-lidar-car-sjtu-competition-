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

const float FRONT_TURN_THRESH = 310;  //前方最小
const float SIDE_TURN_THRESH = 50;
const float SIDE_NEAR_THRESH = 170;   //侧方最小
const float SIDE_DIFF_MARGIN = 100;    //纠偏最小
const float SIDE_DIFFERENCE = 0;
const float SIDE_CORRECTMARGIN = 400;
// ===== 正常直行角度修正参数 =====
// 通过比较 80~90 和 90~100 的雷达距离，判断车头是否斜了
const int ANGLE_FIX_A_START = 80;
const int ANGLE_FIX_A_END   = 90;
const int ANGLE_FIX_B_START = 90;
const int ANGLE_FIX_B_END   = 100;

const float ANGLE_FIX_KP = 0.9;        // 角度修正比例系数，先小一点
const float ANGLE_FIX_MAX = 20.0;       // 最大角速度修正
const float ANGLE_FIX_DEADZONE = 0.0;  // 两段距离差小于40mm就不修
const float ANGLE_FIX_SIGN = 1.0;       // 如果修正方向反了，改成 -1.0

const float VALID_MIN = 0;           // 搞个下界就够了

const int AVG_RANGE = 20;              // 取中心角左右
const unsigned long TURN_DURATION = 880;  // 转弯持续时间
const unsigned long POST_TURN_RUN = 500;  // 转后强制直行时间

// ===== 绕8新增参数 =====
const float EIGHT_OPEN_THRESH = 500;  // 判断某一侧是否开阔
const unsigned long EIGHT_PRE_TURN_RUN = 650;  // 识别到中间开口后，先直行0.5秒再转
const unsigned long EIGHT_LOCK_TIME =700;  // 绕8路口锁定时间，防止同一路口重复触发

#define EIGHT_LEFT  -1
#define EIGHT_RIGHT  1

const int CENTER_ROUTE_COUNT = 3;

// 只用于“前方堵住，左右都开阔”的情况
// 第一次左转，第二次右转，第三次左转，第四次右转……
const int centerRoute[CENTER_ROUTE_COUNT] = {
  EIGHT_LEFT,
  EIGHT_RIGHT,
  EIGHT_LEFT,
};

#define MODE_RUN               0
#define MODE_TURN_LEFT         1
#define MODE_TURN_RIGHT        2
#define MODE_POST_TURN         3
#define MODE_EIGHT_DELAY_TURN  4
#define MODE_TURNBACK          5

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

// ===== 绕8新增状态变量 =====
int centerRouteStep = 0;       // 记录“前方堵住，左右都开阔”出现了几次
bool eightLocked = false;      // 防止同一个路口重复触发
unsigned long eightLockTime = 0;  // 绕8路口锁定开始时间
bool lastLeftOpen = false;     // 上一轮左侧是否开阔，用于判断“突然开阔”
bool lastRightOpen = false;    // 上一轮右侧是否开阔，用于判断“突然开阔”
int pendingEightDirection = 0; // 中间开口延迟转弯时，暂存要转的方向


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

float getRangeAvgDistance(int startAngle, int endAngle) {
  float sum = 0;
  int count = 0;

  for (int angle = startAngle; angle <= endAngle; angle++) {
    int realAngle = (angle + 360) % 360;
    float d = lidarDistances[realAngle];

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

float getAngleCorrectSpeed() {
  float a = getRangeAvgDistance(ANGLE_FIX_A_START, ANGLE_FIX_A_END);
  float b = getRangeAvgDistance(ANGLE_FIX_B_START, ANGLE_FIX_B_END);

  if (a < 0 || b < 0) {
    return 0;
  }

  float diff = a - b;

  if (fabs(diff) < ANGLE_FIX_DEADZONE) {
    return 0;
  }

  float angularW = diff * ANGLE_FIX_KP * ANGLE_FIX_SIGN;

  angularW = constrain(angularW, -ANGLE_FIX_MAX, ANGLE_FIX_MAX);

  return angularW;
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

// ===== 绕8新增函数开始 =====

bool isEightOpen(float d) {
  return d > EIGHT_OPEN_THRESH;
}

void updateEightOpenState(float left, float right) {
  lastLeftOpen = isEightOpen(left);
  lastRightOpen = isEightOpen(right);
}

void startEightTurn(int direction) {
  if (direction == EIGHT_LEFT) {
    DBG_PRINTLN("绕8判断：左转");
    startTurnLeft();
  } else if (direction == EIGHT_RIGHT) {
    DBG_PRINTLN("绕8判断：右转");
    startTurnRight();
  }
}

// 类型1：前方堵住，左右都开阔
// 这个时候按记忆表转弯
bool isCenterMemoryJunction(float front, float left, float right) {
  if (front < 0 || left < 0 || right < 0) {
    return false;
  }

  return front < FRONT_TURN_THRESH+40 &&
         isEightOpen(left) &&
         isEightOpen(right);
}

// 类型2：前方开阔，左边或右边突然开阔
// 这个时候向突然开阔的一侧转
bool isSideSuddenlyOpenJunction(float front, float left, float right) {
  if (front < 0 || left < 0 || right < 0) {
    return false;
  }

  bool leftOpen = isEightOpen(left);
  bool rightOpen = isEightOpen(right);

  bool leftSuddenlyOpen = leftOpen && !lastLeftOpen && !rightOpen;
  bool rightSuddenlyOpen = rightOpen && !lastRightOpen && !leftOpen;

  return front >= FRONT_TURN_THRESH*3 &&
         (leftSuddenlyOpen || rightSuddenlyOpen);
}

int getSuddenlyOpenDirection(float left, float right) {
  bool leftOpen = isEightOpen(left);
  bool rightOpen = isEightOpen(right);

  bool leftSuddenlyOpen = leftOpen && !lastLeftOpen && !rightOpen;
  bool rightSuddenlyOpen = rightOpen && !lastRightOpen && !leftOpen;

  if (leftSuddenlyOpen) {
    return EIGHT_LEFT;
  }

  if (rightSuddenlyOpen) {
    return EIGHT_RIGHT;
  }

  return 0;
}

// 尝试处理绕8判断
// 返回 true 表示已经触发绕8转向，后面的普通L弯逻辑不要执行
bool tryHandleEight(float front, float left, float right) {
  bool centerMemoryJunction = isCenterMemoryJunction(front, left, right);
  bool sideSuddenlyOpenJunction = isSideSuddenlyOpenJunction(front, left, right);

  bool leftOpenNow = isEightOpen(left);
  bool rightOpenNow = isEightOpen(right);

  // 如果刚刚识别过一个绕8路口，就先按时间锁住
  // 锁定期间不让普通L弯逻辑抢走判断
  if (eightLocked) {
    if (millis() - eightLockTime >= EIGHT_LOCK_TIME) {
      eightLocked = false;
      DBG_PRINTLN("绕8路口锁按时间解除");
    } else {
      updateEightOpenState(left, right);
      setSpeed(BASE_SPEED, 0, 0);
      return true;
    }
  }

  // 类型1：前方堵住，左右都开阔
  // 按记忆表转弯
  if (centerMemoryJunction) {
    int direction = centerRoute[centerRouteStep % CENTER_ROUTE_COUNT];

    centerRouteStep++;
    eightLocked = true;
    eightLockTime = millis();

    DBG_PRINT("识别到绕8中心记忆路口，次数：");
    DBG_PRINTLN(centerRouteStep);

    if(centerRouteStep == 3){
    currentMode = MODE_TURNBACK;
    modeStartTime = millis();
    centerRouteStep=0;
    return true;
    }
    else if (direction == EIGHT_LEFT) {
      DBG_PRINTLN("记忆表决定：左转");
    } else {
      DBG_PRINTLN("记忆表决定：右转");
    }

    updateEightOpenState(left, right);
    startEightTurn(direction);
    return true;
  }

  // 类型2：前方开阔，左边或右边突然开阔
  // 不立刻转，先继续直行0.5秒，再向突然开阔的一侧转
  if (sideSuddenlyOpenJunction) {
    int direction = getSuddenlyOpenDirection(left, right);

    eightLocked = true;
    eightLockTime = millis();
    pendingEightDirection = direction;

    if (direction == EIGHT_LEFT) {
      DBG_PRINTLN("识别到左侧突然开阔，先直行0.5秒再左转");
    } else if (direction == EIGHT_RIGHT) {
      DBG_PRINTLN("识别到右侧突然开阔，先直行0.5秒再右转");
    }

    updateEightOpenState(left, right);

    currentMode = MODE_EIGHT_DELAY_TURN;
    modeStartTime = millis();

    return true;
  }

  updateEightOpenState(left, right);
  return false;
}

// ===== 绕8新增函数结束 =====

void handleEightDelayTurn() {
  setSpeed(BASE_SPEED, 0, 0);

  if (millis() - modeStartTime >= EIGHT_PRE_TURN_RUN) {
    stopCar();

    DBG_PRINTLN("中间开口延迟直行结束，开始转弯");

    startEightTurn(pendingEightDirection);
  }
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

  // ===== 绕8新增判断 =====
  // 只加在普通L弯判断前面，其他逻辑不动
  if (tryHandleEight(front, left, right)) {
    return;
  }

  if (front > 0 && front < FRONT_TURN_THRESH && (left>= SIDE_TURN_THRESH || right >= SIDE_TURN_THRESH) &&(left<= 500 || right <= 500)) {
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

  if (left > 0 && right > 0 && front >= FRONT_TURN_THRESH ) {
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
    else if (((left - right) >= SIDE_DIFFERENCE || (right - left) >= SIDE_DIFFERENCE )&& front >= 2* FRONT_TURN_THRESH && left <= SIDE_CORRECTMARGIN && right<= SIDE_CORRECTMARGIN)
    {
      lateralSpeed = -(left-right)/3;
      DBG_PRINTLN("两边距离相差太大，开始自动纠偏");
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
  float angularCorrect = 0;

  if (front >= 2.5 * FRONT_TURN_THRESH) {
  angularCorrect = getAngleCorrectSpeed();
  }

  setSpeed(BASE_SPEED, lateralSpeed, angularCorrect);
}

void handleturnback(){
  if (millis() - modeStartTime < 2*TURN_DURATION) {
    turnRight(TURN_SPEED);
  } else {
    stopCar();
    currentMode = MODE_POST_TURN;
    modeStartTime = millis();
    DBG_PRINTLN("掉头结束，进入掉头直行");
  }
  return;
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

    eightLocked = true;
    eightLockTime = millis();

    currentMode = MODE_RUN;
    modeStartTime = millis();
    DBG_PRINTLN("转弯后直行结束，回到正常模式，并短暂锁定绕8判断");
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

      case MODE_EIGHT_DELAY_TURN:
        handleEightDelayTurn();
        break;
      case MODE_TURNBACK:
        handleturnback();
        break;
    }
  }
}