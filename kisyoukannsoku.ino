#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


#define num 400

float data[4][num];

float sin0 = 0;
float cos0 = 0;
float radian = 0;
float speed = 0;  //m/s

float calibration0 = 0;
float calibration1 = 0;
float calibration2 = 0;
float calibration3 = 0;



void calibration() {
  long long sum0 = 0;
  long long sum1 = 0;
  long long sum2 = 0;
  long long sum3 = 0;

  for (int i = 0; i < num; i++) {

    data[0][1] = analogRead(15);
    data[1][1] = analogRead(2);
    data[2][1] = analogRead(0);
    data[3][1] = analogRead(4);

    sum0 += data[0][1];
    sum1 += data[1][1];
    sum2 += data[2][1];
    sum3 += data[3][1];

    for (int i = num - 1; i > 1; i--) {
      for (int j = 0; j < 4; j++) {
        data[j][i] = data[j][i - 1];
      }
    }
  }

  calibration0 = 1645 - sum0 / num;
  calibration1 = 1645 - sum1 / num;
  calibration2 = 1645 - sum2 / num;
  calibration3 = 1645 - sum3 / num;
}



void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(LED_PIN, OUTPUT);

  // 初期化
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < num; j++) {
      data[i][j] = 0;
    }
  }

  calibration();

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "esp32_publisher"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // メッセージの初期化
  msg.data.size = 2; // データサイズを設定
  msg.layout.data_offset = 0;
}

void loop() {
  data[0][1] = analogRead(15) + calibration0;
  data[1][1] = analogRead(2) + calibration1;
  data[2][1] = analogRead(0) + calibration2;
  data[3][1] = analogRead(4) + calibration3;

  // データ更新処理
  for (int i = num - 1; i > 1; i--) {
    for (int j = 0; j < 4; j++) {
      data[j][i] = data[j][i - 1];
    }
  }

  for (int i = 0; i < 4; i++) {
    long long sum = 0;
    for (int j = 1; j < num; j++) {
      sum += data[i][j];
    }

    float wind = sum / num;
    if (wind > 1620 && wind < 1670) {
      data[i][0] = 0;
    } else if (wind >= 1670) {
      data[i][0] = ((sum / num) - 1531.3) / 62.815;
    } else {
      data[i][0] = ((sum / num) - 1798.7) / 79.04;
    }
  }

  float tmp_sin = (data[2][0] + data[1][0] * sqrt(2) + data[3][0] * sqrt(2)) / 3.00;
  float tmp_cos = (data[0][0] + data[1][0] * sqrt(2) - data[3][0] * sqrt(2)) / 3.00;

  sin0 = tmp_sin / sqrt(pow(tmp_sin, 2) + pow(tmp_cos, 2));
  cos0 = tmp_cos / sqrt(pow(tmp_sin, 2) + pow(tmp_cos, 2));

  if (cos0 >= 0) {
    radian = asin(sin0);
  } else {
    radian = 3.1416 - asin(sin0);
  }

  if (radian < 0) {
    radian += 6.2832;
  }

  speed = sqrt(pow(tmp_sin, 2) + pow(tmp_cos, 2));

  // メッセージにデータを設定
  msg.data.data = (float *)malloc(2 * sizeof(float));
  msg.data.size = 2;
  msg.data.capacity = 2;
  msg.data.data[0] = radian;
  msg.data.data[1] = speed;

  // メッセージ送信
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  // Executorを実行
  //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
