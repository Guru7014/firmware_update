//23-7-25 Lora communication done with increased speed and ensuring the parameter remains the same at any point
//24-7-25 RTC external and internal clock developed in case of no external clock, clock sync will sync the time of the pcb
//25-7-25 File system with all the data checks properly and increased the features for future,
//        also stored previous data in powerloss, also improved lora communication based on aux feedback
//26-7-25 upgrade the logic for robot movement
//28-7-25 upgrade the logic for motor over current protection and schedulling of the robot along with sensor fault and distance
//        limit
#include <HardwareSerial.h>
#include <esp_task_wdt.h>
// #include "uRTCLib.h"
#include "RTC_clock.h"
#include "LittleFS.h"
#include <esp_timer.h>
HardwareSerial LoRaSerial(2);
// uRTCLib rtc(0x68);
RTC_clock rtcClock;

#define WDT_TIMEOUT 10

//===================================================
//      Pinout PCB v3.0, v2.8, v2.6
#define dir_pin 32
#define brush_pwm_pin 33
#define wheel_pwm_pin 25
#define windlock_pin 13
#define indication_pin 12

#define left_sensor_pin 14
#define right_sensor_pin 15
#define left_direction_pin 26
#define right_direction_pin 27
#define manual_switch_pin 23
#define reset_switch_pin 4
#define select_pin 2

#define battery_voltage_pin 36
#define charger_current_pin 39
#define brush_current_pin 34
#define wheel_current_pin 35

#define M0 19
#define M1 18
#define AUX_PIN 5
//======================================================

//======================================================
//    Pinout v2.4
// #define dir_pin 32
// #define brush_pwm_pin 33
// #define wheel_pwm_pin 25
// #define windlock_pin 13.
// #define indication_pin 12

// #define left_sensor_pin 27
// #define right_sensor_pin 26
// #define left_direction_pin 14
// #define right_direction_pin 15
// #define manual_switch_pin 23
// #define reset_switch_pin 4
// #define select_pin 2

// #define M0 19
// #define M1 18
// #define AUX_PIN 5

// #define battery_voltage_pin 39
// #define charger_current_pin 36
// #define brush_current_pin 34
// #define wheel_current_pin 35
// //==================================================

#define transmitter true
#define receiver false
#define RIGHT HIGH
#define LEFT LOW

#define fast_pwm 245
#define slow_pwm 245

#define address_byte 0
#define running_mode_byte 1
#define timer1_u_byte 2
#define timer1_l_byte 3
#define timer2_u_byte 4
#define timer2_l_byte 5
#define timer3_u_byte 6
#define timer3_l_byte 7
#define timer_trigger_byte 8
#define system_commission_u_byte 9
#define system_commission_l_byte 10
#define battery_commission_u_byte 11
#define battery_commission_l_byte 12
#define motor_commission_u_byte 13
#define motor_commission_l_byte 14
#define brush_commission_u_byte 15
#define brush_commission_l_byte 16
#define function_u_byte 17
#define function_l_byte 18
#define brush_curr_limit_byte 19
#define wheel_curr_limit_byte 20
#define battery_curr_limit_byte 21
#define runtime_u_byte 22
#define runtime_l_byte 23
#define run_status_byte 24
#define command_status_byte 25
#define motor_status_byte 26
#define bat_sens_status_byte 27
#define upp_motor_left_limit_byte 28
#define upp_motor_right_limit_byte 29
#define curr_sens_left_byte 30
#define curr_sens_right_byte 31
#define sens_delay_left_byte 32
#define sens_delay_right_byte 33
#define distance_limit_upper_byte 34
#define distance_limit_lower_byte 35
#define frequency_upper_byte 36
#define frequency_lower_byte 37
#define lora_delay_upper_byte 38
#define lora_delay_lower_byte 39
#define curr_pos_upper_byte 40
#define curr_pos_lower_byte 41

#define brush_channel 0
#define wheel_channel 1
#define resolution 8
// #define frequency 11000

#define wheel_rampup_time 2000
#define brush_rampup_time 2000
#define brush_rampdown_time 1500
#define stop_wait_time 3000
#define windlock_time 7000
#define critical_error_time 1000

#define critical_brush_current 10.0f
#define critical_wheel_current 10.0f
#define check_battery 1
#define check_capacity 1
#define battey_min_voltage 20.0f

TaskHandle_t Task1, Task2, Task3;

uint8_t month = 1, date = 1, hour = 0, minute = 0, second = 0, prev_RTC_second = 0;
uint16_t sync_year = 0, year = 2000;
uint8_t sync_month = 0, sync_date = 0, sync_hour = 0, sync_minute = 0;
uint64_t prev_esp_timer = 0;
bool direction_changed = 0;
uint8_t fwd_pwm = 255, bwd_pwm = 255;
uint8_t lora_channel = 0, SPED = 0, OPTION = 0;
bool lora_error = 0;
bool hold_brush = 0;
uint16_t docking_time = 4000;
uint16_t error_limit_time = 4000;
const float VBE_25 = 590.0;
const float SLOPE = -2.0;
float temperature = 0;
struct Record {
  uint16_t record_num;
  uint8_t record_data[35];
};

uint16_t current_clean_record_num = 0, current_alarm_record_num = 0;
const char *clean_record_file_path = "/clean_records.bin";
const char *alarm_record_file_path = "/alarm_records.bin";
uint8_t NodeID = 0, timer_counter = 0, trigger_counter = 0, running_mode = 0;
uint8_t brush_current_limit = 0, wheel_current_limit = 0, battery_current_limit = 0;
float brush_limit_value = 5.0, wheel_limit_value = 2.0, battery_limit_value = 12.0;
float current_sensor_factor = 0.0542f, current_ref_val = 2.45f;

uint16_t timer1 = 0, timer2 = 0, timer3 = 0, function = 0;
uint8_t trigger_counter1 = 0, trigger_counter2 = 0, trigger_counter3 = 0;
uint16_t system_commission = 0, battery_commission = 0, motor_commission = 0, brush_commission = 0;
uint8_t send_BVOL = 0, send_DRV1 = 0, send_DRV2 = 0, send_DRV3 = 0, send_DRV4 = 0, send_DRV5 = 0;
uint8_t send_DRV6 = 0, send_BSOC = 0, send_CURH = 0, send_CURL = 0, send_SENS = 0, send_YRMO = 0;
uint8_t send_DATE = 0, send_HOUR = 0, send_MIN = 0, send_CMDS = 0, send_RUNS = 0, send_MOTS = 0;
uint8_t send_BATS = 0, send_SEST = 0, send_SRVS = 0, send_LSRT = 0, send_CPSH = 0, send_CPSL = 0;
bool manual_run = 0, auto_run = 0, manual_fwd = 0, manual_bwd = 0, clock_sync = 0, time_stamp = 0;
bool remote_run = 0, remote_stop = 0, all_left = 0, all_right = 0, all_run = 0, all_stop = 0;
bool remote_fwd = 0, remote_bwd = 0, remote_cycle = 0;
bool robot_dir_change = 0, reverse_flag = 0;
bool roll_CW = 0, roll_CCW = 0, docking_delay = 0, roll_self_clean = 0, roll_anti_dir = 0, show_charigng_Ah = 0;
bool moving_right_flag = 0, moving_left_flag = 0, roll_anti_dir_stop = 0;
bool prev_moving_right_flag = 0, prev_moving_left_flag = 0, prev_is_running = 0;
uint8_t docking_mode = 0;
uint16_t travelled_distance = 0;
uint8_t last_RT_min = 0, run_status = 0, command_status = 0, motor_status = 0, bat_sens_status = 0;
uint8_t upp_motor_left_limit = 0, upp_motor_right_limit = 0, curr_sens_left = 0;
uint8_t curr_sens_right = 0, sens_delay_left = 0, sens_delay_right = 0;
uint16_t distance_limit = 0;
uint16_t frequency = 11000;
uint16_t lora_delay = 200;
bool cycle_completed = 0, half_cycle_completed = 0;

bool is_running = 0, transmit_flag = 0;
uint8_t data1[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00 };
uint8_t data2[] = { 0x00, 0x00, lora_channel };
uint8_t data3[] = { 0x00, 0x00 };
uint8_t basic_parameters[] = { 0x00, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x03, 0x00, 0x50, 0x23, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x32,
                               0x0C, 0x04, 0x03, 0xE8, 0x0B, 0x00, 0x02, 0x00, 0x00, 0x00 };

uint8_t brush_error_count = 0, wheel_error_count = 0;
float battery_voltage = 0, charger_current = 0, brush_current = 0, wheel_current = 0;
float ref_brush_current = 0, ref_wheel_current = 0, ref_charger_current = 0;
float record_brush_current = 0, record_wheel_current = 0;
float current_ref_val1 = 2.5, current_ref_val2 = 2.5, current_ref_val3 = 2.5;
uint8_t read_counts = 11;
float alpha = 0.2, raw_value_1 = 0, raw_value_2 = 0, raw_value_3 = 0, raw_value_4 = 0;
bool battery_error_flag = 0, brush_error_flag = 0, wheel_error_flag = 0;
bool battery_current_flag = 0;
float brush_consumed_Ah = 0, wheel_consumed_Ah = 0;
float charging_Ah = 0.0f;

bool auto_scheduled = 0;
uint8_t SC_hour1 = 0, SC_minute1 = 0;  //need to change
uint8_t SC_hour2 = 0, SC_minute2 = 0;  //need to change
uint8_t SC_hour3 = 0, SC_minute3 = 0;  //need to change
int16_t SC_hour = 0, SC_minute = 0;
uint8_t default_wheel_pwm = 250;
uint8_t default_brush_pwm = 250;
uint8_t left_minus_speed = 0;
uint8_t right_minus_speed = 0;
bool vary_speed_control = 0;
uint16_t run_time = 0, brush_run_time = 0, wheel_run_time = 0;

bool flag_brush = 1, end_flag = 0;
bool abnormal_pos = 0, return_pos = 0, parking_pos = 0;
uint8_t move_fwd = 0, move_bwd = 0;
uint8_t direction_flag = 0;
uint32_t prev_stop_millis = 0, prev_ramp_millis = 0, prev_second = 0, prev_roll_anti_dir_millis = 0;
uint32_t prev_wheel_millis = 0, prev_brush_millis = 0, prev_roll_millis = 0;
uint32_t prev_indication_millis = 0, prev_fetched_time = 0, prev_read_millis = 0;
uint32_t prev_running_millis = 0, prev_wheel_error_millis = 0, prev_brush_error_millis = 0;
int wheel_pwm = default_wheel_pwm, br_pwm = default_brush_pwm, ramp_pwm = 0, ramp_brush = 0;
bool direction_fetched = 0;

static SemaphoreHandle_t statusMutex;

void waitForAUX(uint32_t);

void setup() {
  Serial.begin(115200);
  output_pins_declaration();
  digitalWrite(windlock_pin, HIGH);
  input_pins_declaration();
  analogSetPinAttenuation(select_pin, ADC_11db);
  statusMutex = xSemaphoreCreateMutex();
  if (statusMutex == NULL) {
    // Handle error - out of memory?
    printf("Failed to create mutex\n");
    while (1)
      ;  // Halt or blink error LED
  }

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);  // Add main task

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  read_parameters();

  Serial.printf("Total size: %u bytes\n", LittleFS.totalBytes());
  Serial.printf("Used size: %u bytes\n", LittleFS.usedBytes());
  lora_init();
  // RTC_init();
  rtcClock.begin();
  prev_esp_timer = esp_timer_get_time();
  current_clean_record_num = getCurrentRecordNumber(clean_record_file_path);
  current_alarm_record_num = getCurrentRecordNumber(alarm_record_file_path);

  if (xTaskCreatePinnedToCore(
        read_sensor_values,
        "read_sensor_values",
        8192,
        NULL,
        1,
        &Task2,
        1)
      != pdPASS) {
    Serial.println("Failed to create sensors task");
    ESP.restart();
  }
  prev_wheel_millis = millis();
  prev_brush_millis = millis();
  vTaskDelay(3000 / portTICK_PERIOD_MS);

  if (xTaskCreatePinnedToCore(
        clock_read,
        "clock_read",
        4096,
        NULL,
        1,
        &Task1,
        0)
      != pdPASS) {
    Serial.println("Failed to create time task");
    ESP.restart();
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  // ref_charger_current = charger_current;
  ref_brush_current = brush_current;
  ref_wheel_current = wheel_current;
  ref_charger_current = (ref_brush_current + ref_wheel_current) / 2.0f;

  if (xTaskCreatePinnedToCore(
        robot_task,
        "Robot_task",
        8192,
        NULL,
        1,
        &Task1,
        0)
      != pdPASS) {
    Serial.println("Failed to create robot task");
    ESP.restart();
  }

  vTaskDelay(50 / portTICK_PERIOD_MS);
  prev_stop_millis = millis();
  prev_ramp_millis = millis();
  prev_read_millis = millis();
  prev_indication_millis = millis();
  prev_roll_millis = millis();
  // RTC_error = 1;
}

void loop() {
  // internal_clock();
  esp_task_wdt_reset();
  // Serial.println("lora_running");
  vTaskDelay(10 / portTICK_PERIOD_MS);
  LoRa_functions();
}

void output_pins_declaration() {
  pinMode(dir_pin, OUTPUT);
  pinMode(wheel_pwm_pin, OUTPUT);
  pinMode(brush_pwm_pin, OUTPUT);
  pinMode(windlock_pin, OUTPUT);
  pinMode(indication_pin, OUTPUT);
  ledcSetup(0, frequency, resolution);
  ledcSetup(1, frequency, resolution);
  ledcAttachPin(brush_pwm_pin, brush_channel);
  ledcWrite(brush_channel, 0);
  ledcAttachPin(wheel_pwm_pin, wheel_channel);
  ledcWrite(wheel_channel, 0);
}

void input_pins_declaration() {
  pinMode(left_sensor_pin, INPUT);
  digitalWrite(left_sensor_pin, HIGH);
  pinMode(right_sensor_pin, INPUT);
  digitalWrite(right_sensor_pin, HIGH);
  pinMode(left_direction_pin, INPUT);
  digitalWrite(left_direction_pin, HIGH);
  pinMode(right_direction_pin, INPUT);
  digitalWrite(right_direction_pin, HIGH);
  pinMode(manual_switch_pin, INPUT);
  digitalWrite(manual_switch_pin, HIGH);
  pinMode(reset_switch_pin, INPUT);
  digitalWrite(reset_switch_pin, HIGH);
}

void set_command_status(uint8_t cmd_bits) {
  if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    send_CMDS = cmd_bits;
    Serial.print("Setting CMDS: 0x");
    Serial.println(send_CMDS, HEX);
    xSemaphoreGive(statusMutex);
  }
}

void robot_task(void *parameter) {
  uint32_t lastWdtReset = 0;
  esp_task_wdt_add(xTaskGetCurrentTaskHandle());
  while (1) {
    if ((long)(millis() - lastWdtReset) > 1000) {
      esp_task_wdt_reset();
      lastWdtReset = millis();
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    converting_pwm();
    abnormal_pos = bool(!digitalRead(left_sensor_pin) && !digitalRead(right_sensor_pin) && !(moving_right_flag || moving_left_flag));
    return_pos = 0;
    parking_pos = 0;

    if (!abnormal_pos) {
      return_pos = digitalRead(right_sensor_pin);
      parking_pos = digitalRead(left_sensor_pin);
    } else {
      return_pos = 0;
      parking_pos = 0;
    }

    if (roll_anti_dir_stop && (millis() - prev_roll_anti_dir_millis > docking_time + 2000)) {
      roll_anti_dir_stop = 0;
    }

    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      uint8_t cycle_status = 0x00;
      if (docking_mode == 1) {
        cycle_status = (cycle_completed << 1) | (half_cycle_completed << 2);
      } else {
        cycle_status = (cycle_completed << 2) | (half_cycle_completed << 1);
      }
      send_SEST = ((digitalRead(right_sensor_pin) << 0) | (digitalRead(left_sensor_pin) << 3) | cycle_status);
      send_RUNS = ((time_stamp << 6) | (auto_run << 7) | (abnormal_pos << 4) | (return_pos << 3) | (moving_right_flag << 2) | (moving_left_flag << 1) | (parking_pos << 0));
      xSemaphoreGive(statusMutex);
    }

    if (digitalRead(windlock_pin)) {
      // ref_charger_current = charger_current;
      if (millis() - prev_second > windlock_time) digitalWrite(windlock_pin, LOW);
    } else {
      prev_second = millis();
    }

    if (roll_CCW && !is_running) {
      digitalWrite(dir_pin, HIGH);
      ledcWrite(brush_channel, 255);  //need to comment
      if (millis() - prev_roll_millis > 4000) {
        ledcWrite(brush_channel, 0);  //need to comment
        roll_CCW = 0;
      }
    } else if (roll_CW && !is_running) {
      digitalWrite(dir_pin, LOW);
      ledcWrite(brush_channel, 255);  //need to comment
      if (millis() - prev_roll_millis > 4000) {
        ledcWrite(brush_channel, 0);  //need to comment
        roll_CW = 0;
      }
    } else {
      roll_CCW = 0;
      roll_CW = 0;
      prev_roll_millis = millis();
    }

    if (battery_error_flag && (move_fwd || move_bwd) && check_battery) {
      write_parameters();
      current_alarm_record_num++;
      write_alarm_record(current_alarm_record_num);
      end_flag = 1;
      move_fwd = 0;
      move_bwd = 0;
    }

    if (brush_error_flag && wheel_error_flag) {
      if (millis() - prev_indication_millis > 100) {
        digitalWrite(indication_pin, !digitalRead(indication_pin));
        prev_indication_millis = millis();
      }
    } else if (brush_error_flag) {
      if (millis() - prev_indication_millis > 150) {
        digitalWrite(indication_pin, !digitalRead(indication_pin));
        prev_indication_millis = millis();
      }
    } else if (wheel_error_flag) {
      if (millis() - prev_indication_millis > 250) {
        digitalWrite(indication_pin, !digitalRead(indication_pin));
        prev_indication_millis = millis();
      }
    } else if (battery_error_flag) {
      if (millis() - prev_indication_millis > 2000) {
        digitalWrite(indication_pin, !digitalRead(indication_pin));
        prev_indication_millis = millis();
      }
    } else if (rtcClock.rtc_error_flag) {
      if (millis() - prev_indication_millis > 3000) {
        digitalWrite(indication_pin, !digitalRead(indication_pin));
        prev_indication_millis = millis();
      }
    } else {
      if (millis() - prev_indication_millis > 500) {
        digitalWrite(indication_pin, !digitalRead(indication_pin));
        prev_indication_millis = millis();
      }
    }

    if (!is_running) {
      bool manual_triggered = (millis() - prev_fetched_time > 500);
      if ((all_run || remote_run)) {
        set_command_status(1 << 0);
        // Serial.print("0x");
        // Serial.println(send_CMDS, HEX);
        all_run = 0;
        remote_run = 0;
        if (half_cycle_completed && !cycle_completed) start_movement(0, 1, 0);
        else start_movement(1, 1, 0);
      } else if ((all_left)) {
        if (docking_mode == 0) {
          set_command_status(1 << 2);
          start_movement(0, 1, 0);
        } else if (docking_mode == 1) {
          set_command_status(1 << 3);
          start_movement(1, 0, 0);
        } else {
          set_command_status(1 << 2);
          start_movement(0, 1, 0);
        }
        all_left = 0;
      } else if ((all_right)) {
        if (docking_mode == 0) {
          set_command_status(1 << 3);
          start_movement(1, 0, 0);
        } else if (docking_mode == 1) {
          set_command_status(1 << 2);
          start_movement(0, 1, 0);
        } else {
          set_command_status(1 << 3);
          start_movement(1, 0, 0);
        }
        all_right = 0;
      } else if (!digitalRead(left_direction_pin)) {
        if (manual_triggered) {
          set_command_status((1 << 7) | (1 << 5));
          if (docking_mode == 2) start_movement(1, 0, 0);
          else {
            if (half_cycle_completed && !cycle_completed) start_movement(0, 1, 0);
            else start_movement(1, 1, 0);
          }
        }
      } else if (!digitalRead(right_direction_pin)) {
        if (manual_triggered) {
          set_command_status(1 << 4);
          start_movement(0, 1, 0);
        }
      } else if (!digitalRead(manual_switch_pin)) {
        if (manual_triggered) {
          set_command_status(1 << 5);
          start_movement(1, 0, 0);
        }
      } else if (remote_fwd) {
        set_command_status(1 << 3);
        remote_fwd = 0;
        start_movement(1, 0, 0);
      } else if (remote_bwd) {
        set_command_status(1 << 2);
        remote_bwd = 0;
        start_movement(0, 1, 0);
      } else if (!digitalRead(reset_switch_pin)) {
        if (millis() - prev_fetched_time > 2000) {
          erase_parameters();
          vTaskDelay(pdMS_TO_TICKS(50));  //need to comment
          prev_fetched_time = millis();
        }
      } else prev_fetched_time = millis();

      moving_left_flag = 0;
      moving_right_flag = 0;
      ref_brush_current = brush_current;
      ref_wheel_current = wheel_current;
      prev_brush_error_millis = millis();
    } else {
      moving_left_flag = !digitalRead(dir_pin);
      moving_right_flag = digitalRead(dir_pin);

      if (!end_flag) {
        if (move_fwd) {
          if (docking_mode == 0) {
            move_robot(RIGHT);
          } else if (docking_mode == 1) {
            move_robot(LEFT);
          } else {
            if (digitalRead(right_sensor_pin)) move_fwd = 0;
            else {
              move_robot(RIGHT);
              move_bwd = 0;
            }
          }
        } else if (move_bwd) {
          if (docking_mode == 0) {
            move_robot(LEFT);
          } else if (docking_mode == 1) {
            move_robot(RIGHT);
          } else {
            move_robot(LEFT);
          }
        } else {
          if (docking_delay && (!(all_stop || remote_stop))) {
            docking_time = sens_delay_left * 300;
            if (millis() - prev_stop_millis > docking_time) {
              ledcWrite(wheel_channel, 0);
              ledcWrite(brush_channel, 0);
              write_parameters();
              if (cycle_completed) {
                current_clean_record_num++;
                write_clean_record(current_clean_record_num);
              }
              if ((send_MOTS != 0x00) | (send_BATS != 0x00) | (send_SENS != 0x00)) {
                current_alarm_record_num++;
                write_alarm_record(current_alarm_record_num);
              }
              is_running = 0;
            } else {
              if (docking_mode == 0) {
                if (!digitalRead(right_sensor_pin)) {
                  digitalWrite(dir_pin, HIGH);
                  ledcWrite(wheel_channel, wheel_pwm * (!wheel_error_flag));
                } else ledcWrite(wheel_channel, 0);
              } else if (docking_mode == 1) {
                if (!digitalRead(left_sensor_pin)) {
                  digitalWrite(dir_pin, LOW);
                  ledcWrite(wheel_channel, wheel_pwm * (!wheel_error_flag));
                } else ledcWrite(wheel_channel, 0);
              }
            }
          } else {
            ledcWrite(wheel_channel, 0);
            ledcWrite(brush_channel, 0);
            if (docking_delay) {
              all_stop = 0;
              remote_stop = 0;
            }
            write_parameters();
            if (cycle_completed) {
              current_clean_record_num++;
              write_clean_record(current_clean_record_num);
            }
            if ((send_MOTS != 0x00) | (send_BATS != 0x00) | (send_SENS != 0x00)) {
              current_alarm_record_num++;
              write_alarm_record(current_alarm_record_num);
            }
            is_running = 0;
          }
        }
      } else {
        stop_moving();
      }

      if ((all_stop || remote_stop) && (move_fwd || move_bwd)) {
        // set_command_status(1 << 1);
        if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          send_CMDS |= (1 << 1);
          xSemaphoreGive(statusMutex);
        }
        if (!docking_delay) {
          all_stop = 0;
          remote_stop = 0;
        }
        end_flag = 1;
        move_fwd = 0;
        move_bwd = 0;
      }
    }
  }
}

void start_movement(uint8_t fwd, uint8_t bwd, bool run_mode) {
  brush_error_flag = 0;
  wheel_error_flag = 0;
  brush_error_count = 0;
  wheel_error_count = 0;
  hold_brush = 0;
  SC_hour = rtcClock.dt.hour;
  SC_minute = rtcClock.dt.minute;
  brush_run_time = 0;
  wheel_run_time = 0;
  brush_consumed_Ah = 0;
  wheel_consumed_Ah = 0;
  if (cycle_completed) {
    run_time = 0;
    travelled_distance = 0;
    half_cycle_completed = 0;
    cycle_completed = 0;
  }
  digitalWrite(windlock_pin, HIGH);
  prev_second = millis();
  prev_ramp_millis = millis();
  move_fwd = fwd;
  move_bwd = bwd;
  is_running = 1;
  auto_run = run_mode;
  manual_run = !run_mode;
  send_MOTS = 0x00;
  if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (auto_run) send_CMDS = 0x00;
    else send_CMDS |= (manual_run << 7);
    Serial.print("Movement started, CMDS: 0x");
    Serial.println(send_CMDS, HEX);
    xSemaphoreGive(statusMutex);
  }
}

void move_robot(bool direction_right) {
  if (!reverse_flag) {
    digitalWrite(dir_pin, direction_right);
  }
  uint8_t control_pin;
  if (direction_right) control_pin = right_sensor_pin;
  else control_pin = left_sensor_pin;
  if (!digitalRead(control_pin)) {
    if (!vary_speed_control) {
      if (flag_brush) {
        if (ramp_brush < br_pwm) {
          if (!reverse_flag) brush_ramp_up();
        } else {
          flag_brush = 0;
          prev_ramp_millis = millis();
        }
      } else {
        if (!reverse_flag) wheel_ramp_up();
      }
    } else {
      if (!reverse_flag) wheel_ramp_up();
    }

    if (!reverse_flag) {
      ledcWrite(wheel_channel, ramp_pwm);
      ledcWrite(brush_channel, ramp_brush * !hold_brush);
    }
    prev_stop_millis = millis();
  } else {
    // uint8_t sens_delay = direction_right ? ((sens_delay_right + 1) * 15) : sens_delay_left + 1) * 15);
    uint8_t sens_delay = ((sens_delay_right > 0) ? sens_delay_right : 1) * 25;
    if (millis() - prev_stop_millis > sens_delay + 100) {
      end_flag = 1;
      if (move_fwd) {
        move_fwd = 0;
        half_cycle_completed = 1;
        write_parameters();
      } else {
        move_bwd = 0;
        // hal_cycle_completed = 0;
        cycle_completed = 1;
      }
      prev_stop_millis = millis();
    } else {
      if (move_bwd && millis() - prev_stop_millis < sens_delay) {

      } else {
        if (!vary_speed_control) {
          ledcWrite(wheel_channel, 0);
        } else {
          ledcWrite(wheel_channel, 0);
          ledcWrite(brush_channel, 0);
        }
      }
    }
  }
}

void converting_pwm() {
  if (vary_speed_control) {
    if (move_bwd) {
      wheel_pwm = default_wheel_pwm - (vary_speed_control * left_minus_speed);
      // br_pwm = default_brush_pwm - (vary_speed_control * left_minus_speed);
    } else if (move_fwd) {
      // Serial.println("aaa");
      wheel_pwm = default_wheel_pwm - (vary_speed_control * right_minus_speed);
      // br_pwm = default_brush_pwm - (vary_speed_control * right_minus_speed);
    } else {
      wheel_pwm = default_wheel_pwm - (vary_speed_control * left_minus_speed);
    }
  } else {
    wheel_pwm = default_wheel_pwm;
  }
  br_pwm = default_brush_pwm;
}

void brush_ramp_up() {
  if (millis() - prev_ramp_millis > (brush_rampup_time / br_pwm)) {
    prev_ramp_millis = millis();
    ramp_brush++;
    ramp_brush = constrain(ramp_brush, 0, br_pwm);
    ramp_pwm = 0;
  }
}

void brush_ramp_down() {
  if (millis() - prev_ramp_millis > (brush_rampdown_time / br_pwm)) {
    ramp_brush--;
    ramp_pwm = 0;
    ramp_brush = constrain(ramp_brush, 0, br_pwm);
    ledcWrite(brush_channel, ramp_brush * (!hold_brush));
    prev_ramp_millis = millis();
  }
  flag_brush = 1;
}

void wheel_ramp_up() {
  if (millis() - prev_ramp_millis > (wheel_rampup_time / wheel_pwm)) {
    uint16_t diff = 0;
    diff = (millis() - prev_ramp_millis) / (wheel_rampup_time / wheel_pwm);
    prev_ramp_millis = millis();
    ramp_pwm += diff;
    ramp_brush += diff;
    if(!roll_anti_dir_stop) ramp_brush += diff;
    ramp_pwm = constrain(ramp_pwm, 0, wheel_pwm);
    ramp_brush = constrain(ramp_brush, 0, br_pwm);
  }
}

void speed_ramp_down(uint16_t ramp_down_time) {
  // if (millis() - prev_ramp_millis > (ramp_down_time / (wheel_pwm > br_pwm ? wheel_pwm : br_pwm))) {
  if (millis() - prev_ramp_millis > (ramp_down_time / (wheel_pwm > br_pwm ? wheel_pwm : br_pwm))) {
    uint16_t diff = 0;
    diff = (millis() - prev_ramp_millis) / (ramp_down_time / (wheel_pwm > br_pwm ? wheel_pwm : br_pwm)) + 1;
    prev_ramp_millis = millis();
    ramp_pwm -= diff;
    ramp_brush -= diff;
    ramp_pwm = constrain(ramp_pwm, 0, wheel_pwm);
    ramp_brush = constrain(ramp_brush, 0, br_pwm);
  }
}

void stop_moving() {
  ledcWrite(wheel_channel, 0);
  if (end_flag) {
    if ((millis() - prev_stop_millis < stop_wait_time)) {
      if (!vary_speed_control) {
        brush_ramp_down();
      }
    } else {
      ramp_pwm = 0;
      ramp_brush = 0;
      end_flag = 0;
      prev_ramp_millis = millis();
      prev_stop_millis = millis();
    }
  }
}

void clock_read(void *parameter) {
  uint32_t lastWdtReset = 0;
  esp_task_wdt_add(xTaskGetCurrentTaskHandle());
  uint8_t local_SC_hour = 0, local_SC_minute = 0;
  while (1) {
    if ((millis() - lastWdtReset) > 1000) {
      esp_task_wdt_reset();
      lastWdtReset = millis();
      // Serial.print(abs(charger_current - ref_charger_current));
      // Serial.print("  ");
      // Serial.print(ref_charger_current);
      // Serial.print("  ");
      Serial.print(rtcClock.dt.year);
      Serial.print("  ");
      Serial.print(rtcClock.dt.month);
      Serial.print("  ");
      Serial.print(rtcClock.dt.date);
      Serial.print("  ");
      Serial.print(rtcClock.dt.hour);
      Serial.print("  ");
      Serial.print(rtcClock.dt.minute);
      Serial.print("  ");
      Serial.println(rtcClock.rtc.temp());
      // Serial.print("  ");
      // Serial.print(abs(brush_current - ref_brush_current));
      // Serial.print("  ");
      // Serial.print(ref_brush_current);
      // Serial.print("  ");
      // Serial.print(abs(wheel_current - ref_wheel_current));
      // Serial.print("  ");
      // Serial.println("new");
      // Serial.println(analogReadMilliVolts(brush_current_pin));
      // Serial.println(String(date) + "/" + String(month) + "/" + String(year) + " " + String(hour) + ":" + String(minute) + ":" + String(second));
      // RTC_error_flag = !rtc.refresh();
      rtcClock.update();
      if (is_running && (abs(wheel_current - ref_wheel_current) > 0.5 || abs(brush_current - ref_brush_current) > 0.5)) {  //change
        run_time++;
        last_RT_min = run_time / 60;
        if (abs(wheel_current - ref_wheel_current) > 0.5 && !reverse_flag) {
          wheel_run_time++;
          if (move_fwd) {
            Serial.print("move_fwd  ");
            if (wheel_run_time % 5 == 0) {
              Serial.print(travelled_distance);
              travelled_distance++;
              send_CPSL = (travelled_distance & 0x00FF);
              send_CPSH = (travelled_distance >> 8);
            }
            Serial.println();
          } else if (move_bwd) {
            Serial.print("move_bwd  ");
            if (wheel_run_time % 5 == 0) {
              Serial.print(travelled_distance);
              if (travelled_distance > 0) travelled_distance--;
              send_CPSL = (travelled_distance & 0x00FF);
              send_CPSH = (travelled_distance >> 8);
            }
            Serial.println();
          }
        }
        if (abs(brush_current - ref_brush_current) > 0.5 && !reverse_flag) brush_run_time++;
        send_LSRT = last_RT_min;
      }
    }

    send_YRMO = ((rtcClock.dt.year - 2017) << 4) | (rtcClock.dt.month & 0x0F);
    send_DATE = rtcClock.dt.date;
    send_HOUR = rtcClock.dt.hour;
    send_MIN = rtcClock.dt.minute;

    if (!is_running) {
      if ((SC_hour1 == rtcClock.dt.hour && SC_minute1 == rtcClock.dt.minute) && ((timer1 >> 13) & 0x07 != 0x00)) {
        if (++trigger_counter1 == ((timer1 >> 13) & 0x07 != 0x00)) {
          Serial.println("yes");
          if (half_cycle_completed && !cycle_completed) start_movement(0, 1, 1);
          else start_movement(1, 1, 1);
          // start_movement(1, 1, 1);
          trigger_counter1 = 0;
        }
        write_parameters();
      } else if ((SC_hour2 == rtcClock.dt.hour && SC_minute2 == rtcClock.dt.minute) && ((timer2 >> 13) & 0x07 != 0x00)) {
        if (++trigger_counter2 == ((timer2 >> 13) & 0x07 != 0x00)) {
          if (half_cycle_completed && !cycle_completed) start_movement(0, 1, 1);
          else start_movement(1, 1, 1);
          trigger_counter2 = 0;
        }
        write_parameters();
      } else if ((SC_hour3 == rtcClock.dt.hour && SC_minute3 == rtcClock.dt.minute) && ((timer3 >> 13) & 0x07 != 0x00)) {
        if (++trigger_counter3 == ((timer3 >> 13) & 0x07 != 0x00)) {
          if (half_cycle_completed && !cycle_completed) start_movement(0, 1, 1);
          else start_movement(1, 1, 1);
          trigger_counter3 = 0;
        }
        write_parameters();
      }
    } else {
      uint16_t extra;
      if ((((SC_hour * 60 + SC_minute) + (distance_limit / 6)) > 1440) && (abs(hour * 60 + minute - (SC_hour * 60 + SC_minute + (distance_limit / 6))) > ((distance_limit / 6)))) extra = 1440;
      else if ((((SC_hour * 60 + SC_minute) + (distance_limit / 12)) > 1440) && (abs(hour * 60 + minute - (SC_hour * 60 + SC_minute + (distance_limit / 12))) > ((distance_limit / 12)))) extra = 1440;
      else extra = 0;

      if (travelled_distance >= distance_limit && move_fwd) {
        // send_SENS = (1 << 7);
        move_fwd = 0;
        end_flag = 1;
      }

      if ((hour * 60 + minute + extra) >= (SC_hour * 60 + SC_minute + (distance_limit / 6) + 30)) {
        send_SENS = (1 << 7);
        move_fwd = 0;
        move_bwd = 0;
        end_flag = 1;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void LoRa_functions() {
  byte local_command = 0;
  uint8_t local_sync_YM = 0, local_sync_D = 0, local_sync_H = 0, local_sync_M = 0;
  if (LoRaSerial.available() >= 1) {  // Wait for full message
    byte buf[8];
    int a = LoRaSerial.readBytes(buf, sizeof(buf));

    if (buf[0] == NodeID) {
      if (check_CRC(buf, sizeof(buf))) {
        // Serial.println("data_received_succesfully");
        Serial.print(buf[1], HEX);
        Serial.print("  ");
        Serial.print(buf[2], HEX);
        Serial.print("  ");
        Serial.println(buf[3], HEX);

        if (buf[1] == 0x03) {
          if (buf[1] == 0x03 && buf[2] == 0x00 && buf[3] == 0x10) {
            transmit_flag = 1;
            data1[0] = NodeID;
            data1[1] = 0x03;
            data1[2] = 0x20;
            data1[3] = (system_commission >> 8);
            data1[4] = (system_commission & 0xFF);
            data1[5] = 0x00;
            data1[6] = 0x00;
            data1[7] = (battery_commission >> 8);
            data1[8] = (battery_commission & 0xFF);
            data1[9] = 0x00;
            data1[10] = 0x00;
            data1[11] = (motor_commission >> 8);
            data1[12] = (motor_commission & 0xFF);
            data1[13] = 0x00;
            data1[14] = 0x00;
            data1[15] = (brush_commission >> 8);
            data1[16] = (brush_commission & 0xFF);
            data1[17] = 0x00;
            data1[18] = 0x00;
            // Serial.println("commission");
            // memcpy(data1, &commission_data, sizeof(commission_data));
          } else if (buf[1] == 0x03 && buf[2] == 0x00 && buf[3] == 0x20) {
            transmit_flag = 1;
            data1[0] = NodeID;
            data1[1] = 0x03;
            data1[2] = 0x20;
            data1[5] = 0x18;
            data1[6] = 0x0F;
            data1[17] = 0x01;
            data1[18] = 0x6D;
            data1[19] = 0x01;
            data1[20] = 0x6D;
            data1[21] = 0x01;
            data1[22] = 0x6D;
            // Serial.println("config");
            // memcpy(data1, &config_data, sizeof(config_data));
          } else if (buf[1] == 0x03 && buf[2] == 0x00 && buf[3] == 0x40) {
            transmit_flag = 1;
            data1[0] = NodeID;
            data1[1] = 0x03;
            data1[2] = 0x20;
            data1[3] = (timer1 >> 8);
            data1[4] = (timer1 & 0xFF);
            data1[5] = (timer2 >> 8);
            data1[6] = (timer2 & 0xFF);
            data1[7] = (timer3 >> 8);
            data1[8] = (timer3 & 0xFF);
            data1[11] = 0;
            data1[12] = 0;
            // Serial.println("timer");
            // memcpy(data1, &timer_data, sizeof(timer_data));
          } else if (buf[1] == 0x03 && buf[2] == 0x00 && buf[3] == 0x50) {
            transmit_flag = 1;
            data1[0] = NodeID;
            data1[1] = 0x03;
            data1[2] = 0x20;
            data1[3] = (function >> 8);
            data1[4] = (function & 0xFF);
            data1[5] = (running_mode & 0xF0);
            data1[6] = ((running_mode & 0x0F) << 4);
            // Serial.println("live");
            // memcpy(data1, &function_data, sizeof(function_data));
          } else if (buf[1] == 0x03 && buf[2] == 0x00 && buf[3] == 0x30) {
            transmit_flag = 1;
            data1[0] = NodeID;
            data1[1] = 0x03;
            data1[2] = 0x20;
            data1[3] = brush_current_limit;
            data1[4] = wheel_current_limit;
            data1[5] = wheel_current_limit;
            data1[6] = wheel_current_limit;
            data1[7] = wheel_current_limit;
            data1[8] = wheel_current_limit;
            data1[9] = 0x00;
            data1[10] = battery_current_limit;
            data1[13] = 0x59;
            data1[14] = 0xD8;
            data1[15] = 0x75;
            data1[16] = 0x30;
            data1[17] = 0x0A;
            data1[18] = 0x8C;
            data1[19] = 0x0E;
            data1[20] = 0x10;
            data1[21] = 0x10;
            data1[22] = 0x0C;
            data1[23] = 0x5B;
            data1[24] = 0xCC;
            data1[27] = 0x0A;
            data1[28] = 0x8C;
            data1[29] = 0x0E;
            data1[30] = 0x10;
            // Serial.println("live");
            // memcpy(data1, &limits_data, sizeof(limits_data));
          } else if (buf[1] == 0x03 && buf[2] == 0x00 && buf[3] == 0x51) {
            transmit_flag = 1;
            data1[0] = NodeID;
            data1[1] = 0x03;
            data1[2] = 0x20;
            data1[3] = (running_mode & 0xF0);
            data1[4] = ((running_mode & 0x0F) << 4);
            // Serial.println("live");
            // memcpy(data1, &running_data, sizeof(running_data));
          } else if (buf[1] == 0x03 && buf[2] == 0x00 && buf[3] == 0x55) {
            transmit_flag = 1;
            data1[0] = NodeID;
            data1[1] = 0x03;
            data1[2] = 0x20;
            data1[3] = upp_motor_left_limit;
            data1[4] = upp_motor_right_limit;
            data1[5] = uint8_t(lora_delay / 100);
            data1[6] = uint8_t((lora_delay % 100) / 10);
            data1[7] = curr_sens_left;
            data1[8] = curr_sens_right;
            data1[9] = uint8_t(frequency / 1000);
            data1[10] = uint8_t((frequency % 1000) / 100);
            data1[11] = sens_delay_left;
            data1[12] = sens_delay_right;
            data1[13] = distance_limit >> 8;
            data1[14] = distance_limit & 0xFF;
            // data1[15] = 0x2C;
            // Serial.println("live");
            // memcpy(data1, &control_data, sizeof(control_data));
          } else if (buf[1] == 0x03 && buf[2] == 0x00 && buf[3] == 0x90) {
            transmit_flag = 1;
            if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
              data1[0] = NodeID;
              data1[1] = 0x03;
              data1[2] = 0x20;
              data1[3] = (current_clean_record_num >> 8) & 0xFF;
              data1[4] = current_clean_record_num & 0xFF;
              data1[5] = (current_alarm_record_num >> 8) & 0xFF;
              data1[6] = current_alarm_record_num & 0xFF;
              data1[7] = send_CMDS;
              data1[8] = send_RUNS;
              data1[9] = send_MOTS;
              data1[10] = send_BATS;
              data1[11] = send_SENS;
              data1[12] = send_SRVS;
              data1[13] = send_DRV1;
              data1[14] = send_DRV2;
              data1[15] = send_DRV3;
              data1[16] = send_DRV4;
              data1[17] = send_DRV5;
              data1[18] = send_DRV6;
              data1[19] = send_BVOL;
              data1[20] = send_BSOC;
              data1[21] = send_CURH;
              data1[22] = send_CURL;
              data1[23] = ((rtcClock.rtc.temp() / 100) + 50);
              data1[24] = 0x00;
              data1[25] = uint8_t(temperature + 50);
              data1[26] = 0x00;
              data1[27] = send_CPSH;
              data1[28] = send_CPSL;
              data1[29] = send_LSRT;
              data1[30] = send_SEST;
              data1[31] = send_YRMO;
              data1[32] = send_DATE;
              data1[33] = send_HOUR;
              data1[34] = send_MIN;
              xSemaphoreGive(statusMutex);
            }
            // Serial.print("0x");
            // Serial.println(send_CMDS, HEX);
            // Serial.println("live data");
            // memcpy(data1, &live_data, sizeof(live_data));
          } else if (buf[1] == 0x03 && buf[2] == 0x01 && buf[3] == 0x00) {
            transmit_flag = 1;
            find_clean_record(((buf[4] << 8) | buf[5]));
          } else if (buf[1] == 0x03 && buf[2] == 0x10 && buf[3] == 0x00) {
            transmit_flag = 1;
            find_alarm_record(((buf[4] << 8) | buf[5]));
          }
        } else if (buf[1] == 0x06) {
          transmit_flag = 1;
          if (buf[3] == 0x22) {
            if (!is_running) NodeID = buf[5];
          } else if (buf[3] == 0x50) {
            function = (buf[4] << 8);
            // Serial.println(function, HEX);
          } else if (buf[3] == 0x51) {
            running_mode = ((buf[4]) | (buf[5] >> 4));
          } else if (buf[3] == 0x46) {
            if (!is_running) remote_run = 1;
            // remote_cycle = 1;
          } else if (buf[3] == 0x49) {
            remote_stop = 1;
          } else if (buf[3] == 0x47) {
            if (!is_running) remote_bwd = 1;
            // docking_mode = 3;
          } else if (buf[3] == 0x48) {
            if (!is_running) remote_fwd = 1;
            // docking_mode = 4;
          } else if (buf[3] == 0x4B) {
            if (!is_running) roll_CW = 1;
          } else if (buf[3] == 0x4A) {
            if (!is_running) roll_CCW = 1;
          } else if (buf[3] == 0x40) {
            timer1 = ((buf[4] << 8) | buf[5]);
          } else if (buf[3] == 0x41) {
            timer2 = ((buf[4] << 8) | buf[5]);
          } else if (buf[3] == 0x42) {
            timer3 = ((buf[4] << 8) | buf[5]);
          } else if (buf[3] == 0x10) {
            system_commission = ((buf[4] << 8) | buf[5]);
            // Serial.println((buf[5] & 0x1F))
          } else if (buf[3] == 0x12) {
            battery_commission = ((buf[4] << 8) | buf[5]);
          } else if (buf[3] == 0x14) {
            motor_commission = ((buf[4] << 8) | buf[5]);
          } else if (buf[3] == 0x16) {
            brush_commission = ((buf[4] << 8) | buf[5]);
          } else if (buf[3] == 0x30) {
            brush_current_limit = buf[4];
            wheel_current_limit = buf[5];
          } else if (buf[3] == 0x33) {
            battery_current_limit = buf[5];
          } else if (buf[3] == 0x4F) {
            time_stamp = 0;
            set_command_status(0x00);
            send_RUNS = 0x00;
            send_MOTS = 0x00;
            send_BATS = 0x00;
            // erase_parameters();
          } else if (buf[3] == 0x55) {
            upp_motor_left_limit = buf[4];
            upp_motor_right_limit = buf[5];
          } else if (buf[3] == 0x56) {
            // lora_delay_th = buf[4];
            // lora_delay_hn = buf[5];
            lora_delay = ((buf[4] > 0 ? buf[4] : 1) * 100) + ((buf[5]) * 10);
          } else if (buf[3] == 0x57) {
            curr_sens_left = buf[4];
            curr_sens_right = buf[5];
          } else if (buf[3] == 0x58) {
            // freq_th = buf[4];
            // freq_hn = buf[5];
            // frequency = ((freq_th > 0 ? freq_th : 1) * 1000) + (((freq_hn % 10) * 100);
            frequency = ((buf[4] > 0 ? buf[4] : 1) * 1000) + ((buf[5] % 10) * 100);
            ledcSetup(0, frequency, resolution);
            ledcSetup(1, frequency, resolution);
          } else if (buf[3] == 0x59) {
            sens_delay_left = buf[4];
            sens_delay_right = buf[5];
          } else if (buf[3] == 0x5A) {
            distance_limit = ((buf[4] << 8) | buf[5]);
            Serial.println(distance_limit);
          }
          write_parameters();
          convert_data();
        } else {
          transmit_flag = 0;
        }

        if (transmit_flag) {
          if (buf[1] == 0x03) {
            if (buf[2] == 0x00 || buf[2] == 0x10 || buf[2] == 0x01) {
              // delay(600);
              set_mode(transmitter);
              vTaskDelay((200 + lora_delay) / portTICK_PERIOD_MS);  //need to comment
              Serial.println("transmitter_done");
              uint16_t crc_value;
              crc_value = generate_CRC(data1, sizeof(data1));
              data3[1] = crc_value;
              data3[0] = (crc_value >> 8);
              uint8_t send_data[sizeof(data1) + sizeof(data2) + sizeof(data3)];
              memcpy(send_data, data2, sizeof(data2));
              memcpy(send_data + sizeof(data2), data1, sizeof(data1));
              memcpy(send_data + sizeof(data1) + sizeof(data2), data3, sizeof(data3));
              LoRaSerial.write(send_data, sizeof(send_data));
              // if (!lora_error) LoRaSerial.write(data2, sizeof(data2));
              // LoRaSerial.write(data1, sizeof(data1));
              // LoRaSerial.write(data3, sizeof(data3));
              // delay(200);
              Serial.println("transmit_done");
              waitForAUX(1000);
              Serial.println("aux_done");
              set_mode(receiver);
              Serial.println("receiver_done");
            }
          } else if (buf[1] == 0x06) {
            // delay(600);
            set_mode(transmitter);
            vTaskDelay((200 + lora_delay) / portTICK_PERIOD_MS);  //need to comment
            uint8_t send_data[sizeof(data2) + sizeof(buf)];
            memcpy(send_data, data2, sizeof(data2));
            memcpy(send_data + sizeof(data2), buf, sizeof(buf));
            LoRaSerial.write(send_data, sizeof(send_data));
            // if (!lora_error) LoRaSerial.write(data2, sizeof(data2));
            // LoRaSerial.write(buf, sizeof(buf));
            waitForAUX(1000);
            set_mode(receiver);
          }
        } else {
          Serial.println("wrong_cmd");
        }
      } else {
        Serial.println("data_error");
      }
    } else if (buf[0] == 0xFE) {
      if (buf[1] == 0x22) {
        rtcClock.clock_sync_flag = true;
        time_stamp = 1;
        rtcClock.dt.year = (buf[2] >> 4) + 2017;
        rtcClock.dt.month = (buf[2] & 0x0F);
        rtcClock.dt.date = buf[3];
        rtcClock.dt.hour = buf[4];
        rtcClock.dt.minute = buf[5];
      } else if (buf[1] == 0xA5) {
        all_stop = 1;
      } else if (buf[1] == 0x5A) {
        if (!is_running) all_run = 1;
      } else if (buf[1] == 0xDD) {
        if (!is_running) all_left = 1;
      } else if (buf[1] == 0xBB) {
        if (!is_running) all_right = 1;
      }
    } else {
      Serial.println("data_for_other_node");
    }
  }
}

void lora_init() {
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX_PIN, INPUT);
  LoRaSerial.begin(9600, SERIAL_8N1);
  while (!LoRaSerial)
    ;
  waitForAUX(1000);
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(10);
  waitForAUX(200);
  uint32_t prev_lora_millis = millis();
  while (millis() - prev_lora_millis < 1000) {
    uint8_t readCommand[] = { 0xC1, 0xC1, 0xC1 };
    LoRaSerial.write(readCommand, sizeof(readCommand));
    waitForAUX(200);
    uint8_t response[6];
    int bytesRead = LoRaSerial.readBytes(response, 6);
    if (response[0] == 0xC0 || response[0] == 0xC2) {
      lora_channel = uint8_t(response[4]);
      data2[2] = lora_channel;
      if (response[3] != 0x00) SPED = uint8_t(response[3]);
      if (response[5] != 0x00) OPTION = uint8_t(response[5]);
      lora_error = 0;
      break;
    } else {
      lora_error = 1;
      continue;
    }
  }
  Serial.print("0x");
  Serial.print(lora_channel, HEX);
  Serial.print("  ");
  Serial.print("0x");
  Serial.print(SPED, HEX);
  Serial.print("  ");
  Serial.print("0x");
  Serial.println(OPTION, HEX);
  waitForAUX(200);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  waitForAUX(200);
}

void waitForAUX(uint32_t timeout_ms = 1000) {
  uint32_t start = millis();
  while (digitalRead(AUX_PIN) == LOW) {
    if (millis() - start > timeout_ms) {
      Serial.println("AUX timeout");
      break;
    }
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);  //need to comment
  // delay(10);
}

void set_mode(bool fixed_transmit_mode) {
  waitForAUX(200);
  digitalWrite(M1, HIGH);
  digitalWrite(M0, HIGH);
  // delay(20);
  vTaskDelay(20 / portTICK_PERIOD_MS);  //need to comment
  waitForAUX(200);

  uint8_t option = fixed_transmit_mode ? (OPTION | (1 << 7)) : (OPTION & ~(1 << 7));
  uint8_t command[] = { 0xC2, 0x00, 0x00, SPED, lora_channel, option };

  // Serial.println("aa");
  LoRaSerial.write(command, sizeof(command));
  // delay(20);
  vTaskDelay(20 / portTICK_PERIOD_MS);  //need to comment
  waitForAUX(200);
  // Serial.println("aaa");

  uint8_t response[6];
  int bytesRead = LoRaSerial.readBytes(response, 6);

  if (bytesRead != 6) {
    Serial.println("LoRa response error");
  }
  waitForAUX(200);
  digitalWrite(M1, LOW);
  digitalWrite(M0, LOW);
  // delay(20);
  vTaskDelay(20 / portTICK_PERIOD_MS);  //need to comment
  waitForAUX(1000);
  // Serial.println("aux com");
}

void read_sensor_values(void *parameter) {
  uint32_t lastWdtReset = 0;
  uint8_t pwm = br_pwm;
  float sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
  esp_task_wdt_add(xTaskGetCurrentTaskHandle());
  while (1) {  //need to comment
    if ((long)(millis() - lastWdtReset) > 1000) {
      esp_task_wdt_reset();
      if (is_running) {
        if (abs(brush_current - ref_brush_current) > 0.5) brush_consumed_Ah += (abs(brush_current - ref_brush_current) * (millis() - lastWdtReset) / 3600000.0f);
        if (abs(wheel_current - ref_wheel_current) > 0.5) wheel_consumed_Ah += (abs(wheel_current - ref_wheel_current) * (millis() - lastWdtReset) / 3600000.0f);
        // consumed_Ah = brush_consumed_Ah + wheel_consumed_Ah;
      }
      if (abs(charger_current - ref_charger_current) > 0.2 && charger_current > 0 && battery_voltage > 29.0f) {
        charging_Ah += (abs(charger_current - ref_charger_current) * (millis() - lastWdtReset) / 3600000.0f);
      }
      lastWdtReset = millis();
      // Serial.print(consumed_Ah);
      // Serial.print("  ");
      // Serial.print(wheel_consumed_Ah);
      // Serial.print("  ");
      // Serial.println(brush_consumed_Ah);
    }
    read_raw_values(battery_voltage_pin, charger_current_pin, brush_current_pin, wheel_current_pin, 100);
    float signal_value_1 = raw_value_1 * 11.05f;
    float signal_value_2 = ((raw_value_2 * 1.5f) - (current_ref_val + ((50 - curr_sens_right) * 0.002))) / (current_sensor_factor + ((50 - curr_sens_left) * 0.0002));
    float signal_value_3 = ((raw_value_3 * 1.5f) - (current_ref_val + ((50 - curr_sens_right) * 0.002))) / (current_sensor_factor + ((50 - curr_sens_left) * 0.0002));
    float signal_value_4 = ((raw_value_4 * 1.5f) - (current_ref_val + ((50 - curr_sens_right) * 0.002))) / (current_sensor_factor + ((50 - curr_sens_left) * 0.0002));
    int vbe = analogReadMilliVolts(select_pin);
    float tempC = 25.0 + ((float)vbe - VBE_25) / SLOPE;

    battery_voltage = alpha * signal_value_1 + (1 - alpha) * battery_voltage;
    charger_current = alpha * signal_value_2 + (1 - alpha) * charger_current;
    brush_current = alpha * signal_value_3 + (1 - alpha) * brush_current;
    wheel_current = alpha * signal_value_4 + (1 - alpha) * wheel_current;
    ref_charger_current = (ref_brush_current + ref_wheel_current) / 2.0f;
    // temperature = alpha * tempC + (1 - alpha) * temperature;
    temperature = tempC;

    if (battery_voltage < 20.0f) battery_error_flag = 1;
    else battery_error_flag = 0;

    if (!brush_error_flag) {
      if (is_running) {
        check_brush_error();
      }
    } else {
      if (millis() - prev_brush_millis > 6000) {
        direction_changed = 0;
        reverse_flag = 0;
        if (roll_anti_dir) {
          roll_anti_dir_stop = 1;
          prev_roll_anti_dir_millis = millis();
        }
        if (millis() - prev_brush_error_millis < 20000) brush_error_count++;
        prev_brush_error_millis = millis();
        brush_error_flag = 0;
        if (brush_error_count > 2) {
          if (move_fwd) {
            move_fwd = 0;
            end_flag = 1;
            brush_error_count = 0;
            wheel_error_count = 0;
          } else {
            if (vary_speed_control) {
              end_flag = 1;
              move_bwd = 0;
            }
            hold_brush = 1;
            send_MOTS |= (1 << 0);
          }
          prev_ramp_millis = millis();
        }
      } else {
        if (brush_error_count < 3 && !end_flag) {
          reverse_flag = 1;
          if (millis() - prev_brush_millis < 1000) {
            speed_ramp_down(800);
            ledcWrite(brush_channel, ramp_brush);
            ledcWrite(wheel_channel, ramp_pwm);
          } else if (millis() - prev_brush_millis < 5000) {
            if (!direction_changed) {
              digitalWrite(dir_pin, !digitalRead(dir_pin));
              direction_changed = 1;
            }
            wheel_ramp_up();
            if (!vary_speed_control) ledcWrite(brush_channel, 0);
            else ledcWrite(brush_channel, ramp_brush);
            ledcWrite(wheel_channel, ramp_pwm);
          } else {
            speed_ramp_down(800);
            if (!vary_speed_control) ledcWrite(brush_channel, 0);
            else ledcWrite(brush_channel, ramp_brush);
            ledcWrite(wheel_channel, ramp_pwm);
          }
        }
      }
    }

    if (!wheel_error_flag) {
      if (is_running) {
        check_wheel_error();
      }
    } else {
      if (millis() - prev_wheel_millis > 6000) {
        if (roll_anti_dir) {
          roll_anti_dir_stop = 1;
          prev_roll_anti_dir_millis = millis();
        }
        direction_changed = 0;
        reverse_flag = 0;
        if (millis() - prev_wheel_error_millis < 20000) wheel_error_count++;
        prev_wheel_error_millis = millis();
        wheel_error_flag = 0;
        if (wheel_error_count > 2) {
          if (move_fwd) {
            move_fwd = 0;
            end_flag = 1;
            brush_error_count = 0;
            wheel_error_count = 0;
          } else {
            move_bwd = 0;
            end_flag = 1;
            send_MOTS |= (1 << 1);
            wheel_error_flag = 1;
          }
          prev_ramp_millis = millis();
        }
      } else {
        if (wheel_error_count < 3 && !end_flag) {
          reverse_flag = 1;
          if (millis() - prev_wheel_millis < 1000) {
            speed_ramp_down(800);
            ledcWrite(brush_channel, ramp_brush);
            ledcWrite(wheel_channel, ramp_pwm);
          } else if (millis() - prev_wheel_millis < 5000) {
            if (!direction_changed) {
              digitalWrite(dir_pin, !digitalRead(dir_pin));
              direction_changed = 1;
            }
            wheel_ramp_up();
            if (!vary_speed_control) ledcWrite(brush_channel, 0);
            else ledcWrite(brush_channel, ramp_brush);
            ledcWrite(wheel_channel, ramp_pwm);
          } else {
            speed_ramp_down(800);
            if (!vary_speed_control) ledcWrite(brush_channel, 0);
            else ledcWrite(brush_channel, ramp_brush);
            ledcWrite(wheel_channel, ramp_pwm);
          }
        }
      }
    }

    if ((abs(wheel_current - ref_wheel_current) + abs(brush_current - ref_brush_current)) > battery_limit_value) battery_current_flag = 1;
    else battery_current_flag = 0;

    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      send_DRV1 = int(abs(brush_current - ref_brush_current) * 10);
      send_DRV2 = int(abs(wheel_current - ref_wheel_current) * 10);
      send_DRV3 = 0;
      send_DRV4 = 0;
      send_DRV5 = 0;
      send_DRV6 = 0;
      send_BVOL = int((battery_voltage - 10) * 10);
      send_BSOC = battery_SOC(battery_voltage);
      if (show_charigng_Ah) {
        send_CURL = (int16_t(charging_Ah * 10) & 0x00FF);
        send_CURH = (int16_t(charging_Ah * 10) >> 8);
      } else {
        if (abs(charger_current - ref_charger_current) > 0.2) {
          send_CURL = (int16_t(abs(charger_current - ref_charger_current) * 10) & 0x00FF);
          send_CURH = (int16_t(abs(charger_current - ref_charger_current) * 10) >> 8);
        } else {
          send_CURL = 0;
          send_CURH = 0;
        }
      }
      bool is_battery_soc_low = (send_BSOC < 30);
      send_BATS = ((is_battery_soc_low << 6) | (battery_current_flag << 5) | (battery_error_flag << 0));
      xSemaphoreGive(statusMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void check_brush_error() {
  if ((abs(brush_current - ref_brush_current) > critical_brush_current)) {
    if ((millis() - prev_brush_millis > critical_error_time)) {
      move_fwd = 0;
      move_bwd = 0;
      end_flag = 1;
      send_MOTS = (1 << 7);
      write_parameters();
    }
  } else if ((abs(brush_current - ref_brush_current) > brush_limit_value)) {
    if ((millis() - prev_brush_millis > (error_limit_time - (200 * upp_motor_right_limit)))) {
      brush_error_flag = 1;
      prev_brush_millis = millis();
    }
  } else prev_brush_millis = millis();
}

void check_wheel_error() {
  if ((abs(wheel_current - ref_wheel_current) > critical_wheel_current)) {
    if ((millis() - prev_wheel_millis > critical_error_time)) {
      move_fwd = 0;
      move_bwd = 0;
      end_flag = 1;
      send_MOTS = (1 << 7);
      write_parameters();
    }
  } else if ((abs(wheel_current - ref_wheel_current) > wheel_limit_value)) {
    if ((millis() - prev_wheel_millis > (error_limit_time - (200 * upp_motor_right_limit)))) {
      wheel_error_flag = 1;
      prev_wheel_millis = millis();
    }
  } else prev_wheel_millis = millis();
}

void read_raw_values(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint8_t samples) {
  long sum = 0;
  int32_t raw1 = 0, raw2 = 0, raw3 = 0, raw4 = 0;
  for (int i = 0; i < samples; i++) {
    raw1 += analogReadMilliVolts(pin1);
    raw2 += analogReadMilliVolts(pin2);
    raw3 += analogReadMilliVolts(pin3);
    raw4 += analogReadMilliVolts(pin4);
  }
  raw_value_1 = (float)raw1 / (samples * 1000);
  raw_value_2 = (float)raw2 / (samples * 1000);
  raw_value_3 = (float)raw3 / (samples * 1000);
  raw_value_4 = (float)raw4 / (samples * 1000);
  // raw_value_1 -= 0.03;
  // raw_value_2 -= 0.04;
  // raw_value_3 -= 0.04;
  // raw_value_4 -= 0.04;
}

uint16_t generate_CRC(uint8_t *data, size_t length) {
  uint16_t CRC_data = 0xFFFF;  // Initial value for CRC-16
  for (size_t i = 0; i < length; i++) {
    CRC_data ^= data[i];  // XOR with current byte
    for (int j = 0; j < 8; j++) {
      if (CRC_data & 0x0001) {                // If LSB is 1
        CRC_data = (CRC_data >> 1) ^ 0xA001;  // XOR with polynomial
      } else {
        CRC_data = (CRC_data >> 1);  // Just shift
      }
    }
  }
  return CRC_data;
}

bool check_CRC(uint8_t *data, size_t length) {
  uint16_t CRC_data = 0xFFFF;  // Initial value for CRC-16
  for (size_t i = 0; i < length - 2; i++) {
    CRC_data ^= data[i];  // XOR with current byte
    for (int j = 0; j < 8; j++) {
      if (CRC_data & 0x0001) {                // If LSB is 1
        CRC_data = (CRC_data >> 1) ^ 0xA001;  // XOR with polynomial
      } else {
        CRC_data = (CRC_data >> 1);  // Just shift
      }
    }
  }
  if (CRC_data == ((data[length - 2] << 8) | (data[length - 1]))) return 1;
  else return 0;
}

uint8_t battery_SOC(float voltage) {
  if (voltage >= 27.2) return uint8_t(100);
  else if (voltage < 27.2 && voltage >= 26.8) return uint8_t(mapFloat(voltage, 27.2, 26.8, 100, 90));
  else if (voltage < 26.8 && voltage >= 26.6) return uint8_t(mapFloat(voltage, 26.8, 26.6, 90, 80));
  else if (voltage < 26.6 && voltage >= 26.4) return uint8_t(mapFloat(voltage, 26.6, 26.4, 80, 70));
  else if (voltage < 26.4 && voltage >= 26.1) return uint8_t(mapFloat(voltage, 26.4, 26.1, 70, 50));
  else if (voltage < 26.1 && voltage >= 26.0) return uint8_t(mapFloat(voltage, 26.1, 26.0, 50, 40));
  else if (voltage < 26.0 && voltage >= 25.8) return uint8_t(mapFloat(voltage, 26.0, 25.8, 40, 30));
  else if (voltage < 25.8 && voltage >= 25.6) return uint8_t(mapFloat(voltage, 25.8, 25.6, 30, 20));
  else if (voltage < 25.6 && voltage >= 24.0) return uint8_t(mapFloat(voltage, 25.6, 24.0, 20, 10));
  else return uint8_t(10);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

void find_clean_record(uint16_t recordNum) {
  File file = LittleFS.open("/clean_records.bin", FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Record record;
  int recordSize = sizeof(Record);
  int position = (recordNum - 1) * recordSize;  // Calculate position in file

  file.seek(position);  // Jump to the correct record position
  file.read((uint8_t *)&record, recordSize);
  file.close();

  if (record.record_num == recordNum) {
    memcpy(data1, record.record_data, sizeof(record.record_data));
  } else {
    memset(data1, 0, sizeof(data1));
  }
}

void find_alarm_record(uint16_t recordNum) {
  File file = LittleFS.open("/alarm_records.bin", FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Record record;
  int recordSize = sizeof(Record);
  int position = (recordNum - 1) * recordSize;  // Calculate position in file

  file.seek(position);  // Jump to the correct record position
  file.read((uint8_t *)&record, recordSize);
  file.close();

  if (record.record_num == recordNum) {
    memcpy(data1, record.record_data, sizeof(record.record_data));
  } else {
    memset(data1, 0, sizeof(data1));
  }
}

void write_clean_record(uint16_t recordNum) {
  Record record;
  record.record_num = recordNum;
  data1[0] = NodeID;
  data1[1] = 0x03;
  data1[2] = 0x20;
  data1[3] = (recordNum >> 8) & 0xFF;
  data1[4] = (recordNum & 0xFF);
  if (auto_run) data1[5] = 0x01;
  else if (manual_run || remote_run) data1[5] = 0x00;
  data1[6] = 0x00;
  data1[7] = 0x00;
  data1[8] = 0x00;
  if (brush_run_time == 0) {
    data1[9] = 0;
  } else {
    data1[9] = uint8_t(brush_consumed_Ah * 3600.0f * 10.0f / brush_run_time);
  }
  if (wheel_run_time == 0) {
    data1[10] = 0;
  } else {
    data1[10] = uint8_t(wheel_consumed_Ah * 3600.0f * 10.0f / wheel_run_time);
  }
  data1[11] = 0x00;
  data1[12] = 0x00;
  data1[13] = 0x00;
  data1[14] = 0x00;
  data1[15] = uint8_t((battery_voltage - 10) * 10);
  data1[16] = battery_SOC(battery_voltage);
  if (run_time == 0) {
    data1[17] = 0x00;
    data1[18] = 0x00;
    data1[23] = 0x00;
    data1[24] = 0x00;
  } else {
    data1[17] = uint16_t(((brush_consumed_Ah + wheel_consumed_Ah) * 3600 / run_time) * 10.0f) >> 8;
    data1[18] = uint16_t(((brush_consumed_Ah + wheel_consumed_Ah) * 3600 / run_time) * 10.0f) & 0xFF;
    data1[23] = uint16_t(((brush_consumed_Ah + wheel_consumed_Ah)) * 10) >> 8;
    data1[24] = uint16_t(((brush_consumed_Ah + wheel_consumed_Ah)) * 10) & 0xFF;
  }
  data1[19] = ((rtcClock.rtc.temp() / 100) + 50);
  data1[20] = 0x95;
  data1[21] = uint8_t(temperature + 50);
  data1[22] = 0x64;
  data1[25] = 0x00;
  data1[26] = 0x00;
  data1[27] = run_time >> 8;
  data1[28] = run_time & 0xFF;
  data1[29] = uint8_t(rtcClock.dt.year % 2000);
  data1[30] = rtcClock.dt.month;
  data1[31] = rtcClock.dt.date;
  data1[32] = rtcClock.dt.hour;
  data1[33] = rtcClock.dt.minute;
  data1[34] = 0x00;

  memcpy(record.record_data, data1, sizeof(data1));
  File file = LittleFS.open("/clean_records.bin", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.write((uint8_t *)&record, sizeof(Record));  // Write binary data
  file.close();
}

void write_alarm_record(uint16_t recordNum) {
  Record record;
  record.record_num = recordNum;
  data1[0] = NodeID;
  data1[1] = 0x03;
  data1[2] = 0x20;
  data1[3] = (recordNum >> 8);
  data1[4] = (recordNum & 0xFF);
  data1[5] = send_MOTS;
  data1[6] = send_BATS;
  data1[7] = send_SENS;
  data1[8] = 0x00;
  if (brush_run_time == 0) {
    data1[9] = 0;
  } else {
    data1[9] = uint8_t(brush_consumed_Ah * 3600.0f * 10.0f / brush_run_time);
  }
  if (wheel_run_time == 0) {
    data1[10] = 0;
  } else {
    data1[10] = uint8_t(wheel_consumed_Ah * 3600.0f * 10.0f / wheel_run_time);
  }
  data1[11] = 0x00;
  data1[12] = 0x00;
  data1[13] = 0x00;
  data1[14] = 0x00;
  data1[15] = uint8_t((battery_voltage - 10) * 10);
  data1[16] = battery_SOC(battery_voltage);
  if (run_time == 0) {
    data1[17] = 0x00;
    data1[18] = 0x00;
    data1[23] = 0x00;
    data1[24] = 0x00;
    data1[25] = 0x00;
    data1[26] = 0x00;
  } else {
    data1[17] = uint16_t(((brush_consumed_Ah + wheel_consumed_Ah) * 3600 / run_time) * 10.0f) >> 8;
    data1[18] = uint16_t(((brush_consumed_Ah + wheel_consumed_Ah) * 3600 / run_time) * 10.0f) & 0xFF;
    data1[23] = uint16_t(((brush_consumed_Ah + wheel_consumed_Ah) * battery_voltage * 3600 / run_time) * 10) >> 8;
    data1[24] = uint16_t(((brush_consumed_Ah + wheel_consumed_Ah) * battery_voltage * 3600 / run_time) * 10) & 0xFF;
    data1[25] = uint16_t((charging_Ah * 3600 / run_time) * 10.0f) >> 8;
    data1[26] = uint16_t((charging_Ah * 3600 / run_time) * 10.0f) & 0xFF;
  }
  data1[19] = ((rtcClock.rtc.temp() / 100) + 50);
  data1[20] = 0x95;
  data1[21] = uint8_t(temperature + 50);
  data1[22] = 0x64;
  data1[27] = 0x00;
  data1[28] = 0x00;
  data1[29] = uint8_t(rtcClock.dt.year % 2000);
  data1[30] = rtcClock.dt.month;
  data1[31] = rtcClock.dt.date;
  data1[32] = rtcClock.dt.hour;
  data1[33] = rtcClock.dt.minute;
  data1[34] = 0x00;
  memcpy(record.record_data, data1, sizeof(data1));

  File file = LittleFS.open("/alarm_records.bin", "a");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.write((uint8_t *)&record, sizeof(record));  // Write binary data
  file.close();
}

int getCurrentRecordNumber(String file_path) {
  if (LittleFS.exists(file_path)) {
    File file = LittleFS.open(file_path, FILE_READ);
    if (!file) {
      Serial.println("Failed to open file");
      return -1;  // Return -1 if file doesn't exist
    }
    Serial.println("size");
    int fileSize = file.size();  // Get file size in bytes
    Serial.println(fileSize);
    int recordSize = sizeof(Record);  // Size of one record
    Serial.println(recordSize);
    file.close();
    return fileSize / recordSize;  // Number of records
  } else {
    File file = LittleFS.open(file_path, "w");
    if (!file) {
      Serial.println("Failed to open file for writing");
    }
    file.close();
    return 0;
  }
}

void read_parameters() {
  // File file = LittleFS.open("/parameters.bin", "r");
  if (LittleFS.exists("/parameters.bin")) {  // Check if file exists
    File file = LittleFS.open("/parameters.bin", "r");
    size_t fileSize = file.size();  // Get the size
    file.close();
    if (fileSize < 36) {
      LittleFS.remove("/parameters.bin");
      File file = LittleFS.open("/parameters.bin", "w");  // Append mode
      if (!file) {
        Serial.println("Failed to open file for writing");
        return;
      }
      file.write(basic_parameters, sizeof(basic_parameters));
      file.close();
    } else {
      File validFile = LittleFS.open("/parameters.bin", "r");
      validFile.readBytes((char *)basic_parameters, sizeof(basic_parameters));
      validFile.close();
    }
  } else {
    File file = LittleFS.open("/parameters.bin", "w");  // Append mode
    if (!file) {
      Serial.println("Failed to open file for writing");
      return;
    }
    file.write(basic_parameters, sizeof(basic_parameters));
    file.close();
  }
  Serial.println("Reading records...");
  NodeID = basic_parameters[address_byte];
  running_mode = basic_parameters[running_mode_byte];
  timer1 = (((basic_parameters[timer1_u_byte]) << 8) | basic_parameters[timer1_l_byte]);
  timer2 = (((basic_parameters[timer2_u_byte]) << 8) | basic_parameters[timer2_l_byte]);
  timer3 = (((basic_parameters[timer3_u_byte]) << 8) | basic_parameters[timer3_l_byte]);
  trigger_counter = basic_parameters[timer_trigger_byte];
  system_commission = (((basic_parameters[system_commission_u_byte]) << 8) | basic_parameters[system_commission_l_byte]);
  battery_commission = (((basic_parameters[battery_commission_u_byte]) << 8) | basic_parameters[battery_commission_l_byte]);
  motor_commission = (((basic_parameters[motor_commission_u_byte]) << 8) | basic_parameters[motor_commission_l_byte]);
  brush_commission = (((basic_parameters[brush_commission_u_byte]) << 8) | basic_parameters[brush_commission_l_byte]);
  function = (((basic_parameters[function_u_byte]) << 8) | basic_parameters[function_l_byte]);
  brush_current_limit = basic_parameters[brush_curr_limit_byte];
  wheel_current_limit = basic_parameters[wheel_curr_limit_byte];
  battery_current_limit = basic_parameters[battery_curr_limit_byte];
  run_status = basic_parameters[run_status_byte];
  command_status = basic_parameters[command_status_byte];
  motor_status = basic_parameters[motor_status_byte];
  bat_sens_status = basic_parameters[bat_sens_status_byte];
  upp_motor_left_limit = basic_parameters[upp_motor_left_limit_byte];
  upp_motor_right_limit = basic_parameters[upp_motor_right_limit_byte];
  curr_sens_left = basic_parameters[curr_sens_left_byte];
  curr_sens_right = basic_parameters[curr_sens_right_byte];
  sens_delay_left = basic_parameters[sens_delay_left_byte];
  sens_delay_right = basic_parameters[sens_delay_right_byte];
  distance_limit = (((basic_parameters[distance_limit_upper_byte]) << 8) | basic_parameters[distance_limit_lower_byte]);
  // frequency = (((basic_parameters[frequency_upper_byte]) << 8) | basic_parameters[frequency_lower_byte]);
  frequency = ((basic_parameters[frequency_upper_byte] * 1000) + (basic_parameters[frequency_lower_byte] * 100));
  lora_delay = ((basic_parameters[lora_delay_upper_byte] * 100) + (basic_parameters[lora_delay_lower_byte] * 10));
  run_time = (((basic_parameters[runtime_u_byte]) << 8) | basic_parameters[runtime_l_byte]);
  travelled_distance = (((basic_parameters[curr_pos_upper_byte]) << 8) | basic_parameters[curr_pos_lower_byte]);
  last_RT_min = run_time / 60;
  // run_time = last_RT_min * 60;
  // wheel_run_time = run_time;
  // brush_run_time = run_time;
  // time_stamp = bool(run_status & 0x40);
  auto_run = bool(run_status & 0x80);
  // abnormal_pos = bool(run_status & 0x10);
  // return_pos = bool(run_status & 0x08);
  // parking_pos = bool(run_status & 0x01);

  Serial.print("NodeID: ");
  Serial.println(NodeID);

  convert_data();

  send_RUNS = run_status;
  send_CMDS = command_status;
  send_MOTS = motor_status;
  send_BATS = (bat_sens_status & 0xF0);
  send_CPSL = (travelled_distance & 0x00FF);
  send_CPSH = (travelled_distance >> 8);
  send_SEST = (bat_sens_status & 0x0F);
  send_LSRT = last_RT_min;

  half_cycle_completed = bool(send_SEST & 0x02);
  cycle_completed = bool(send_SEST & 0x04);

  // if (cycle_completed) {
  //   travelled_distance = 0;
  //   send_CPSL = (travelled_distance & 0xFF);
  //   send_CPSH = (travelled_distance >> 8);
  // } else {
  //   send_LSRT = last_RT_min;
  //   travelled_distance = wheel_run_time * 12 / (60 * 2);
  //   send_CPSL = (travelled_distance & 0xFF);
  //   send_CPSH = (travelled_distance >> 8);
  // }

  Serial.println("Robot ready with NodeID:" + String(NodeID));
  Serial.println("runtime:" + String(run_time));
  Serial.println("lora_delay:" + String(lora_delay));
  Serial.println("curr_sens_left:" + String(curr_sens_left));
  Serial.println("curr_sens_right:" + String(curr_sens_right));
  Serial.println("sens_delay_left:" + String(sens_delay_left));
  Serial.println("sens_delay_right:" + String(sens_delay_right));
}

void convert_data() {
  if (brush_current_limit == 0 || wheel_current_limit == 0 || battery_current_limit == 0) {
  } else {
    brush_limit_value = brush_current_limit / 10.0f;
    wheel_limit_value = wheel_current_limit / 10.0f;
    battery_limit_value = battery_current_limit / 10.0f;
  }

  if ((running_mode & 0xF0) == 0x00) docking_mode = 1;
  else if ((running_mode & 0xF0) == 0x40) docking_mode = 0;
  else if ((running_mode & 0xF0) == 0xC0) docking_mode = 2;
  else docking_mode = 0;

  if ((running_mode & 0x0F) == 0x00) {
    fwd_pwm = slow_pwm;
    bwd_pwm = slow_pwm;
  } else if ((running_mode & 0x0F) == 0xC0) {
    fwd_pwm = fast_pwm;
    bwd_pwm = fast_pwm;
  } else if ((running_mode & 0x0F) == 0x80) {
    fwd_pwm = fast_pwm;
    bwd_pwm = slow_pwm;
  } else if ((running_mode & 0x0F) == 0x40) {
    fwd_pwm = slow_pwm;
    bwd_pwm = fast_pwm;
  }

  if ((timer1 >> 13) & 0x07 != 0x00) {
    SC_hour1 = constrain(((timer1 >> 8) & 0x1F), 0, 23);
    SC_minute1 = constrain((timer1 & 0x00FF), 0, 59);
  }

  if ((timer2 >> 13) & 0x07 != 0x00) {
    SC_hour2 = constrain(((timer2 >> 8) & 0x1F), 0, 23);
    SC_minute2 = constrain((timer2 & 0x00FF), 0, 59);
  }

  if ((timer3 >> 13) & 0x07 != 0x00) {
    SC_hour3 = constrain(((timer3 >> 8) & 0x1F), 0, 23);
    SC_minute3 = constrain((timer3 & 0x00FF), 0, 59);
  }

  roll_anti_dir = bool((function >> 8) & 0x02);
  roll_self_clean = bool((function >> 8) & 0x01);
  docking_delay = bool((function >> 8) & 0x10);
  vary_speed_control = bool((function >> 8) & 0x08);
  show_charigng_Ah = bool((function & 0xFF) & 0x04);
  if (!vary_speed_control) {
    // Serial.println("gt");
    left_minus_speed = 0;
    right_minus_speed = 0;
  } else {
    float temp = brush_limit_value;
    brush_limit_value = wheel_limit_value;
    wheel_limit_value = temp;
    left_minus_speed = upp_motor_left_limit;
    right_minus_speed = upp_motor_left_limit;
  }
}

void write_parameters() {
  basic_parameters[address_byte] = NodeID;
  basic_parameters[running_mode_byte] = running_mode;
  basic_parameters[timer1_u_byte] = (timer1 >> 8);
  basic_parameters[timer1_l_byte] = (timer1 & 0xFF);
  basic_parameters[timer2_u_byte] = (timer2 >> 8);
  basic_parameters[timer2_l_byte] = (timer2 & 0xFF);
  basic_parameters[timer3_u_byte] = (timer3 >> 8);
  basic_parameters[timer3_l_byte] = (timer3 & 0xFF);
  trigger_counter = ((trigger_counter1 & 0x07) | ((trigger_counter2 & 0x07) << 3) | ((trigger_counter3 & 0x07) << 6));
  basic_parameters[timer_trigger_byte] = trigger_counter;
  basic_parameters[system_commission_u_byte] = (system_commission >> 8);
  basic_parameters[system_commission_l_byte] = (system_commission & 0xFF);
  basic_parameters[battery_commission_u_byte] = (battery_commission >> 8);
  basic_parameters[battery_commission_l_byte] = (battery_commission & 0xFF);
  basic_parameters[motor_commission_u_byte] = (motor_commission >> 8);
  basic_parameters[motor_commission_l_byte] = (motor_commission & 0xFF);
  basic_parameters[brush_commission_u_byte] = (brush_commission >> 8);
  basic_parameters[brush_commission_l_byte] = (brush_commission & 0xFF);
  basic_parameters[function_u_byte] = (function >> 8);
  basic_parameters[function_l_byte] = (function & 0xFF);
  basic_parameters[brush_curr_limit_byte] = brush_current_limit;
  basic_parameters[wheel_curr_limit_byte] = wheel_current_limit;
  basic_parameters[battery_curr_limit_byte] = battery_current_limit;
  basic_parameters[runtime_u_byte] = run_time >> 8;
  basic_parameters[runtime_l_byte] = run_time & 0x00FF;
  basic_parameters[curr_pos_upper_byte] = travelled_distance >> 8;
  basic_parameters[curr_pos_lower_byte] = travelled_distance & 0x00FF;
  basic_parameters[run_status_byte] = send_RUNS | (half_cycle_completed << 3) | (cycle_completed << 0);
  basic_parameters[command_status_byte] = send_CMDS;
  basic_parameters[motor_status_byte] = send_MOTS;
  basic_parameters[bat_sens_status_byte] = ((send_BATS & 0xF0) | (send_SEST & 0x0F));
  basic_parameters[upp_motor_left_limit_byte] = upp_motor_left_limit;
  basic_parameters[upp_motor_right_limit_byte] = upp_motor_right_limit;
  basic_parameters[curr_sens_left_byte] = curr_sens_left;
  basic_parameters[curr_sens_right_byte] = curr_sens_right;
  basic_parameters[sens_delay_left_byte] = sens_delay_left;
  basic_parameters[sens_delay_right_byte] = sens_delay_right;
  basic_parameters[distance_limit_upper_byte] = ((distance_limit >> 8) & 0xFF);
  basic_parameters[distance_limit_lower_byte] = (distance_limit & 0xFF);
  basic_parameters[frequency_upper_byte] = uint8_t(frequency / 1000);
  basic_parameters[frequency_lower_byte] = uint8_t((frequency % 1000) / 100);
  basic_parameters[lora_delay_upper_byte] = uint8_t(lora_delay / 100);
  basic_parameters[lora_delay_lower_byte] = uint8_t((lora_delay % 100) / 10);

  File file = LittleFS.open("/parameters.bin", "w");  // Append mode
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.write(basic_parameters, sizeof(basic_parameters));
  file.close();
}

void erase_parameters() {
  // basic_parameters[address_byte] = 0;
  // basic_parameters[run_status_byte] = 0;
  // basic_parameters[command_status_byte] = 0;
  // basic_parameters[motor_status_byte] = 0;
  // basic_parameters[bat_sens_status_byte] = 0;
  // File file = LittleFS.open("/parameters.bin", "w");  // Append mode
  // if (!file) {
  //   Serial.println("Failed to open file for writing");
  //   return;
  // }
  // file.write(basic_parameters, sizeof(basic_parameters));
  // file.close();

  // if (LittleFS.exists("/parameters.bin")) {  // Check if file exists

  //   LittleFS.remove("/parameters.bin");

  //   File file = LittleFS.open("/parameters.bin", "w");  // Append mode
  //   if (!file) {
  //     Serial.println("Failed to open file for writing");
  //     return;
  //   }
  //   file.write(basic_parameters, sizeof(basic_parameters));
  //   file.close();
  // }
  if (LittleFS.exists("/parameters.bin")) {
    if (LittleFS.remove("/parameters.bin")) {
      Serial.println("parameter File deleted successfully!");
    } else {
      Serial.println("Failed to delete the parameter file!");
    }
  } else {
    Serial.println("parameter File does not exist!");
  }

  if (LittleFS.exists("/clean_records.bin")) {
    if (LittleFS.remove("/clean_records.bin")) {
      Serial.println("clean records File deleted successfully!");
    } else {
      Serial.println("Failed to delete the clean records file!");
    }
  } else {
    Serial.println("clean records File does not exist!");
  }

  if (LittleFS.exists("/alarm_records.bin")) {
    if (LittleFS.remove("/alarm_records.bin")) {
      Serial.println("alarm records File deleted successfully!");
    } else {
      Serial.println("Failed to delete the alarm records file!");
    }
  } else {
    Serial.println("alarm records File does not exist!");
  }
  ESP.restart();
}