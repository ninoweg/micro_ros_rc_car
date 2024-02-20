#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int16.h>

#include <ESP32Servo.h>

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
bool micro_ros_init_successful;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

std_msgs__msg__Int16 channel1_msg;
std_msgs__msg__Int16 channel2_msg;

Servo channel1; 
Servo channel2;

bool channel1_wd;
bool channel2_wd;

struct Vehicle 
{
  float wheel_radius = 0.055;
  float wheelbase = 0.33;
  float track = 0.245;
} vehicle;

float x = 0.0;
float y = 0.0;
float yaw = 0.0;

class Wheel {
public:
  Wheel() : falling_{micros()}, 
            rising_{micros()},
            last_rising_{0}, 
            last_falling_{0}, 
            period_{1e9}
  {}

  ~Wheel() = default;

  void setup(uint8_t pin, void (*ISR_callback)(void), int value)
  {
    pin_ = pin;
    attachInterrupt(pin, ISR_callback, value);
  }

  inline void handleInterrupt(void)
  {
    if(digitalRead(pin_))
    {
      last_rising_ = rising_;
      rising_ = micros();
    }
    else
    {
      last_falling_ = falling_;
      falling_ = micros();
    }
  }

  float get_omega() { 
    auto last_update = micros() - rising_;
    if (last_update / 1000000.0 > 0.5)
      return 0.0;
    period_ = (rising_ - last_rising_) / 2.0; 
    if (falling_ > rising_)
      duty_cycle_ = falling_ - rising_; 
    else
      duty_cycle_ = falling_ - last_rising_; 

    auto omega = 2 * M_PI * 1000000.0 / period_; // in rad/s
    return duty_cycle_ < period_ / 2.0 ? omega : -omega; 
  }

private:
  unsigned long falling_, last_falling_;
  unsigned long rising_, last_rising_;
  unsigned long period_, duty_cycle_;
  uint8_t pin_;
};

auto wheel_rl = new Wheel();
auto wheel_rr = new Wheel();
auto wheel_fl = new Wheel();
auto wheel_fr = new Wheel();

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{ 
  static int counter = 0;
  static int64_t last_time = micros();
  
  if (timer != NULL) {
    auto dt = (micros() - last_time) / 1000000.0;
    last_time = micros();

    auto linear_velocity = vehicle.wheel_radius * (wheel_rl->get_omega() + wheel_rr->get_omega()) / 2.0;
    auto angular_velocity = vehicle.wheel_radius * (wheel_rr->get_omega() - wheel_rl->get_omega()) / vehicle.track;
    
    yaw += angular_velocity * dt;
    x += linear_velocity * dt * cos(yaw);
    y += linear_velocity * dt * sin(yaw);

    odom_msg.header.stamp.sec = (uint16_t)(rmw_uros_epoch_millis()/1000);
    odom_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(yaw / 2.0);
    odom_msg.pose.pose.orientation.w = cos(yaw / 2.0);

    odom_msg.pose.covariance[0]  = 0.2;
    odom_msg.pose.covariance[7]  = 0.2;
    odom_msg.pose.covariance[35] = 0.4;

    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.x = 0.0;

    odom_msg.twist.twist.angular.z = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;
        
    rcl_publish(&odom_publisher, &odom_msg, NULL);
  }
  if (counter >= 5)
  {
    if(!channel1_wd) {
      channel1.write(1500);
    }
    if(!channel2_wd) {
      channel2.write(1500);
    }
    channel1_wd = false;
    channel2_wd = false;
    counter = 5;
  }
  counter++;
}

//channel1 message cb
void channel1_callback(const void *msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  channel1.write(msg->data);
  channel1_wd = true;
}

//channel2 message cb
void channel2_callback(const void *msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  channel2.write(msg->data);
  channel2_wd = true;
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_rc_receiver", "", &support));

  // create subscriber for channel1
  RCCHECK(rclc_subscription_init_default(
    &subscriber1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "channel1"));

  // create subscriber for channel2
  RCCHECK(rclc_subscription_init_default(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "channel2"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &channel1_msg, &channel1_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &channel2_msg, &channel2_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&odom_publisher, &node);
  rcl_timer_fini(&timer);
  rcl_subscription_fini(&subscriber1, &node);
  rcl_subscription_fini(&subscriber2, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(1000);

  channel1.attach(A7);
  channel2.attach(A8);

  state = WAITING_AGENT;

  channel1.write(1500);
  channel2.write(1500);

  channel1_wd = false;
  channel2_wd = false;

  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  wheel_rl->setup(A2, []{wheel_rl->handleInterrupt();}, CHANGE);
  wheel_rr->setup(A3, []{wheel_rr->handleInterrupt();}, CHANGE);
  wheel_fl->setup(A4, []{wheel_fl->handleInterrupt();}, CHANGE);
  wheel_fr->setup(A5, []{wheel_fr->handleInterrupt();}, CHANGE);

  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.child_frame_id.data = (char *)"base_link";
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}
