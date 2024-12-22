#include <ros.h>
#include <std_msgs/String.h>
#include <Stepper.h>

#define STEPS_PER_REVOLUTION 200

#define DIR_PIN 2
#define STEP_PIN 3

Stepper stepper(STEPS_PER_REVOLUTION, DIR_PIN, STEP_PIN);

ros::NodeHandle nh;

String motor_command = "S";

void callback(const std_msgs::String& cmd) {
    motor_command = cmd.data;
}

ros::Subscriber<std_msgs::String> sub("motor_command", &callback);

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    stepper.setSpeed(100);
}

void loop() {
    nh.spinOnce();

    if (motor_command == "L") {
        stepper.step(-STEPS_PER_REVOLUTION / 100);
    } else if (motor_command == "R") {
        stepper.step(STEPS_PER_REVOLUTION / 100);
    }

    delay(1000);
}
