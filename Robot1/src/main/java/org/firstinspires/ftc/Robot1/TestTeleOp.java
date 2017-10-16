package org.firstinspires.ftc.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="FTC11138: Test TeleOp Mode", group="Test Sensors")

public class TestTeleOp extends OpMode {

    MecanumBot myRobot = new MecanumBot();
    Gamepad g;

    final double dpad_speed = 0.3;
    double armPower = 0.0;
    double theta = 0.0, v_theta = 0.0, v_rotation = 0.0;
    double servoMinPos = 0.0;
    double servoMaxPos = 0.5; //ToDo need to check with real robot on servo position
    double jewelServoPos = 0.5;
    double INCREMENT = 0.01;
    double clawServoMinPos = 0.0;
    double clawServoMaxPos = 0.5;
    double clawServoPos = 0.5;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        g = gamepad1;
        myRobot.initMecanumBot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }
    @Override
   public void loop() {
        if (g.dpad_up) {
            theta = 0.0;
            v_theta = dpad_speed;
        } else if (g.dpad_down) {
            theta = Math.PI;
            v_theta = dpad_speed;
        } else if (g.dpad_left) {
            theta = 3.0 * Math.PI / 2.0;
            v_theta = dpad_speed;
        } else if (g.dpad_right) {
            theta = Math.PI / 2.0;
            v_theta = dpad_speed;
        } else {
            final double lx = g.left_stick_x;
            final double ly = -g.left_stick_y;

            theta = Math.atan2(lx, ly);
            v_theta = Math.sqrt(lx * lx + ly * ly);
            v_rotation = g.right_stick_x;
        }

        myRobot.drive(theta, v_theta, v_rotation); //move robot

        if (g.left_trigger > 0.0) {
            armPower = g.left_trigger * 0.2; // Maximum speed of arm motor os 0.2
        } else if (g.right_trigger > 0.0)
            armPower = g.right_trigger * -0.2;
        else armPower = 0.0;
        myRobot.controlArm(armPower); //move arm up and down
        
        /*if (g.a) {
            // jewel servo go down
            jewelServoPos -= INCREMENT;
            if (jewelServoPos <= servoMinPos) {
                jewelServoPos = servoMinPos;
            }
        }
         if (g.b) {
             jewelServoPos += INCREMENT;
             if (jewelServoPos >= servoMaxPos) {
                 jewelServoPos = servoMaxPos;
             }
         }
        // Set the servo to the new position and pause;
       myRobot.moveJewelServo(jewelServoPos);


        if (g.right_bumper) {
            clawServoPos -= INCREMENT ;
            if (clawServoPos <= clawServoMinPos) {
                clawServoPos = clawServoMinPos;
            }
        }
        if (g.left_bumper) {
            clawServoPos += INCREMENT ;
            if (clawServoPos >= clawServoMaxPos){
                clawServoPos = clawServoMinPos;
            }
        }

        myRobot.moveClawServo(clawServoPos);


        telemetry.addData("jewel color", myRobot.readJewelColor());
        telemetry.addData("bottom color", myRobot.readFloorColor());
        telemetry.addData("heading", myRobot.getAngle());
        telemetry.addData("Jewel_Servo_Position", "%5.2f", jewelServoPos);
        telemetry.addData("Claw_Servo_Position", "%5.2f", clawServoPos);*/
        telemetry.update();

    }
}
