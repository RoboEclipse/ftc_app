package org.firstinspires.ftc.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="FTC11138:TeleOp", group="Test Sensors")

public class TestTeleOp extends OpMode {

    MecanumBot myRobot = new MecanumBot();
    Gamepad g1;
    Gamepad g2;

    final double dpad_speed = 0.5;
    double armPower = 0.0;
    double theta = 0.0, v_theta = 0.0, v_rotation = 0.0;

    double servoMinPos = 0.0;
    double servoMaxPos = 0.5; //ToDo need to check with real robot on servo position
    double jewelServoPos = 0.5;
    double INCREMENT = 0.01;
    double clawServoMinPos = 0.0;
    double clawServoMaxPos = 0.5;
    double clawServoPos = 0.5;
    double currentArmPower = 0.2;
    double extenderPower ;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        g1 = gamepad1;
        g2 = gamepad2;
        myRobot.initMecanumBot(hardwareMap, telemetry);
        myRobot.disableEncoders();

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
        if (g1.dpad_up) {
            theta = 0.0;
            v_theta = dpad_speed;
        } else if (g1.dpad_down) {
            theta = Math.PI;
            v_theta = dpad_speed;
        } else if (g1.dpad_left) {
            theta = 3.0 * Math.PI / 2.0;
            v_theta = dpad_speed;
        } else if (g1.dpad_right) {
            theta = Math.PI / 2.0;
            v_theta = dpad_speed;
        } else {
            final double lx = g1.left_stick_x;
            final double ly = -g1.left_stick_y;

            theta = Math.atan2(lx, ly);
            v_theta = Math.sqrt(lx * lx + ly * ly);
            v_rotation = g1.right_stick_x;
        }

        myRobot.drive(theta, 0.8*v_theta, 0.50*v_rotation); //move robot

        armPower = -gamepad2.left_stick_y*0.1;
        extenderPower = -gamepad2.right_stick_y*0.5;
        if(gamepad1.a){
            myRobot.setJewelArm(0.67);
        }
        if(armPower>0){
            armPower=armPower*5;
        }


        if(gamepad2.dpad_up){
            if(clawServoPos>=0.0){
                clawServoPos-=0.02;
            }
        }
        if(gamepad2.dpad_down){
            if(clawServoPos<=1.0){
                clawServoPos+=0.02;
            }
        }
        if(gamepad2.left_bumper){
            if(clawServoPos>=0.02){
                clawServoPos-=0.02;
            }
        }
        if(gamepad2.right_bumper){
            if(clawServoPos<=1.0){
                clawServoPos+=0.02;
            }
        }
        myRobot.moveSideBar(clawServoPos);
        myRobot.extenderDrive(extenderPower);
        myRobot.controlArm(armPower);
        myRobot.holdArm();
        /*
        if (g2.a)
        {
            currentArmPower += RobotConfiguration.PowerIncrement;
        }
        if (g2.b)
        {
            currentArmPower -= RobotConfiguration.PowerIncrement;
        }

        if (g2.left_trigger > 0.0) {
            armPower = currentArmPower; // Maximum speed of arm motor os 0.2
        } else if (g2.right_trigger > 0.0)
            armPower = -currentArmPower / 5;
        else armPower = 0.0;
        myRobot.controlArm(armPower); //move arm up and down

       if (g1.a) {
            // jewel servo go down
            jewelServoPos -= INCREMENT;
            if (jewelServoPos <= servoMinPos) {
                jewelServoPos = servoMinPos;
            }
        }
         if (g1.b) {
             jewelServoPos += INCREMENT;
             if (jewelServoPos >= servoMaxPos) {

                 jewelServoPos = servoMaxPos;
             }
         }
        // Set the servo to the new position and pause;
       myRobot.moveJewelServo(jewelServoPos);

/*
        if (g2.right_bumper) {
            clawServoPos -= INCREMENT ;
            myRobot.moveClawServo(clawServoPos);
            if (clawServoPos <= clawServoMinPos) {
                clawServoPos = clawServoMinPos;
            }
        }
        if (g2.left_bumper) {
            clawServoPos += INCREMENT ;
            myRobot.moveClawServo(clawServoPos);
           if (clawServoPos >= clawServoMaxPos){
                clawServoPos = clawServoMinPos;
            }
        }
        */




        //telemetry.addData("jewel color", myRobot.readJewelColor());
        //telemetry.addData("bottom color", myRobot.readFloorColor());
        //telemetry.addData("heading", myRobot.getAngle());
        //telemetry.addData("Jewel_Servo_Position", "%5.2f", jewelServoPos);
        telemetry.addData("encoderPosition", myRobot.getEncoderPosition());
        telemetry.addData("currentArmPower", currentArmPower);
        telemetry.addData("Claw_Servo_Position", "%5.2f", clawServoPos);
        telemetry.update();

    }
}
