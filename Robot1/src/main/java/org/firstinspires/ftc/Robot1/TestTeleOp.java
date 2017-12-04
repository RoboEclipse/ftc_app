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
    double topservoPos = 0.5;
    double currentArmPower = 0.2;
    double extenderPower ;
    double speedMultiplier = 1.0;
    double rotationMultiplier=0.5;

    boolean reset = true; //Boolean determining if the precision sidebar movement has been done yet
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
        if(gamepad1.x){
            speedMultiplier=0.5;
            rotationMultiplier=0.25;
        }
        if(gamepad1.right_trigger == 1){
            if(clawServoPos<=0.6){
                clawServoPos+=0.03;
            if(topservoPos<=0.06){
                topservoPos+=0.03;
            }
            }
        }
        if(gamepad2.left_trigger == 1){
            if(clawServoPos>=0.02){
                clawServoPos-=0.03;
            if(topservoPos>=0.02){
                topservoPos-=0.03;
            }
            }
        }


        myRobot.drive(theta, speedMultiplier*v_theta, rotationMultiplier*v_rotation); //move robot

        armPower = -gamepad2.left_stick_y*0.08;
        if(gamepad1.a){
            myRobot.setJewelArm(1.0);
            myRobot.flick(1.0);
        }
        if(armPower>0){
            armPower=armPower*6;
        }
        if(gamepad1.right_bumper){
            armPower=-0.48;
        }
        if(gamepad1.left_bumper){
            armPower=0.08;
        }


        if(gamepad2.dpad_up){
            if(clawServoPos>=0.0){
                clawServoPos-=0.02;
            if(topservoPos>=0.0){
                topservoPos-=0.02;
            }
            }
        }
        if(gamepad2.dpad_down){
            if(clawServoPos<=0.6){
                clawServoPos+=0.02;
            if(topservoPos<= 0.6){
                topservoPos+=0.02;
            }
            }
        }
        if(gamepad2.left_bumper){
            if(clawServoPos>=0.02){
                clawServoPos-=0.03;
            if(topservoPos>=0.02){
                topservoPos-=0.03;
            }
            }
        }
        if(gamepad2.right_bumper){
            if(clawServoPos<=0.6){
                clawServoPos+=0.03;
            if(topservoPos<=0.6){
                topservoPos+=0.03;
            }
            }
        }
        //ToDo: Test the top servo positions for a and b
        if(gamepad2.a){
            clawServoPos = 0.1;
            topservoPos = 0.5;
        }
        if(gamepad2.b){
            clawServoPos = 0.29;
            topservoPos =0.5;
        }
        myRobot.moveSideBar(clawServoPos);
        myRobot.controlArm(armPower);
        myRobot.holdArm();

        telemetry.addData("encoderPosition", myRobot.getEncoderPosition());
        telemetry.addData("currentArmPower", currentArmPower);
        telemetry.addData("Claw_Servo_Position", "%5.2f", clawServoPos);
        telemetry.addData("Top_Servo_Position", topservoPos);
        telemetry.update();

    }
}
