package org.firstinspires.ftc.Robot3;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Arcade Teleop", group = "Test")
public class ArcadeTeleop extends OpMode {
    private MecanumRobot robot;
    private Controller g1;

    @Override
    public void init() {
        robot = new MecanumRobot(hardwareMap, telemetry);
        g1 = new Controller(gamepad1);
    }

    @Override
    public void init_loop() {
        telemetry.update();
    }

    @Override
    public void start() {
        robot.onStart();
        robot.resetGyro();
    }

    @Override
    public void loop() {
        g1.update();

        if (g1.A() && g1.B()) {
            robot.resetGyro();
        }

        double lx = g1.left_stick_x, ly = - g1.left_stick_y;
        double v = Math.sqrt(lx * lx + ly * ly);
        double theta = Math.atan2(lx, ly);
        double current = Math.toRadians(robot.getHeading());
        robot.drive(theta + current, v, g1.right_stick_x);
    }

    public static class Controller {
        private com.qualcomm.robotcore.hardware.Gamepad gamepad;

        private int dpad_up, dpad_down, dpad_left, dpad_right;
        private int x, y, a, b;
        private int left_bumper, right_bumper;

        public double left_stick_x, right_stick_x, left_stick_y, right_stick_y;
        public double left_trigger, right_trigger;

        public Controller(com.qualcomm.robotcore.hardware.Gamepad g) {
            gamepad = g;
        }

        public void update() {
            if (gamepad.x) { ++x; } else { x = 0; }
            if (gamepad.y) { ++y; } else { y = 0; }
            if (gamepad.a) { ++a; } else { a = 0; }
            if (gamepad.b) { ++b; } else { b = 0; }
            if (gamepad.dpad_up) { ++dpad_up; } else { dpad_up = 0; }
            if (gamepad.dpad_down) { ++dpad_down; } else { dpad_down = 0; }
            if (gamepad.dpad_left) { ++dpad_left; } else { dpad_left = 0; }
            if (gamepad.dpad_right) { ++dpad_right; } else { dpad_right = 0; }
            if (gamepad.left_bumper) { ++left_bumper; } else { left_bumper = 0; }
            if (gamepad.right_bumper) { ++right_bumper; } else { right_bumper = 0; }

            left_stick_x = gamepad.left_stick_x;
            left_stick_y = gamepad.left_stick_y;
            right_stick_x = gamepad.right_stick_x;
            right_stick_y = gamepad.right_stick_y;
            left_trigger = gamepad.left_trigger;
            right_trigger = gamepad.right_trigger;
        }

        public boolean dpadUp() { return 0 < dpad_up; }
        public boolean dpadDown() { return 0 < dpad_down; }
        public boolean dpadLeft() { return 0 < dpad_left; }
        public boolean dpadRight() { return 0 < dpad_right; }
        public boolean X() { return 0 < x; }
        public boolean Y() { return 0 < y; }
        public boolean A() { return 0 < a; }
        public boolean B() { return 0 < b; }
        public boolean leftBumper() { return 0 < left_bumper; }
        public boolean rightBumper() { return 0 < right_bumper; }

        public boolean dpadUpOnce() { return 1 == dpad_up; }
        public boolean dpadDownOnce() { return 1 == dpad_down; }
        public boolean dpadLeftOnce() { return 1 == dpad_left; }
        public boolean dpadRightOnce() { return 1 == dpad_right; }
        public boolean XOnce() { return 1 == x; }
        public boolean YOnce() { return 1 == y; }
        public boolean AOnce() { return 1 == a; }
        public boolean BOnce() { return 1 == b; }
        public boolean leftBumperOnce() { return 1 == left_bumper; }
        public boolean rightBumperOnce() { return 1 == right_bumper; }

    }

    @TeleOp(name="FTC11138:EncoderTeleOp", group="Test Sensors")
    @Disabled
    public static class DriveByEncoderTeleOp extends OpMode {

        MecanumBot myRobot = new MecanumBot();
        com.qualcomm.robotcore.hardware.Gamepad g1;
        com.qualcomm.robotcore.hardware.Gamepad g2;

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

            myRobot.loop();

            //myRobot.drive(theta, 0.65*v_theta, 0.60*v_rotation); //move robot

            armPower = -gamepad2.left_stick_y*0.2;
            extenderPower = -gamepad2.right_stick_y*0.5;
            if(gamepad1.a){
                myRobot.setJewelArm(0.4);
            }
            if(armPower>0){
                armPower=armPower*2.5;
            }


            if(gamepad2.dpad_up){
                if(clawServoPos>=0.3){
                    clawServoPos-=0.01;
                }
            }
            if(gamepad2.dpad_down){
                if(clawServoPos<=0.7){
                    clawServoPos+=0.01;
                }
            }
            if(gamepad2.left_bumper){
                if(clawServoPos>=0.3){
                    clawServoPos-=0.01;
                }
            }
            if(gamepad2.right_bumper){
                if(clawServoPos<=0.7){
                    clawServoPos+=0.01;
                }
            }
            myRobot.controlBottonClaws(clawServoPos);
            myRobot.extenderDrive(extenderPower);
            myRobot.controlArm(armPower);
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
                armPositionGoal = currentArmPower; // Maximum speed of arm motor os 0.2
            } else if (g2.right_trigger > 0.0)
                armPositionGoal = -currentArmPower / 5;
            else armPositionGoal = 0.0;
            myRobot.controlArm(armPositionGoal); //move arm up and down

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
}
