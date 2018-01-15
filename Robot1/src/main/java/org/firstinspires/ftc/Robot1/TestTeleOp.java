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

    double clawServoPos = 0.5;
    double topServoPos = 0.5;
    double speedMultiplier = 1.0;
    double rotationMultiplier = 0.5;
    double relicArmServoPos=0.5;
    double relicHandServoPos=0.5;
    int firstLevel = 450;
    int secondLevel = 750;
    int thirdLevel = 950;

    boolean reset = true; //Boolean determining if the precision sidebar movement has been done yet
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        g1 = gamepad1;
        g2 = gamepad2;
        myRobot.initMecanumBot(hardwareMap, telemetry);
        myRobot.disableDriveEncoders();
        myRobot.disableArmEncoders();

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

        //Drive motor controls
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
        if (gamepad1.x || gamepad1.x) {
            speedMultiplier = 0.5;
            rotationMultiplier = 0.25;
        } else {
            speedMultiplier = 1;
            rotationMultiplier = 0.5;
        }

        myRobot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation); //move robot

        //Determine arm power
        armPower = -gamepad2.left_stick_y * 0.1;

        //Allow controller 1 to control arm if controller 2 is not
        if (Math.abs(armPower) <= 0.01) {
            if (gamepad1.left_trigger>=0.8) {
                armPower = 0.1;
            } else if (gamepad1.right_trigger>=0.8) {
                armPower = -0.1;
            }
        }
        //If arm is going up, multiply power
        if (armPower > 0) {
            armPower = armPower * 6;
        }

        if (myRobot.CheckTouchSensor()) {
            myRobot.resetArmEncoder();
        }
        if (gamepad2.y) {
            if (myRobot.GetArmEncoder() < firstLevel - 50) {
                myRobot.EncoderArm(firstLevel, 0.6);
                myRobot.disableArmEncoders();
            }
            else if (myRobot.GetArmEncoder() < secondLevel - 50) {
                myRobot.EncoderArm(secondLevel, 0.6);
                myRobot.disableArmEncoders();
            }
        }

        if (Math.abs(armPower) <= 0.01
                && myRobot.GetArmEncoder() > 30
                && myRobot.GetArmEncoder() < 1000) {
            myRobot.EncoderHoldArm();
        }
        else {
            myRobot.disableArmEncoders();
            myRobot.controlArm(armPower);
        }
        //Control for linear slide
        myRobot.moveLinearSlide(0.5*gamepad2.right_stick_y);
        //Control for glyph arm servos
        if (gamepad2.dpad_up && relicArmServoPos < 1){
            relicArmServoPos+=0.03;
        }
        else if (gamepad2.dpad_down && relicArmServoPos > 0){
            relicArmServoPos-=0.03;
        }
        if (relicHandServoPos+gamepad2.right_trigger*0.03 > 0 && relicHandServoPos+gamepad2.right_trigger*0.03 < 0.7) {
            relicHandServoPos += gamepad2.right_trigger * 0.03;
        }

        //Button to set up jewel arm, flicker, top servo and claw
        if (gamepad1.a || gamepad2.a) {
            aButton();
        }
        if (gamepad1.b || gamepad2.b) {
            bButton();
        }

        //Allow for dpad to control claw
        if (gamepad2.dpad_up) {
            if (clawServoPos >= 0.0) {
                clawServoPos -= 0.02;
            }
        }
        if (gamepad2.dpad_down) {
            if (clawServoPos <= 0.6) {
                clawServoPos += 0.02;
            }
        }
        //Give bumpers control of claw as well
        if (gamepad2.left_bumper) {
            if (clawServoPos >= 0.02) {
                clawServoPos -= 0.03;
                topServoPos = 1.0;
            }

        }
        if (gamepad2.right_bumper) {
            if (clawServoPos <= 0.55) {
                clawServoPos += 0.03;
                topServoPos = 0.5;
            }
        }
        if (gamepad1.right_bumper) {
            if (clawServoPos <= 0.55) {
                clawServoPos += 0.03;
                topServoPos = 0.5;
            }

        }
        if (gamepad1.left_bumper) {
            if (clawServoPos >= 0.02) {
                clawServoPos -= 0.03;
                topServoPos = 1.0;
            }
        }


        myRobot.moveSideBar(clawServoPos);
        myRobot.moveTopServo(topServoPos);
        myRobot.moveRelicArmServo(relicArmServoPos);
        myRobot.moveRelicHandServo(relicHandServoPos);


        telemetry.addData("encoderPosition", myRobot.getEncoderPosition());
        telemetry.addData("currentArmPower", armPower);
        telemetry.addData("Claw_Servo_Position", "%5.2f", clawServoPos);
        telemetry.addData("Top_Servo_Position", topServoPos);
        telemetry.addData("Touched", myRobot.CheckTouchSensor());
        telemetry.update();

    }
    public void aButton(){
        myRobot.setJewelArm(1.0);
        myRobot.flick(1.0);
        topServoPos = 0.75;
        clawServoPos = .4;
    }
    public void bButton(){
        clawServoPos = 0.28;
        topServoPos = 0.75;
    }

}

