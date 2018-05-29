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

    double clawServoPos = 0.1;
    double speedMultiplier = 1.2;
    double rotationMultiplier = 0.6;
    double relicArmServoPos=0.0;
    double relicHandServoPos=0.5;
    double topSidebarPosition= .6;
    double bottomSidebarPosition = .25;
    double topRotatingPosition = 0.5;
    double relicArmSlideSpeed = 0.9;

    boolean reset = true; //Boolean determining if the precision sidebar movement has been done yet
    boolean started = false;
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
        if (gamepad1.b) {
            speedMultiplier = 0.25;
            rotationMultiplier = 0.25;
        } else {
            speedMultiplier = 1.35;
            rotationMultiplier = 0.75;
        }

        myRobot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation); //move robot

        //Determine arm power
        armPower = -gamepad2.left_stick_y * 0.25;

        //Allow controller 1 to control arm if controller 2 is not
        if (Math.abs(armPower) <= 0.01) {
            if (gamepad1.left_trigger>=0.8) {
                armPower = 0.25;
            } else if (gamepad1.right_trigger>=0.8) {
                armPower = -0.25;
            }
        }
        //If arm is going up, multiply power
        if (armPower > 0) {
            armPower = armPower * 3;
        }

        if (myRobot.CheckTouchSensor()) {
            myRobot.resetArmEncoder();
        }

 /*           if (myRobot.GetArmEncoder() < firstLevel - 50) {
                myRobot.EncoderArm(firstLevel, 0.6);
                myRobot.disableArmEncoders();

            }
            else if (myRobot.GetArmEncoder() < secondLevel - 50) {
                myRobot.EncoderArm(secondLevel, 0.6);
                myRobot.disableArmEncoders();
            }
            myRobot.moveTopServo(0.5);
            Sleep(100);
            myRobot.disableArmEncoders();
            myRobot.controlArm(-1);
            Sleep(200);
            myRobot.moveSideBar(0.28);
            myRobot.controlArm(1);
            Sleep(100);
            myRobot.moveSideBar(0.1);
            myRobot.moveTopServo(0.55);
            myRobot.controlArm(0);
            */

        if (Math.abs(armPower) <= 0.01) {
            myRobot.EncoderHoldArm();
        }
        else {
            myRobot.disableArmEncoders();

            myRobot.controlArm(-armPower);
        }
        //Control for linear slide
        myRobot.moveLinearSlide(relicArmSlideSpeed*gamepad2.right_stick_y);
        if (gamepad1.x) {
            myRobot.moveLinearSlide(relicArmSlideSpeed);
        }
        if(gamepad1.y){
            myRobot.moveLinearSlide(-relicArmSlideSpeed);
        }
        //Control for glyph arm servos
        if (gamepad2.dpad_down){
            relicArmServoPos-=0.015;
            if(relicArmServoPos<0){
                relicArmServoPos=0;
            }


        }
        else if (gamepad2.dpad_up){
            relicArmServoPos+=0.015;

            if(relicArmServoPos>1.0){
                relicArmServoPos=1.0;
            }

        }
        if(gamepad2.dpad_right){
            relicHandServoPos+=0.06;
            if(relicHandServoPos>1){
                relicHandServoPos=1;
            }
        }
        if(gamepad2.dpad_left){
            relicHandServoPos-=0.06;
            if(relicHandServoPos<0){
                relicHandServoPos=0;
            }
        }
        //0.34: Vertical point
        if(gamepad2.x){
            //relicArmServoPos = 0.34;
            myRobot.xButton();
            relicHandServoPos = 1.0;
            topSidebarPosition = 0.36;
            bottomSidebarPosition = 0.5;
        }
        if(gamepad2.y){
            relicArmServoPos = 0.46;
            topSidebarPosition = 0.36;
            bottomSidebarPosition = 0.5;
        }
        //Button to set up jewel arm, flicker, top servo and claw
        if (gamepad1.a || gamepad2.a) {
            aButton();
        }
        if (gamepad2.b) {

            bButton();
        }
        //Give bumpers control of claw

        if (gamepad2.right_bumper) {
                topSidebarPosition += 0.05;
                topRotatingPosition += 0.08;
        }
        if (gamepad2.left_bumper) {
            if (clawServoPos <= 0.55) {
                topSidebarPosition -= 0.05;
                topRotatingPosition -= 0.08;
            }
        }
        if(gamepad2.left_trigger>0.01){
            bottomSidebarPosition += 0.05*gamepad2.left_trigger;
        }
        if(gamepad2.right_trigger>0.01){
            bottomSidebarPosition -= 0.05*gamepad2.right_trigger;
        }
        if (gamepad1.right_bumper) {
            bottomSidebarPosition -= 0.05;
            topSidebarPosition += 0.05;
        }
        if (gamepad1.left_bumper) {
            if (clawServoPos <= 0.55) {
                bottomSidebarPosition += 0.05;
                topSidebarPosition -= 0.05;
            }
        }
        if(topSidebarPosition> 1){
            topSidebarPosition=1;
        }
        if(topSidebarPosition< 0){
            topSidebarPosition = 0;
        }
        if(bottomSidebarPosition > 1){
            bottomSidebarPosition = 1;
        }
        if(bottomSidebarPosition < 0){
            bottomSidebarPosition = 0;
        }

        //if(!gamepad1.atRest() || !gamepad2.atRest()){
        myRobot.controlBottonClaws(bottomSidebarPosition);
        myRobot.controlTopClaws(topSidebarPosition);
        myRobot.moveRelicArmServo(relicArmServoPos);
        myRobot.moveRelicHandServo(relicHandServoPos);
        myRobot.controlRotatingClaws(topRotatingPosition);
        //}

        telemetry.addData("encoderPosition", myRobot.getEncoderPosition());
        telemetry.addData("currentArmPower", armPower);
        telemetry.addData("Relic_Claw_Servo_Position", "%5.2f", relicHandServoPos);
        telemetry.addData("Relic Arm Servo Position", relicArmServoPos);
        telemetry.addData("TopClawPosition", topSidebarPosition);
        telemetry.addData("BottomClawPosition", bottomSidebarPosition);
        telemetry.addData("Touched", myRobot.CheckTouchSensor());
        telemetry.update();

    }

    private void Sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void aButton(){
        myRobot.setJewelArm(1.0);
        myRobot.flick(1.0);
        topRotatingPosition = 0.5;
        topSidebarPosition= .65;
        bottomSidebarPosition = .2;
    }
    public void bButton(){
        topSidebarPosition = 0.55;
        bottomSidebarPosition = .27;
        topRotatingPosition = 0.5;
    }

}