/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.Robot1;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Iterative Opmode")
//@Disabled
public class RoverRuckusTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RoverRuckusClass myRobot = new RoverRuckusClass();
    private double elevatorServoPosition = 1;
    private double tokenServoPosition = 0;
    private int stage = 0;
    private boolean fast = false;
    //private double theta = 0.0, v_theta = 0.0, v_rotation = 0.0;
    //private double speedMultiplier = 1;
    //double leadScrewPower = 0;
    //private double exMotorPower = 0;
    //private double cServoPower = 0;
    //private boolean cFlipCheck = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){

        //Drive motor controls
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier = 1;
        if(gamepad1.dpad_up){
            ly=1;
            speedMultiplier = 0.5;
        }
        else if(gamepad1.dpad_down){
            ly=-1;
            speedMultiplier = 0.5;
        }
        else if(gamepad1.dpad_left){
            lx=-1;
            speedMultiplier = 0.5;
        }
        else if(gamepad1.dpad_right){
            lx=1;
            speedMultiplier = 0.5;
        }
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        myRobot.drive(theta,  speedMultiplier*v_theta, v_rotation*.8); //move robot

        if(stage == 0) {
            driver2Manual();
            fast = false;
        }
        else {

            if (gamepad2.dpad_right || gamepad2.right_trigger>0.99) {
                stage = 0;
                elevatorServoPosition=1;
            }
            if(gamepad2.y){
                fast = true;
                elevatorServoPosition=0.45;
            }
            else if(!fast){
                elevatorServoPosition=0.7;
            }
        }
        if (gamepad2.left_trigger>0.99 || stage != 0) {
            stage = myRobot.autoDump(stage, fast);
        }

        telemetry.update();
        // Show the elapsed game time and wheel power.


    }

    private void driver2Manual() {
        double leadScrewPower = -gamepad2.right_stick_y;
        //Lead Screw Controls
        if (gamepad1.left_bumper) {
            leadScrewPower = 1;
        } else if (gamepad1.right_bumper && myRobot.isElevatorLimitSwitchNOTPressed()) {
            leadScrewPower = -1;
        }
        else {
            if (leadScrewPower > 0) {
                if (!myRobot.isElevatorLimitSwitchNOTPressed()) {
                    leadScrewPower = 0;
                    telemetry.addData("LeadScrewAutoStopped", "True");
                }
            }
        }

        myRobot.leadScrewDrive(leadScrewPower);

        //Elevator Motor Controls
        double elevatorDistance = myRobot.getElevatorDistanceSensor();
        double elevatorPower = gamepad2.left_stick_y;
        elevatorPower = elevatorTeleOp(elevatorDistance, elevatorPower);

        //Collector Servo Control
        double cServoPower = 0;
        if (gamepad2.left_bumper) {
            cServoPower = -0.79;
        } else if (gamepad2.right_bumper) {
            cServoPower = 0.79;
            myRobot.resetCFlipEncoder();
        }
        myRobot.cServoDrive(cServoPower);

        //Collector Extender Controls
        double extenderDistance = myRobot.getExtenderDistanceSensor();
        double exMotorPower = 0;
        int extenderEncoder = myRobot.getExtenderEncoder();
        if (gamepad2.dpad_up) {
            exMotorPower = 0.6;
            if(extenderDistance<=2){
                exMotorPower = 0.3;
                telemetry.addData("Slow", "true");
                myRobot.resetExtenderEncoder();
            }
        } else if (gamepad2.dpad_down) {
            exMotorPower = -0.6;
            if(extenderDistance>23){
                exMotorPower = -0.3;
                telemetry.addData("Slow", "true");
            }
        }
        myRobot.newExMotor(exMotorPower);

        if (gamepad1.x && tokenServoPosition <= 1) {
            tokenServoPosition += 0.03;
        } else if (gamepad1.y && tokenServoPosition >= 0) {
            tokenServoPosition -= 0.03;
        }
        myRobot.markerServoDrive(tokenServoPosition);
        //Collector Flipper Controls
        double cFlipPower = 0;
        int cFlipEncoder = myRobot.getCFlipEncoder();
        int eMotorEncoder = myRobot.getElevatorEncoder();
        if (gamepad1.right_trigger > 0.7) {
            cFlipPower = 0.4;
        } else if (gamepad1.left_trigger > 0.7) {
            cFlipPower = -0.8;
        } else if (!gamepad2.a && !gamepad2.b) {
            cFlipPower = 0;
        } else if (gamepad2.a) {
            cFlipPower = 0.4;
        } else if (gamepad2.b) {
            cFlipPower = -0.8;
        }
        /*
        if(gamepad2.dpad_left && gamepad2.dpad_right){
        cFlipCheck = false;
        }
        if ((Math.abs(cFlipEncoder) > RoverRuckusConstants.TICKS_PER_ROTATION / 4) && (cFlipPower > 0)) {
            if(elevatorDistance>10 && elevatorDistance<800){
                cFlipPower = 0;
                telemetry.addData("StoppedFlipElevatorDistance", elevatorDistance);
                Log.d("StopFlip","cFlipEncoder" + Math.abs(cFlipEncoder) + "Elevator too High?" + elevatorDistance);
            }

            if(extenderLimitSwitch){
                cFlipPower = 0;
                telemetry.addData("StoppedFlipLimitSwitch", cFlipEncoder);
                Log.d("StopFlip","ExtenderLimitSwitch:"+extenderLimitSwitch + "cFlipEncoder" + Math.abs(cFlipEncoder));
            }
        }
        */
        myRobot.cFlipDrive(cFlipPower);

        //Elevator Flipper Controls
        if (gamepad2.x && elevatorServoPosition < 1) {
            elevatorServoPosition = 1;
        }
        if (gamepad2.y && elevatorServoPosition > 0) {
            elevatorServoPosition = 0.45;
        }

        myRobot.elevatorServoDrive(elevatorServoPosition);

        telemetry.addData("", "Run Time: " + runtime.toString() + " Angle: " + myRobot.getHorizontalAngle());
        //telemetry.addData("", "LeftDistanceSensor: " + myRobot.getLeftDistanceSensor() + " RightDistanceSensor: "+myRobot.getRightDistanceSensor());
        //telemetry.addData("colorSensor", "Red: " + myRobot.getColorSensorRed() + " Blue: " + myRobot.getColorSensorBlue());

        String extenderData =  extenderDistance + "exMotorPower: " + exMotorPower+ "Encoder: " + extenderEncoder;
        telemetry.addData("extender: at ", extenderData);
        Log.d("extender System, ", ""+extenderData);

        String elevatorData = "Dist:"+elevatorDistance + " Power:" + elevatorPower + "Encoder" + eMotorEncoder + " Servo:" + elevatorServoPosition;
        telemetry.addData("Elevator ", elevatorData);
        Log.d("Elevator", elevatorData + "ElevatorServoPosition:" + elevatorServoPosition);

        telemetry.addData("TokenServoPosition", tokenServoPosition);
        Log.d("TokenServoPosition", ""+tokenServoPosition);

        String flipperData = "Power" + cFlipPower + "EncoderValue" + cFlipEncoder;
        telemetry.addData("cFlip: ", flipperData);
        Log.d("cFlipper", flipperData);

        myRobot.readEncoders();

        telemetry.addData("collectorServoPower", cServoPower);

    }

    private double elevatorTeleOp(double elevatorDistance, double elevatorPower) {
        //Set holding powers
        if (gamepad2.left_stick_y == 0) {
            if (elevatorDistance > 30) {
                elevatorPower = -0.1;
            } else {
                elevatorPower = -0.05;
            }
        }
        //Stop motor from going too low
        if (elevatorDistance < 5 && gamepad2.left_stick_y > 0) {
            elevatorPower = 0;
            telemetry.addData("DriveOptimization", "PowerCutForElevator");
            Log.d("DriveOptimization", "PowerCutForElevator");
        }
        if (elevatorDistance < 46 && elevatorDistance > 20 && gamepad2.left_stick_y < 0) {
            elevatorServoPosition = 0.7;
        }
        if (elevatorPower > 0 && elevatorDistance < 40) {
            elevatorServoPosition = 1;
        }

        myRobot.eMotorDrive(elevatorPower);

        if(elevatorDistance < 3){
            myRobot.resetElevatorEncoder();
        }
        return elevatorPower;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
