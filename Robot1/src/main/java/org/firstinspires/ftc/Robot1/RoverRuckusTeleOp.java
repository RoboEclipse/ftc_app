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
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

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
    //private double elevatorServoPosition = 1;
    private double elevatorServoPosition = 0.7;
    private double tokenServoPosition = 0;
    private int stage = 0;
    //private double tiltPosition = 0.59;
    private double tiltPosition = 0.187;
    private boolean fast = false;
    private boolean retracting = false;
    private boolean dumping = false;
    private ElapsedTime tiltTime = new ElapsedTime();
    boolean LED = true;
    private ElapsedTime dumpTime = new ElapsedTime();
    private ElapsedTime returnTime = new ElapsedTime();
    ExpansionHubEx expansionHub;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RevExtensions2.init();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
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
        double rotationMultiplier = .8;
        if(gamepad1.dpad_up){
            ly=1;
            speedMultiplier = 0.75;
        }
        else if(gamepad1.dpad_down){
            ly=-1;
            speedMultiplier = 0.75;
        }
        if(gamepad1.dpad_left){
            lx=-1;
            speedMultiplier = 0.75;
        }
        else if(gamepad1.dpad_right){
            lx=1;
            speedMultiplier = 0.75;
        }

        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation); //move robot

        if(stage == 0) {
            driver2Manual();
            fast = false;
        }
        else {
            if (gamepad2.dpad_right || gamepad2.right_trigger>0.99) {
                stage = 0;
                elevatorServoPosition=.7;
            }
            /*
            if(gamepad2.y){
            if(gamepad2.y){
                fast = true;
                elevatorServoPosition=0.6;
            }
            else if(!fast){
                elevatorServoPosition=0.8;
            }
            */
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
        } else if (gamepad1.right_bumper) {
            leadScrewPower = -1;
        }
        if (leadScrewPower > 0) {
            if (!myRobot.isElevatorLimitSwitchNOTPressed()) {
                leadScrewPower = 0;
                telemetry.addData("LeadScrewAutoStopped", "True");
            }
        }
        myRobot.leadScrewDrive(leadScrewPower);

        //Elevator Motor Controls
        double elevatorDistance = myRobot.getElevatorDistanceSensor();
        double elevatorPower = gamepad2.left_stick_y;
        elevatorPower = elevatorTeleOp(elevatorDistance, elevatorPower);
        int eMotorEncoder = myRobot.getElevatorEncoder();

        //Collector Servo Control
        double cServoPower = 0;
        //Eject
        if (gamepad2.left_bumper) {
            cServoPower = -0.89;
        } else if (gamepad2.right_bumper) {
            //Collect
            cServoPower = 0.89;
            myRobot.resetCFlipEncoder();
        }
        myRobot.cServoDrive(cServoPower);

        //Collector Extender Controls
        double extenderDistance = myRobot.getExtenderDistanceSensor();
        double exMotorPower = 0;
        int extenderEncoder = myRobot.getExtenderEncoder();
        if (gamepad2.dpad_up) {
            exMotorPower = 1;
            if(extenderDistance<=3){
                exMotorPower = 0.1;
                telemetry.addData("Slow", "true");
                myRobot.resetExtenderEncoder();
            }
        } else if (gamepad2.dpad_down) {
            exMotorPower = -1;
            if(extenderDistance>24){
                exMotorPower = -0.3;
                telemetry.addData("Slow", "true");
            }
        }
        myRobot.newExMotor(exMotorPower);

        //Token servo controls
        if (gamepad1.x && tokenServoPosition <= 1) {
            tokenServoPosition += 0.03;
        } else if (gamepad1.y && tokenServoPosition >= 0) {
            tokenServoPosition -= 0.03;
        }
        myRobot.markerServoDrive(tokenServoPosition);

        //Collector Flipper Controls
        double cFlipPower;
        int cFlipEncoder = myRobot.getCFlipEncoder();
        if (gamepad1.right_trigger > 0.7) {
            cFlipPower = 0.4;
        } else if (gamepad1.left_trigger > 0.7) {
            cFlipPower = -0.8;
        } else if (!gamepad2.a && !gamepad2.b) {
            cFlipPower = 0;
        } else if (gamepad2.a) {
            cFlipPower = 0.4;
        } else {
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
            //elevatorServoPosition = 1;
            elevatorServoPosition = 0.7;
        }
        if (gamepad2.dpad_left){
            elevatorServoPosition = tiltPosition+0.1;
        }
        if (gamepad2.y) {
            /*
            returnTime.reset();
            retracting = true;
            elevatorServoPosition = 1;
            */
            elevatorServoPosition=tiltPosition;
            //tiltTime.reset();
            //dumping=true;
        }
        else{
            dumpTime.reset();
            if(retracting){
                retracting=myRobot.autoRetract(returnTime.milliseconds());
            }
        }
        if(!retracting){
            myRobot.elevatorServoDrive(elevatorServoPosition);
        }

        if(dumping /*&& tiltTime.milliseconds()>100*/){
            myRobot.teleOpSleep(150);
            elevatorServoPosition=tiltPosition;
            dumping=false;
        }

        double ledPower = getLedPower();
        myRobot.ledControl(ledPower);

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

        telemetry.addData("FlipTime", dumpTime);

        myRobot.readEncoders();

        telemetry.addData("collectorServoPower", cServoPower);

    }

    private double getLedPower() {
        int[] leftColorSensor = myRobot.getLeftColorSensor();
        int[] rightColorSensor = myRobot.getRightColorSensor();
        boolean left = (leftColorSensor[0]>1000 && leftColorSensor[1]>700 && leftColorSensor[2]>1000);
        boolean right = (rightColorSensor[0]>1000 && rightColorSensor[1]>700 && rightColorSensor[2]>1000);
        double LEDPower;
        if(gamepad1.b){
            if(LED){
                LED=false;
            }
            else{
                LED=true;
            }
        }
        if(LED){
            if(left || right){
                if(!left || !right){
                    LEDPower=0.6;
                }
                else{
                    LEDPower=0.71;
                }
            }
            else {
                LEDPower=0.6657;
            }
        }
        else{
            LEDPower=0.4768;
        }
        telemetry.addData("Left: ", "Red: "+ leftColorSensor[0] + " Blue: " + leftColorSensor[1] + " Green: " + leftColorSensor[2]);
        telemetry.addData("Right: ", "Red: "+ rightColorSensor[0] + " Blue: " + rightColorSensor[1] + " Green: " + rightColorSensor[2]);
        telemetry.addData("LEDPower", LEDPower);
        return LEDPower;
    }

    private double elevatorTeleOp(double elevatorDistance, double elevatorPower) {
        //Stuff that happens when it's going downwards
        if(elevatorPower>0){
            //Stop motor from going too low
            if(elevatorDistance<5){
                elevatorPower = 0;
                telemetry.addData("DriveOptimization", "PowerCutForElevator");
                Log.d("DriveOptimization", "PowerCutForElevator");
            }
            //Return servo to default position at certain point
            if(elevatorDistance<48){
                elevatorServoPosition = .7;
            }
            elevatorPower=elevatorPower*myRobot.elevatorModifier*0.2;
        }
        //Stuff that happens when it's going upwards
        if(elevatorPower<0){
            //Half-rotate at certain heights
            if(elevatorDistance < 46 && elevatorDistance > 20){
                elevatorServoPosition = 0.4;
            }
            //Slow down at certain heights
            /*
            if(elevatorDistance>40){
                elevatorPower *= 4/5;
            }
            */
        }
        //Set holding powers
        if (gamepad2.left_stick_y == 0) {
            if (elevatorDistance > 30) {
                elevatorPower = -0.3;
            }
            else if (elevatorDistance<10){
                elevatorPower = 0;
            }
            else {
                elevatorPower = -0.2;
            }
        }

        if(!retracting){
            myRobot.eMotorDrive(elevatorPower);
        }
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
