/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlueActuatorColorSensorLinear", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class BlueActuatorColorSensorLinear extends LinearOpMode {

    /* Declare OpMode members. */
    //Importing the timer
    private ElapsedTime runtime = new ElapsedTime();
    //Importing Color Sensor stuff
    byte[] colorCcache;
    I2cDevice colorC;
    I2cDeviceSynch colorCReader;
    //defining motors
    Servo servo;
    double servoPosition = 0.0;
    DcMotor leftMotor;
    DcMotor rightMotor;
    //Defining booleans
    boolean timeToStop = false;
    boolean finished = false;
    boolean started = false;
    //Importing the encoder based navigator
    EncoderBasedNavigator navigator;


    @Override
    public void runOpMode() throws InterruptedException {
        //add the basic telemetries
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Define and prepare the color sensor
        colorC = hardwareMap.i2cDevice.get("cc");
        colorCReader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCReader.engage();
        colorCcache = colorCReader.read(0x04, 1);
        colorCReader.write8(3,1);
        //Define the servo
        servo = hardwareMap.servo.get("servo");
        servo.setPosition(servoPosition);
        //Define and prepare the motors
        leftMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor = hardwareMap.dcMotor.get("Right_Motor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Define Navigators
        navigator= new EncoderBasedNavigator();
        navigator.Init(this.telemetry, this.hardwareMap);
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //if (!reachedBeacon1)
            //{
            //    reachedBeacon1 = true;
            //}


            //At the beginning of the program, fully extend the servo
            if (!started && servoPosition != 1.0){
                servoPosition=1.0;
                servo.setPosition(servoPosition);
                sleep(2000);
            }
            //if bot drove past beacon, run check
            if (finished && colorCcache [0]==0){
                check();
            }
            //otherwise, run beacon pressing
            else{
                BeaconPressing();
            }
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    private void BeaconPressing() {
        double power = 0.5;
        timeToStop = false;
        telemetry.addData("2 #C", colorCcache[0] & 0xFF);
        telemetry.addData("Stage", "Phase"+"ColorDetection");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        //Drive forward
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        colorCcache = colorCReader.read(0x04, 1);

        //display values


        if (colorCcache[0] > 9 && servoPosition != 1.0){
            finished=true;
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);
            telemetry.addData("Stage", "detectedRed");
            telemetry.update();
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            double servoPosition=1.0;
            servo.setPosition(servoPosition);
            sleep(5000);
            navigator.DriveByEncoder(1400,1400,0.5);
        }
        if (colorCcache[0] < 4 && colorCcache[0] >0 && servoPosition != 0.5) {
            finished = true;
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);
            telemetry.addData("Stage", "detectedBlue");
            telemetry.update();
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            double servoPosition=0.5;
            servo.setPosition(servoPosition);
            sleep(3000);
            navigator.DriveByEncoder(300,0,0.5);
            timeToStop = true;
            leftMotor.setPower(power);
            rightMotor.setPower(power);
            navigator.DriveByEncoder(2800,2800,0.5);
            navigator.DriveByEncoder(-300,0,0.5);
        }
        telemetry.addData("Status", "Termination" + "finished");
        telemetry.update();
        started = true;
    }
    private void check() {
        telemetry.addData("2 #C", colorCcache[0] & 0xFF);
        telemetry.addData("Stage", "checking");
        telemetry.update();
        servoPosition=1.0;
        servo.setPosition(servoPosition);
        sleep(3000);
        navigator.DriveByEncoder(0,280,0.1);
        navigator.DriveByEncoder(-6000,-6000,0.5);
        navigator.DriveByEncoder(0,-300, 0.1);
        finished=false;
    }
}