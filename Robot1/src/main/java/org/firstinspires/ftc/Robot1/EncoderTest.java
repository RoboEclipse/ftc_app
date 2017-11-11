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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="EncoderTest", group="Linear Opmode")
//@Disabled
public class EncoderTest extends LinearOpMode {
    private static final double TICKS_PER_INCH = 1120 / (Math.PI * 4.0);
    double speed = 0.5; //limit is good and cool
    double close = 25; //Determines when the robot begins slowing down
    double enuff = 2; //Determines margin of error
    int inches = 27;
    int blue;
    int red;

    boolean knocked = false;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    MecanumBot mecanumBot = new MecanumBot();
    VuMark pattern;
    @Override
    public void runOpMode() {
        mecanumBot.initMecanumBot(hardwareMap, telemetry);
        pattern = new VuMark(telemetry,hardwareMap);
        pattern.onInit(VuforiaLocalizer.CameraDirection.FRONT);
        //Getting the motors and servos in the right place
        mecanumBot.moveSideBar(0.5);
        mecanumBot.controlArm(-0.1);
        sleep(1000);
        mecanumBot.controlArm(0.1);
        sleep(2000);
        mecanumBot.controlArm(0.0);

        while (!knocked){
            //Knock off the jewel
            blue = mecanumBot.GetJewelSensorBlue();
            red = mecanumBot.GetJewelSensorRed();
            if (red>blue*2.2) {

            }
            if (blue>red+15) {

            }
        }
        //Drive forward
        mecanumBot.encoderTankDrive((int)TICKS_PER_INCH*27,(int)TICKS_PER_INCH*27,speed);
        if(pattern.isCenterRelicVisable()){
            telemetry.addData("Pattern ", "Center");
        }
        if(pattern.isLeftRelicVisable()){
            telemetry.addData("Pattern ", "Left");
            inches += 3;
        }
        if(pattern.isRightRelicVisable()){
            telemetry.addData("Pattern ", "Right");
            inches -= 3;
        }

        while (mecanumBot.driveMotorsBusy()) {

            telemetry.addData("encoderPosition", mecanumBot.getEncoderPosition());
            telemetry.addData("gyroPosition", mecanumBot.getAngle());

            telemetry.update();
        }
        //Turn 90 degrees
        mecanumBot.encoderTankDrive((int)TICKS_PER_INCH*inches,(int)TICKS_PER_INCH*-27,speed);
        mecanumBot.encoderTurn(-90,close, enuff);
        //Lower Arm
        speed = 0.5;
        mecanumBot.controlArm(0.1);
        sleep(2000);
        mecanumBot.controlArm(0.0);
        //Drive glyph into box
        mecanumBot.encoderTankDrive((int)TICKS_PER_INCH*10,(int)TICKS_PER_INCH*10,speed);
        while (mecanumBot.driveMotorsBusy()) {
            telemetry.addData("encoderPosition", mecanumBot.getEncoderPosition());
            telemetry.addData("gyroPosition", mecanumBot.getAngle());
            telemetry.update();
        }
        mecanumBot.tankDrive(0,0);
        mecanumBot.br8kMotors();
        mecanumBot.moveSideBar(0.0);
        telemetry.addData("encoderPosition", mecanumBot.getEncoderPosition());
        telemetry.addData("gyroPosition", mecanumBot.getAngle());
        telemetry.update();

    }

    private void EncoderTurn() {
        while (mecanumBot.driveMotorsBusy()) {
            telemetry.addData("encoderPosition", mecanumBot.getEncoderPosition());
            telemetry.addData("gyroPosition", mecanumBot.getAngle());

            if(mecanumBot.getAngle()<-90+close && mecanumBot.getAngle()>-90-close) {
                speed = 0.1;
                mecanumBot.tankDrive(speed, speed);
                if (mecanumBot.getAngle() < -90 + enuff && mecanumBot.getAngle() > -90 - enuff) {
                    telemetry.update();
                    break;
                }
            }
            telemetry.update();
        }
    }
}
