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

@Autonomous(name="EncoderTest", group="Linear Opmode")
//@Disabled
public class EncoderTest extends LinearOpMode {
    private static final double TICKS_PER_INCH = 1120 / (Math.PI * 4.0);
    double speed = 0.5; //limit is cool and good
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
        mecanumBot.flick(0.5);
        mecanumBot.moveSideBar(0.5);
        mecanumBot.controlArm(0.5);
        sleep(1000);
        mecanumBot.controlArm(0.1);
        sleep(2000);
        mecanumBot.controlArm(0.0);

        //Knock off the jewel
        while (!knocked){
            mecanumBot.setJewelArm(1.0);
            blue = mecanumBot.GetJewelSensorBlue();
            red = mecanumBot.GetJewelSensorRed();
            if (red>blue*2.2) {
                mecanumBot.flick(1.0);
                knocked = true;
            }
            if (blue>red+15) {
                mecanumBot.flick(0.0);
                knocked = true;
            }
        }
        mecanumBot.flick(0.5);
        mecanumBot.setJewelArm(0.4);

        //Determine Pattern
        pattern.onLoop();
        telemetry.addData("Pattern: " , pattern.vuMark);

        //Drive forward
        mecanumBot.encoderTankDrive((int)TICKS_PER_INCH*27,(int)TICKS_PER_INCH*27,speed);

        //Turn 90 degrees
        mecanumBot.encoderTurn(-90,close, enuff,speed);

        //Lower Arm
        speed = 0.5;
        mecanumBot.controlArm(0.1);
        sleep(2000);
        mecanumBot.controlArm(0.0);

        //Drive glyph into box
        mecanumBot.encoderTankDrive((int)TICKS_PER_INCH*10,(int)TICKS_PER_INCH*10,speed);
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
