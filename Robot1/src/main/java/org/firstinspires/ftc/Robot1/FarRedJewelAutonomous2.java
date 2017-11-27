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

@Autonomous(name="FarRedAutonomous", group="Linear Opmode")
//@Disabled
public class FarRedJewelAutonomous2 extends LinearOpMode {
    private static final double TICKS_PER_INCH = 1120 / (Math.PI * 4.0);
    double speed = 0.5; //limit is cool and good
    double close = 25; //Determines when the robot begins slowing down
    double enough = 2; //Determines margin of error
    double minflickerPosition = 0.5;//Retracted flicker
    double minjewelarmPosition = 0.67;//Retracted jewelArmPosition
    double maxjewelarmPosition = 0.25;//Extended jewelArmPosition
    int inches = 12;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    MecanumBot mecanumBot = new MecanumBot();
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()){
            mecanumBot.initAutoMecanumBot(hardwareMap, telemetry);

            //Getting the motors and servos in the right place
            mecanumBot.flick(minflickerPosition);
            mecanumBot.moveSideBar(0.2);

            //Knock off the jewel and return the arms
            mecanumBot.knockoffjewel(0.0,1.0,minflickerPosition);
            //Determine Pattern and change drive distance
            if(mecanumBot.DetectPattern().equals("Left")){
                inches+=7.5;
            }
            else if(mecanumBot.DetectPattern().equals("Right")){
                inches-=7.5;
            }



            //Raise arm
            mecanumBot.controlArm(0.5);
            //Drive forward
            mecanumBot.encoderTankDrive((int)(21*TICKS_PER_INCH), (int)(21*TICKS_PER_INCH), 0.5);
            //Drive sideways
            mecanumBot.encoderStrafeDrive((int)(inches*TICKS_PER_INCH),0.5,"left");

            //Lower Arm
            mecanumBot.controlArm(-0.1);
            sleep(1000);
            mecanumBot.controlArm(0.0);


            //Drive glyph into box
            mecanumBot.encoderTankDrive((int)TICKS_PER_INCH*15,(int)TICKS_PER_INCH*15,speed);
            mecanumBot.tankDrive(0,0);
            mecanumBot.moveSideBar(0.5);

            telemetry.addData("encoderPosition", mecanumBot.getEncoderPosition());
            telemetry.addData("gyroPosition", mecanumBot.getAngle());
            telemetry.update();

            //Back Up
            mecanumBot.moveSideBar(0.6);
            mecanumBot.encoderTankDrive((int)TICKS_PER_INCH*-9,(int)TICKS_PER_INCH*-9, -speed);
            mecanumBot.br8kMotors();

            break;

        }
    }

    //region OldEncoderMethod
    /*
    private void EncoderTurn() {
        while (mecanumBot.driveMotorsBusy()) {
            telemetry.addData("encoderPosition", mecanumBot.getEncoderPosition());
            telemetry.addData("gyroPosition", mecanumBot.getAngle());

            if(mecanumBot.getAngle() < -90 + close && mecanumBot.getAngle() > -90-close) {
                speed = 0.1;
                mecanumBot.tankDrive(speed, speed);
                if (mecanumBot.getAngle() < -90 + enough && mecanumBot.getAngle() > -90 - enough) {
                    telemetry.update();
                    break;
                }
            }
            telemetry.update();
        }
    }
    */
    //endregion
}
