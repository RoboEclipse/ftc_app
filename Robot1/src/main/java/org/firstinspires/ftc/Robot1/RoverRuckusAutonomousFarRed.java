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

@Autonomous(name="FarRedCloseBlueGolden", group="Linear Opmode")
//@Disabled
public class RoverRuckusAutonomousFarRed extends RoverRuckusAutonomousMethods {

    int strafeInches = 15;
    int reverseInches = -26;
    int tolerance = RoverRuckusConstants.tolerance;

    @Override
    public void runOpMode() {

        RoverRuckusClass myRobot = initialize();
        //Initialize
        waitForStartTensorFlow(myRobot);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            LandingFull(myRobot);
            rotateSample(myRobot);
            myRobot.encoderStrafeDrive(ticksPerInch*5, 0.6, "Left");
            myRobot.encoderTurn(135,30,tolerance,0.6);
            myRobot.rightRangeSensorStrafe(ticksPerInch*strafeInches, RoverRuckusConstants.wallDistance, 0.4,"Right");
            myRobot.colorSensorDrive(ticksPerInch*reverseInches, 0.7);
            ClaimFull(myRobot);
            myRobot.encoderTurn(135,30,tolerance,0.6);
            myRobot.encoderTankDriveInches(RoverRuckusConstants.park/2, 0.8);
            if(myRobot.getRightDistanceSensor()> RoverRuckusConstants.wallDistance+2
                    || myRobot.getRightDistanceSensor()<RoverRuckusConstants.wallDistance-2){
                myRobot.rightRangeSensorStrafe(200,RoverRuckusConstants.wallDistance, 0.6, "Right");
            }
            newParking(myRobot, 135, RoverRuckusConstants.park/2);
            //rightParking(myRobot, 135, RoverRuckusConstants.wallDistance);


            // Show the elapsed game time and wheel power.
            break;

        }
        AutoTransitioner.transitionOnStop(this,"TeleOp");
    }
}
