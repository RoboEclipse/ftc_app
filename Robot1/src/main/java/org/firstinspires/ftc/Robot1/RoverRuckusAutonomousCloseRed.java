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


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "CloseRedFarBlueSilver", group = "Linear Opmode")
//@Disabled
public class RoverRuckusAutonomousCloseRed extends RoverRuckusAutonomousMethods {
    int extraStrafeInches = 14;
    int strafeInches = -32;
    @Override
    public void runOpMode() {
        //Import classes
        RoverRuckusClass myRobot = initialize();
        //Get mineral positions
        myRobot.initTensorFlow(hardwareMap);
        tensorFlowSetPosition(myRobot);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            //Land
            LandingFull(myRobot);
            //Sample
            SampleFullProcess(myRobot);
            //Line up to wall
            myRobot.encoderStrafeDrive(RoverRuckusConstants.TICKS_PER_INCH * extraStrafeInches, 0.5, "Left");
            myRobot.encoderTurn(-45, 30, 3, 0.4);
            myRobot.leftRangeSensorStrafe(RoverRuckusConstants.TICKS_PER_INCH*4, 20, 0.5, "Left");
            myRobot.encoderTankDrive(strafeInches*RoverRuckusConstants.TICKS_PER_INCH, strafeInches*RoverRuckusConstants.TICKS_PER_INCH, 0.5);
            sleep(100);
            //Place Marker
            myRobot.br8kMotors();
            ClaimFull(myRobot);
            //Park
            Parking(myRobot, -45);
            break;

        }

    }
}