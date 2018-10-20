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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


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

@Autonomous(name="CloseRedAutonomous", group="Linear Opmode")
//@Disabled
public class RoverRuckusAutonomousCloseRed extends LinearOpMode {

    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DetectGoldMineral goldVision;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RoverRuckusClass myRobot = new RoverRuckusClass();
        goldVision = new DetectGoldMineral();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        goldVision.enable();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Initialize

            myRobot.initialize(hardwareMap, telemetry);

            //Lower the robot onto the field 5 seconds
            myRobot.leadScrewDrive(1);
            sleep(3000);
            myRobot.leadScrewDrive(0);
            //Move sideways to detach from the hook
            myRobot.encoderStrafeDrive(200, 0.5, "left");
            //Face back in original direction
            if(Math.abs(myRobot.getHorizontalAngle())>3){
                myRobot.encoderTurn(0,10,3,0.1);
            }
            //Scan two particles
            List<MatOfPoint> contours = goldVision.getContours();
            List<Rect> readContours;
            readContours = new ArrayList<Rect>();
            for (int i = 0; i < contours.size(); i++) {
                Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                //Filter out anything above about half of the y axis
                if((boundingRect.y + boundingRect.width)/2>=200){
                    readContours.add(boundingRect);
                }
            }
            //Print out filtered contours to telemetry
            for(int i = 0; i<readContours.size();i++){
                Rect boundingRect = readContours.get(i);
                telemetry.addData("contour" + Integer.toString(i),
                        String.format(Locale.getDefault(), "(%d, %d)", (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));
            }
            //Deduce where the gold particle is
            String position = "Left";
            //Size of rectangle: (240,320)
            if(readContours.isEmpty()){
                telemetry.addData("Position", position);
            }
            else{
                Rect presumedParticle = readContours.get(0);
                if((presumedParticle.x+presumedParticle.width)/2>=180){
                    position = "Right";
                    telemetry.addData("Position", position);
                }
                else{
                    position = "Center";
                    telemetry.addData("Position", position);
                }
            }
            telemetry.update();
            goldVision.disable();
            //Move forward to clear the hook
            myRobot.encoderTankDrive(200,200, 0.5);
            //Move sideways to recenter
            myRobot.encoderStrafeDrive(200, 0.5, "right");
            //Lower the leadScrew
            myRobot.leadScrewDrive(-1);
            sleep(3000);
            myRobot.leadScrewDrive(0);




            //Drive forward to clear the lander
            myRobot.encoderTankDrive(400,400,0.3);
            sleep(100);

            //Align to the mineral, drive forward, drive back, then align to the left particle
            if(position == "Left"){
                myRobot.encoderStrafeDrive(300, 0.3, "Left");
                myRobot.encoderTankDrive(500,500,0.3);
                myRobot.encoderTankDrive(-500,-500,0.3);
            }
            if(position == "Right"){
                myRobot.encoderStrafeDrive(300, 0.3, "Right");
                myRobot.encoderTankDrive(500,500,0.3);
                myRobot.encoderTankDrive(-500,-500,0.3);
                myRobot.encoderStrafeDrive(600,0.3,"Left");
            }
            if(position == "Center"){
                myRobot.encoderTankDrive(500,500,0.3);
                myRobot.encoderTankDrive(-500,-500,0.3);
                myRobot.encoderStrafeDrive(300,0.3,"Left");
            }

            //Turn around the robot so it will be able to place the marker
            myRobot.encoderTurn(180,60,10,0.2);

            //Move sideways and align to the wall
            myRobot.allDrive(0.3,-0.3, -0.3,0.3);
            sleep(1000);
            myRobot.allDrive(0.1,-0.1, -0.1,0.1);
            sleep(1000);
            //Move sideways to separate from the wall
            myRobot.encoderStrafeDrive(300,0.5,"Left");
            //Move forward to the depot
            //45 inches*(1 rotation/6pi inches)*(1120 ticks/1 rotation) = 26734
            myRobot.encoderTankDrive(2500,2500,0.3);
            //Drive into the crater 5 seconds
            myRobot.driveUntilCrater(-0.3);


            //Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            break;
        }
    }
}
