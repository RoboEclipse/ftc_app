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
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "FTC11138:BlueJewelAutonomous", group = "Sensor")
@Disabled                            // Comment this out to add to the opmode list
public class BlueJewelAutonomousAudienceSide extends LinearOpMode {
    //region Constants
    double maxExtension = 1.0;
    double minExtension = 0.4;
    double power = 0.5;
    //endregion
    //region Variables
    double time = 1000;//If time or power is changed, the other must changed as well
    boolean finished = false;
    ElapsedTime runtime = new ElapsedTime();
    //endregion
    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */
    MecanumBot myRobot= new MecanumBot();
    @Override
    public void runOpMode() {

        myRobot.initMecanumBot(hardwareMap, telemetry);
        // wait for the start button to be pressed.

        waitForStart();
        //Set arm to correct Position
        myRobot.setJewelArm(maxExtension);
        int blue = myRobot.GetJewelSensorBlue();
        int red = myRobot.GetJewelSensorRed();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        // send the info back to driver station using telemetry function.
        telemetry.addData("Red  ", red);
        telemetry.addData("Blue ", blue);

        telemetry.update();

        while (opModeIsActive()) {

            //Read Red and Blue Values

            blue = myRobot.GetJewelSensorBlue();
            red = myRobot.GetJewelSensorRed();
            //Todo: Increase consistency of movement through encoders or gyro
            //If Red is detected, center jewel arm and drive backwards
            //The color sensor works proportionally to light. From expirementation we have determined that if red
            //is over 3 times the value of blue, we can be confident it is red, but it will still detect the red from a distance
            if (red>blue*2 && !finished) {
                myRobot.setJewelArm(maxExtension);
                time = 400;
                sleep(1000);
                myRobot.drive(0,power,0);
                sleep((int)time);
                myRobot.stopDriveMotors();
                finished = true;
                time=2250;
            }
            //If Blue is detected, center jewel arm and drive forwards
            //Note that the sensor detects red much more intensely than blue, so if blue is only slightly greater than red,
            //we can have reasonable confidence that the object is blue
            else if (blue>red+15 && !finished) {
                myRobot.setJewelArm(maxExtension);
                sleep(1000);
                time = 500;
                myRobot.drive(0, -power,0);
                sleep((int)time);
                myRobot.stopDriveMotors();
                finished = true;
                time = 1000;

            }
            /*
            //If no color is detected, adjust arm
            //If arm is raised too high, abort and cancel
            if (red < blue*3 && blue <red && !finished) {
                armPosition -= 0.05 ;
                if (armPosition < maxExtension*0.9) {
                    myRobot.setJewelArm(maxExtension);
                    //finished = true;
                }
            }
            */
            //When Jewel part is ran, raise arm and drive forward
            if (finished) {
                myRobot.setJewelArm(minExtension);
                sleep(1000);
                myRobot.tankDrive(-0.5,-0.44);
                sleep((int)time);
                myRobot.stopDriveMotors();
                break;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // send the info back to driver station using telemetry function.
            telemetry.addData("Red  ", red);
            telemetry.addData("Blue ", blue);

            telemetry.update();

        }


    }
}
