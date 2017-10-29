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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="TankDriveTeleOp", group="Iterative Opmode")
//@Disabled
public class BasicTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lfMotor, lbMotor, rfMotor, rbMotor, arm, extender;
    private Servo claw;
    private MecanumRobot myRobot= null;
    private double clawPosition=1;
    final double dpad_speed = 0.3;
    double theta = 0.0, v_theta = 0.0, v_rotation = 0.0;
    //private Servo left, right;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lfMotor = hardwareMap.get(DcMotor.class, "leftf_motor");
        lbMotor = hardwareMap.get(DcMotor.class, "leftb_motor");
        rfMotor = hardwareMap.get(DcMotor.class, "rightf_motor");
        rbMotor = hardwareMap.get(DcMotor.class, "rightb_motor");
        arm = hardwareMap.get(DcMotor.class, "arm_drive");
        extender = hardwareMap.get(DcMotor.class, "linearslide_motor");
        claw = hardwareMap.get(Servo.class, "claw");
        //left = hardwareMap.get(Servo.class, "left");
        //right = hardwareMap.get(Servo.class, "right");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rfMotor.setDirection(DcMotor.Direction.REVERSE);
        rbMotor.setDirection(DcMotor.Direction.REVERSE);
        MecanumBot myRobot= new MecanumBot();
        myRobot.initMecanumBot(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
        /*
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        telemetry.addData("Status", "Initialized");


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

    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double armPower;
        double extenderPower;



        leftPower  = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;
        armPower = -gamepad2.left_stick_y*0.25;
        extenderPower = -gamepad2.right_stick_y*0.5;

        if(gamepad1.dpad_up){
            leftPower = 1;
            rightPower = 1;
        }
        if(gamepad1.dpad_down){
            leftPower = -1;
            rightPower = -1;
        }
        if (gamepad1.dpad_left) {
            theta = 3.0 * Math.PI / 2.0;
            v_theta = dpad_speed;
            myRobot.drive(theta, v_theta, v_rotation);
        }
        if (gamepad1.dpad_right) {
            theta = Math.PI / 2.0;
            v_theta = dpad_speed;
            myRobot.drive(theta, v_theta, v_rotation);
        }
        /*
        if(gamepad2.left_bumper){
            if(gamepad2.left_trigger>0f){
                left.setPosition(1);
                right.setPosition(1);
            }
            if(gamepad2.right_trigger>0f){
                left.setPosition(0.5);
                left.setPosition(0.5);
            }
        }
        */
        if(gamepad2.a){
            if(clawPosition>0.5){
                clawPosition-=0.1;
            }
        }
        if(gamepad2.b){
            if(clawPosition<1.0){
                clawPosition+=0.1;
            }
        }
        // Send calculated power to wheels
        lfMotor.setPower(leftPower);
        lbMotor.setPower(leftPower);
        rfMotor.setPower(rightPower);
        rbMotor.setPower(rightPower);
        arm.setPower(armPower);
        claw.setPosition(clawPosition);
        extender.setPower(extenderPower);



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
