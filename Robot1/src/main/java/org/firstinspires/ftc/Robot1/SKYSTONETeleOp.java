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
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="SKYSTONETeleOp", group="Iterative Opmode")
//@Disabled
public class SKYSTONETeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private SKYSTONEClass myRobot = new SKYSTONEClass();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
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
        //Drive motor controls
        double lx = -gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
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
        if(gamepad2.a) {
            myRobot.fingers.setPower(1);
        }
        else if (gamepad2.b) {
            myRobot.fingers.setPower(-1);
        }

        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = -gamepad1.right_stick_x;

        myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);
        double leftClawPosition = 0.5;
        if(gamepad2.dpad_up){
            leftClawPosition += 0.01;
        }
        if(gamepad2.dpad_down) {
            leftClawPosition -= 0.01;
        }
        if (gamepad2.b)
        myRobot.setLeftClawPosition(leftClawPosition);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        myRobot.readEncoders();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
