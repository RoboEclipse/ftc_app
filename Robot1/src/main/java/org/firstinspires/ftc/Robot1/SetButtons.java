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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="SetButtonsPositions", group="Iterative Opmode")
@Disabled
public class SetButtons extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double position=0.5;
    double relicHandServoPos=0.5;
    MecanumBot mecanumBot = new MecanumBot();
    ButtonPositions buttonPositions = new ButtonPositions();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        mecanumBot.initMecanumBot(hardwareMap, telemetry);
        // set the telemetry reference so that ButtonPositions can display
        // error during reading/writing if any
        ButtonPositions.setTelemetry(telemetry);

        buttonPositions.setXPosition (mecanumBot.ReadxButton());
        buttonPositions.setYPosition (mecanumBot.ReadYButton());

        telemetry.addData("Status", "Initialized");
        telemetry.addData("xPosition: ", buttonPositions.getXPosition());
        telemetry.addData("yPosition: ", buttonPositions.getYPosition());
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
        if(gamepad2.dpad_up){
            position+=0.002;
        }
        else if(gamepad2.dpad_down){
            position-=0.002;
        }
        if(gamepad2.dpad_right){
            relicHandServoPos+=0.03;
            if(relicHandServoPos>1){
                relicHandServoPos=1;
            }
        }
        if(gamepad2.dpad_left){
            relicHandServoPos-=0.03;
            if(relicHandServoPos<0){
                relicHandServoPos=0;
            }
        }
        if(position>1.0){
            position=1.0;
        }
        if(position<0.0){
            position=0.0;
        }
        mecanumBot.moveRelicArmServo(position);
        mecanumBot.moveRelicHandServo(relicHandServoPos);
        if(gamepad2.x) {
            buttonPositions.setXPosition(position);
            buttonPositions.WritePositions(buttonPositions);
        }
        if(gamepad2.y){
            buttonPositions.setYPosition(position);
            buttonPositions.WritePositions(buttonPositions);
        }

        if (gamepad2.x || gamepad2.y) {
            ButtonPositions bp = ButtonPositions.ReadPositions();
            if (bp != null) {
                telemetry.addData("xPresetPos:", bp.xPositiontoString());
                telemetry.addData("yPresetPos:", bp.yPositiontoString());
            } else {
                telemetry.addData("xPresetPos:", "failed to read");
                telemetry.addData("yPresetPos:", "failed to read");
            }
            telemetry.update();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
