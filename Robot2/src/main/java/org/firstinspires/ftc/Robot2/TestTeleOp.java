package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="VectorTeleOp", group="Test Sensors")

public class TestTeleOp extends OpMode {

    MecanumBot myRobot = new MecanumBot();
    Gamepad g = gamepad1;

    final double dpad_speed = 0.3;
    double theta = 0.0, v_theta = 0.0, v_rotation = 0.0;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    double armPower = 0.0;
    @Override
    public void init() {

        myRobot.initMecanumBot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }
    @Override
    public void loop() {
        if (g.dpad_up) {
            theta = 0.0;
            v_theta = dpad_speed;
        } else if (g.dpad_down) {
            theta = Math.PI;
            v_theta = dpad_speed;
        } else if (g.dpad_left) {
            theta = 3.0 * Math.PI / 2.0;
            v_theta = dpad_speed;
        } else if (g.dpad_right) {
            theta = Math.PI / 2.0;
            v_theta = dpad_speed;
        } else {
            final double lx = g.left_stick_x;
            final double ly = -g.left_stick_y;

            theta = Math.atan2(lx, ly);
            v_theta = Math.sqrt(lx * lx + ly * ly);
            v_rotation = g.right_stick_x;
        }
        if (g.right_bumper || g.left_bumper) {
            theta += Math.PI / 2.0;
        }
        myRobot.drive(theta, v_theta, v_rotation); //move robot

        if (g.left_trigger > 0.0) {
            armPower = g.left_trigger * 0.3; // Maximum speed of arm motor os 0.3
        } else if (g.right_trigger > 0.0)
            armPower = g.right_trigger * -0.3;
        else armPower = 0.0;
        myRobot.controlArm(armPower); //move arm up and down

        telemetry.addData("jewel color", myRobot.readJewelColor());
        telemetry.addData("floor color", myRobot.readFloorColor());
        telemetry.addData("heading", myRobot.getAngle());
        telemetry.update();
    }
}
