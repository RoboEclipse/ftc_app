package org.firstinspires.ftc.Robot3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="FTC11138: Test Gyro Sensor", group="Test Sensors")

public class TestGyroSensor extends OpMode {

    MecanumBot myRobot = new MecanumBot();
    double gyroAngle;
    double gyroGravity;
    /*
     * Code to run ONCE when the driver hits INIT
     */
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
        gyroAngle = myRobot.getAngle();
        gyroGravity = myRobot.getGravity();
        // Display the current value
        telemetry.addData("current heading: ", gyroAngle );
        telemetry.addData("current gravity: ", gyroGravity);
        telemetry.update();
    }
}
