package org.firstinspires.ftc.Robot3;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Nathan on 9/24/2017.
 */

@TeleOp(name="FTC11138: Test Color Sensor", group="Iterative Opmode")
public class TestColorSensor extends OpMode {
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {

        // Display the current value
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();
    }
}
