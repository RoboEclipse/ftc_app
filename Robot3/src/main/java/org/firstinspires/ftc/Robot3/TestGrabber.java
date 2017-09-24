package org.firstinspires.ftc.Robot3;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Nathan on 9/24/2017.
 */

@TeleOp(name="FTC11138: Test Grabber", group="Iterative Opmode")
public class TestGrabber extends OpMode {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    com.qualcomm.robotcore.hardware.Servo servo;
    double  position = MIN_POS; // Start at min position
    boolean isGrabbing = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        servo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "grabber");
        // Set the servo to the max position first;
        servo.setPosition(position);

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
        // slew the servo, according to the rampUp (direction) variable.
        if (isGrabbing) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
            }
        }

        // if A is pressed, then grabbing
        if (gamepad1.a)
        {
            isGrabbing = true;
        }
        // if B is pressed, then releasing
        else if (gamepad1.b)
        {
            isGrabbing = false;
        }

        // Display the current value
        telemetry.addData("RedJewel Position", "%5.2f", position);
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

        // Set the servo to the new position and pause;
        servo.setPosition(position);
    }
}

