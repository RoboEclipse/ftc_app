package org.firstinspires.ftc.Robot3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Nathan on 9/24/2017.
 */

@TeleOp(name="FTC11138: Test Grabber Glyphs", group="Iterative Opmode")
public class TestGrabberGlyphs extends OpMode {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    com.qualcomm.robotcore.hardware.Servo leftservo;
    com.qualcomm.robotcore.hardware.Servo rightservo;
    double  rightposition = MIN_POS; // Start at min position
    double  leftposition = MAX_POS; // Start at max position
    boolean isGrabbing = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftservo = HardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "leftarm");
        // Set the servo to the max position first;
        leftservo.setPosition(leftposition);

        rightservo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "rightarm");
        // Set the servo to the max position first;
        rightservo.setPosition(rightposition);

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
            leftposition -= INCREMENT ;
            if (leftposition <= MIN_POS ) {
                leftposition = MIN_POS;
            }
            rightposition += INCREMENT ;
            if (rightposition >= MAX_POS ) {
                rightposition = MAX_POS;
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            leftposition += INCREMENT ;
            if (leftposition >= MAX_POS ) {
                leftposition = MAX_POS;
            }
            rightposition -= INCREMENT ;
            if (rightposition <= MIN_POS ) {
                rightposition = MIN_POS;
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
        telemetry.addData("RedJewel LeftArmPosition", "%5.2f", leftposition);
        telemetry.addData("RedJewel RightArmPosition", "%5.2f", rightposition);
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

        // Set the servo to the new position and pause;
        leftservo.setPosition(leftposition);
        rightservo.setPosition(rightposition);
    }
}

