package org.firstinspires.ftc.Robot3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Eric on 3/19/2017.
 */
public class EncoderBasedNavigator {
    // Declare motors
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    Telemetry telemetry;
    public static final int ErrorMargin = 10;

    public void Init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        telemetry.addData("Status", "Initialized");
        //Initialize motors
        leftMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor = hardwareMap.dcMotor.get("Right_Motor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");

    }

    public void DriveByEncoder(int leftPosition, int rightPosition, double power) {
        telemetry.addData("x input", leftPosition);
        telemetry.addData("y input", rightPosition);
        //Reset Encoders for second time through
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target position
        leftMotor.setTargetPosition(leftPosition);
        rightMotor.setTargetPosition(rightPosition);

        //Set to RUN_TO_POSITION mode
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        // Floor may not be perfect let it slip and exit with error.
        int loopCount = 0;
        int breakoutLimit = 3000;

        while(leftMotor.isBusy() || rightMotor.isBusy())
        {
            loopCount++;

            //Wait until target position is reached
            int deltax = Math.abs(leftMotor.getCurrentPosition() - leftPosition);
            int deltay = Math.abs(rightMotor.getCurrentPosition() - rightPosition);

            double slowdownBase = 4 * ErrorMargin;
            if (deltax <  slowdownBase)
            {
                leftMotor.setPower(power * deltax/slowdownBase);
            }

            if (deltay < slowdownBase)
            {
                rightMotor.setPower(power * deltay/slowdownBase);
            }

            double rate = 1.0 + 1.0 * loopCount / breakoutLimit;
            if (deltax < rate * ErrorMargin && deltay < rate * ErrorMargin) {
                telemetry.addData("exit delta x", deltax);
                telemetry.addData("exit delta y", deltay);
                break;
            }

        }

        //Stop and change modes back to normal
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());
    }
}

