package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class TileRunnerMecanumClass {
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Telemetry telemetry;

    private double wheelCircumference = 6*Math.PI;
    private double TICKS_PER_ROTATION = 1120;
    private double TICKS_PER_INCH = Math.round(wheelCircumference*TICKS_PER_ROTATION);
    private void multiSetMode(DcMotor.RunMode runMode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    private void multiSetTargetPosition(int ticks, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
        }
    }

    private void multiSetPower(double power, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void init(HardwareMap hardwareMap) {
        //Assigning motors in the program to motors on the robot
        frontLeftDrive = hardwareMap.dcMotor.get("front_left_drive");
        frontRightDrive = hardwareMap.dcMotor.get("front_right_drive");
        backLeftDrive = hardwareMap.dcMotor.get("back_left_drive");
        backRightDrive = hardwareMap.dcMotor.get("back_right_drive");
        //clawServo = hardwareMap.servo.get("claw_servo");
        //armDrive = hardwareMap.dcMotor.get("arm_drive");
        //Preparing encoders for use
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        multiSetMode(DcMotor.RunMode.RUN_USING_ENCODER, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double fl, double fr, double bl, double br) {
        frontLeftDrive.setPower(fl);
        backRightDrive.setPower(br);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
    }

    public void strafe(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power * -1);
        frontRightDrive.setPower(power * -1);
        backRightDrive.setPower(power);
    }

    public void encoderDriveInches(double inches, double power) {
        int ticks = (int)Math.round(inches*TICKS_PER_INCH);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        multiSetTargetPosition(ticks, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        multiSetPower(power, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        while (frontRightDrive.isBusy() || frontLeftDrive.isBusy() || backRightDrive.isBusy() || backLeftDrive.isBusy()) {
            telemetry.addData("frontRightDrive: ", frontRightDrive.getCurrentPosition());
            telemetry.addData("frontLeftDrive: ", frontLeftDrive.getCurrentPosition());
            telemetry.addData("backRightDrive: ", frontRightDrive.getCurrentPosition());
            telemetry.addData("backRightDrive: ", frontLeftDrive.getCurrentPosition());
        }
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        multiSetPower(0, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }


















}
