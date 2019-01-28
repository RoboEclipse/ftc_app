package org.firstinspires.ftc.Robot1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

class TileRunnerClass {
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, armDrive;
    Servo clawServo;
    private void smartSetMode(DcMotor.RunMode runMode, DcMotor ... motors){
        for(DcMotor motor : motors){
            motor.setMode(runMode);
        }
    }
    private void smartSetTargetPosition(int ticks, DcMotor ... motors){
        for(DcMotor motor : motors){
            motor.setTargetPosition(ticks);
        }
    }

    public void init (HardwareMap hardwareMap){
        //Assigning motors in the program to motors on the robot
        frontLeftDrive  = hardwareMap.dcMotor.get("front_left_drive");
        frontRightDrive = hardwareMap.dcMotor.get("front_right_drive");
        backLeftDrive = hardwareMap.dcMotor.get("back_left_drive");
        backRightDrive = hardwareMap.dcMotor.get("back_right_drive");
        //clawServo = hardwareMap.servo.get("claw_servo");
        //armDrive = hardwareMap.dcMotor.get("arm_drive");
        //Preparing encoders for use
        smartSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, frontLeftDrive,frontRightDrive,backLeftDrive,backRightDrive);
        smartSetMode(DcMotor.RunMode.RUN_USING_ENCODER, frontLeftDrive,frontRightDrive,backLeftDrive,backRightDrive);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    public void drive(double leftPower, double rightPower){
       frontLeftDrive.setPower(leftPower);
       backLeftDrive.setPower(leftPower);
       frontRightDrive.setPower(rightPower);
       backLeftDrive.setPower(rightPower);
    }

    public void encoderDrive(int leftTicks, int rightTicks, double power){
        smartSetMode(DcMotor.RunMode.RUN_USING_ENCODER, frontLeftDrive,frontRightDrive,backLeftDrive,backRightDrive);
        smartSetTargetPosition(leftTicks, frontLeftDrive, backLeftDrive);
        smartSetTargetPosition(rightTicks,frontRightDrive,backRightDrive);
        smartSetMode(DcMotor.RunMode.RUN_TO_POSITION, frontLeftDrive,frontRightDrive,backLeftDrive,backRightDrive);
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        while(frontRightDrive.isBusy() || frontLeftDrive.isBusy() || backRightDrive.isBusy() || backLeftDrive.isBusy()){

        }
        smartSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, frontLeftDrive,frontRightDrive,backLeftDrive,backRightDrive);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
