package org.firstinspires.ftc.Robot1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TileRunnerClass {
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
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
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        //Preparing encoders for use
        smartSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, frontLeftDrive,frontRightDrive,backLeftDrive,backLeftDrive);
        smartSetMode(DcMotor.RunMode.RUN_USING_ENCODER, frontLeftDrive,frontRightDrive,backLeftDrive,backLeftDrive);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void encoderDrive(int leftTicks, int rightTicks, int power){
        smartSetMode(DcMotor.RunMode.RUN_USING_ENCODER, frontLeftDrive,frontRightDrive,backLeftDrive,backLeftDrive);
        smartSetTargetPosition(leftTicks, frontLeftDrive, backLeftDrive);
        smartSetTargetPosition(rightTicks,frontRightDrive,backRightDrive);
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        while(frontRightDrive.isBusy() && frontLeftDrive.isBusy() && backRightDrive.isBusy() && backLeftDrive.isBusy()){

        }
        smartSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, frontLeftDrive,frontRightDrive,backLeftDrive,backLeftDrive);
    }
}
