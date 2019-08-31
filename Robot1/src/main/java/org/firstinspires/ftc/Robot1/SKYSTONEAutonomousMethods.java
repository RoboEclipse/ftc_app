package org.firstinspires.ftc.Robot1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

abstract class SKYSTONEAutonomousMethods extends LinearOpMode {


    //Hardware

    //Software
    private Telemetry telemetry;

    //Classes
    private SKYSTONEClass myRobot = new SKYSTONEClass();
    private SKYSTONEConfiguration skystoneNames = new SKYSTONEConfiguration();
    private SKYSTONEConstants skystoneConstants = new SKYSTONEConstants();

    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        myRobot.initialize(hardwareMap, telemetry);
        this.telemetry = telemetry;
    }
    void waitForStart2(){
        ElapsedTime time  = new ElapsedTime();
        while (!isStarted()){
            double clock = time.seconds();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Time", clock);
            telemetry.update();
        }
    }

    //Methods
    void encoderStraightDriveInches(double inches, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        multiSetTargetPosition(inches*skystoneConstants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (anyBusy() // && opModeIsActive()
        ){
            telemetry.addData("Left Front: ", myRobot.lf.getCurrentPosition());
            telemetry.addData("Left Back: ", myRobot.lb.getCurrentPosition());
            telemetry.addData("Right Front: ", myRobot.rf.getCurrentPosition());
            telemetry.addData("Right Back: ", myRobot.rb.getCurrentPosition());
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Shortcuts
    private void setModeAllDrive(DcMotor.RunMode mode){
        myRobot.lb.setMode(mode);
        myRobot.lf.setMode(mode);
        myRobot.rb.setMode(mode);
        myRobot.rf.setMode(mode);
    }
    private void multiSetTargetPosition(double ticks, DcMotor...motors){
        for(DcMotor motor:motors){
            motor.setTargetPosition((int)Math.round(ticks));
        }
    }
    private boolean anyBusy(){
        return myRobot.lb.isBusy() || myRobot.lf.isBusy() || myRobot.rb.isBusy() || myRobot.rf.isBusy();
    }
    private void runMotors (double leftPower, double rightPower){
        myRobot.lb.setPower(leftPower);
        myRobot.lf.setPower(leftPower);
        myRobot.rb.setPower(rightPower);
        myRobot.rf.setPower(rightPower);
    }

}
