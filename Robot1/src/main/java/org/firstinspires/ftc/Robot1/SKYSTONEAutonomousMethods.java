package org.firstinspires.ftc.Robot1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

abstract class SKYSTONEAutonomousMethods extends LinearOpMode {


    //Hardware
    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
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
        //Sensors
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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

    /** This method is used for testing purposes **/

    void diagonalDriveInchesTestRightForwardFourInchWheel(double power, double time, double wheelToMecanumDistanceX, double wheelToMecanumDistanceY) {
        int wheelPower1 = 0;
        int wheelPower2 = 0;
        int wheelPower3 = 0;
        int wheelPower4 = 0;
        double rotationDistance = 4 * Math.PI;
        myRobot.encoderStraightDriveInches(4,5);

    }

    /** The method will be used to move diagonally in both Autonomous and TeleOp**/




    //Methods
    void encoderStraightDriveInches(double inches, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        multiSetTargetPosition(inches*SKYSTONEConstants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (anyBusy() /*&& opModeisActive()*/){
            telemetry.addData("Left Front: ", myRobot.lf.getCurrentPosition());
            telemetry.addData("Left Back: ", myRobot.lb.getCurrentPosition());
            telemetry.addData("Right Front: ", myRobot.rf.getCurrentPosition());
            telemetry.addData("Right Back: ", myRobot.rb.getCurrentPosition());
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Negative = Left, Positive = Right
    void encoderStrafeDriveInchesRight(double inches, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.lf.setTargetPosition((int)Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.lb.setTargetPosition(-(int)Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.rf.setTargetPosition(-(int)Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.rb.setTargetPosition((int)Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        multiSetTargetPosition(inches*SKYSTONEConstants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (anyBusy() /*&& opModeisActive()*/){
            telemetry.addData("Left Front: ", myRobot.lf.getCurrentPosition());
            telemetry.addData("Left Back: ", myRobot.lb.getCurrentPosition());
            telemetry.addData("Right Front: ", myRobot.rf.getCurrentPosition());
            telemetry.addData("Right Back: ", myRobot.rb.getCurrentPosition());
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Positive = Clockwise, Negative = Counterclockwise
    void encoderTurn(double targetAngle, double power, double tolerance){
        double currentAngle = getHorizontalAngle();
        double startDifference = targetAngle-currentAngle;
        double currentDifference = startDifference;
        double drivePower = power;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(drivePower, -drivePower);
        while(getHorizontalAngle()<targetAngle-tolerance || getHorizontalAngle()>targetAngle+tolerance /*&& opModeisActive()*/){
            currentAngle = getHorizontalAngle();
            currentDifference = targetAngle-currentAngle;
            drivePower = 0.1 + Math.abs(currentDifference/startDifference)*power*0.9;
            runMotors(drivePower, -drivePower);
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
    void runMotors (double leftPower, double rightPower){
        myRobot.lb.setPower(leftPower);
        myRobot.lf.setPower(leftPower);
        myRobot.rb.setPower(rightPower);
        myRobot.rf.setPower(rightPower);
    }
    double getHorizontalAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.firstAngle;
        if(output>180){
            output-=360;
        }
        if(output<-180){
            output+=360;
        }
        return output;
    }
    double getRoll(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.secondAngle;
        if(output>180){
            output-=360;
        }
        if(output<-180){
            output+=360;
        }
        return output;
    }
    double getVerticalAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.thirdAngle;
        if(output>180){
            output-=360;
        }
        if(output<-180){
            output+=360;
        }
        return output;
    }
    boolean opModeStatus(){
        if(opModeIsActive()){
            return true;
        }
        else return false;
    }
}
