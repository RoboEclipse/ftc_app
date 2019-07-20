package org.firstinspires.ftc.Robot1;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;


interface OpModeCheck
        {
            boolean CheckOpModeIsActive();
        }

public class RoverRuckusClass {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AcszqaP/////AAABmQPh2+SHAkBwsXAKy4LdjLEctzmZIadppxAnjn5ubFiLREbyOyViDtItmB2qAtRfbfJ1GRhhAXPEl992rkY/XW50xxWEVrQ+FGKMC1m6PDC1ropQyBiufMSvx81nz+XF6eSHp6Ct2rrT4YutN9a81bcvGVNA+4EfTu98lzP2HrPUiv0SMlQVq+ze6Fw107r8e7ULdv7dbdfxVtS0X+H4toGS+gxJFyWlgcdHmchQ++I7n8RdaBqoVgItHzjZDDo3lMbPkHIMwsTbWlBYJDBoNGMiFnIBUm1t0J6Yu45dldLZ8eeTPn7R9M9MkwYyWLmAr8Ijs0bxmqDuY4NgvmDqeCIzbfyH7uEximJiwZaGu58u";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private DcMotor lf, lr, rf, rr, leadScrew,exMotor, cflip, emotor;
    private CRServo cServo;
    private DistanceSensor leftDistanceSensor, rightDistanceSensor, elevatorDistanceSensor, extenderDistanceSensor;
    private ColorSensor colorSensor, leftColorSensor, rightColorSensor;
    private Servo elevatorServo, markerServo, led;
    private Telemetry telemetry;
    private BNO055IMU imu;
    private Orientation angles;
    private DigitalChannel elevatorLimitSwitch;
    private static final int ENCODERS_CLOSE_ENOUGH = 10;

    private OpModeCheck opModeCheck;
    double elevatorModifier = 0.4;
    //double dumpingPosition = 0.58;
    double dumpingPosition = 0.187;
    //private static RoverRuckusConfiguration config = new RoverRuckusConfiguration();

    //New collector stuff end
    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        lf = hardwareMap.dcMotor.get(RoverRuckusConfiguration.LeftFrontMotorName);
        lr = hardwareMap.dcMotor.get(RoverRuckusConfiguration.LeftRearMotorName);
        rf = hardwareMap.dcMotor.get(RoverRuckusConfiguration.RightFrontMotorName);
        rr = hardwareMap.dcMotor.get(RoverRuckusConfiguration.RightRearMotorName);
        elevatorServo = hardwareMap.servo.get(RoverRuckusConfiguration.ElevatorServoName);
        markerServo = hardwareMap.servo.get(RoverRuckusConfiguration.TeamMarkerServoName);
        led = hardwareMap.servo.get(RoverRuckusConfiguration.LEDLightName);


        emotor = hardwareMap.dcMotor.get(RoverRuckusConfiguration.ElevatorMotorName);
        exMotor = hardwareMap.dcMotor.get(RoverRuckusConfiguration.ExtenderMotorName);
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, RoverRuckusConfiguration.Left2MeterDistanceSensorName);
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, RoverRuckusConfiguration.Right2MeterDistanceSensorName);
        elevatorDistanceSensor = hardwareMap.get(DistanceSensor.class, RoverRuckusConfiguration.Elevator2MeterDistanceSensorName);
        extenderDistanceSensor = hardwareMap.get(DistanceSensor.class, RoverRuckusConfiguration.Extender2MeterDistanceSensorName);
        imu = hardwareMap.get(BNO055IMU.class, RoverRuckusConfiguration.IMUNAme);
        elevatorLimitSwitch = hardwareMap.digitalChannel.get(RoverRuckusConfiguration.LimitSwitchName);
        leadScrew = hardwareMap.dcMotor.get(RoverRuckusConfiguration.LeadScrewMotorName);
        colorSensor = hardwareMap.colorSensor.get(RoverRuckusConfiguration.ColorSensorName);
        leftColorSensor = hardwareMap.colorSensor.get(RoverRuckusConfiguration.LeftColorSensorName);
        rightColorSensor = hardwareMap.colorSensor.get(RoverRuckusConfiguration.RightColorSensorName);
        cServo = hardwareMap.crservo.get(RoverRuckusConfiguration.cServoName);
        cflip = hardwareMap.dcMotor.get(RoverRuckusConfiguration.CollectionFlipperName);
        lr.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        multiSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, lf, lr, rf, rr);
        emotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        exMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        emotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cflip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters imuSettings = new BNO055IMU.Parameters();
        imuSettings.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuSettings.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuSettings.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuSettings.loggingEnabled      = false;
        imuSettings.loggingTag          = "IMU";
        imuSettings.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuSettings.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuSettings);

    }
    public void newExMotor(double power){
        exMotor.setPower(power);
    }
    public void tankDrive(double leftPower, double rightPower){
        lf.setPower(leftPower);
        lr.setPower(leftPower);
        rr.setPower(rightPower);
        rf.setPower(rightPower);
    }

    public void leadScrewDrive(double power){
        leadScrew.setPower(power);
    }

    public void extendLeadScrew(double runtime){
        ElapsedTime time = new ElapsedTime();
        leadScrew.setPower(1);
        while(elevatorLimitSwitch.getState()){
            if(time.seconds() > runtime){
                break;
            }
        }
        leadScrew.setPower(0);
    }

    public void eMotorDrive(double power)
    {
        emotor.setPower(power);
    }


    public void markerServoDrive(double position){
        markerServo.setPosition(position);
    }
    public void cFlipDrive(double power)
    {
        cflip.setPower(power);
    }
    public void elevatorServoDrive(double position){
        elevatorServo.setPosition(position);
    }
    public boolean isElevatorLimitSwitchNOTPressed(){
        return elevatorLimitSwitch.getState();
    }

    //Encoder stuff
    public void readEncoders(){
        telemetry.addData("Encoders", "lf: " + lf.getCurrentPosition() + " lr: " + lr.getCurrentPosition() + " rf: " +rf.getCurrentPosition() + " rr: "+ rr.getCurrentPosition());
    }
    public int getCFlipEncoder(){
        return cflip.getCurrentPosition();
    }
    public int getElevatorEncoder(){
        return emotor.getCurrentPosition();
    }
    public void resetElevatorEncoder() {
        emotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        emotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetCFlipEncoder(){
        cflip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cflip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void encoderTankDriveInches(double inches, double power) {
        int TICKS_PER_INCH = (int)(1120/(6*Math.PI));
        encoderTankDrive((int)(inches * TICKS_PER_INCH),(int)(inches*TICKS_PER_INCH), power);
    }
    public void gyroTankDriveInches(double inches, double power){
        int TICKS_PER_INCH = (int)(1120/(6*Math.PI));
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(TICKS_PER_INCH*(int)inches);
        rf.setTargetPosition(TICKS_PER_INCH*(int)inches);
        lr.setTargetPosition(TICKS_PER_INCH*(int)inches);
        rr.setTargetPosition(TICKS_PER_INCH*(int)inches);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(power, lf, lr, rf, rr);
        while (anyBusy() && getVerticalAngle()<5) {
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.update();
        }
    }
    public void encoderTankDrive(int leftTicks, int rightTicks, double power) {
        ElapsedTime KILLTIMER = new ElapsedTime();
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(leftTicks);
        rf.setTargetPosition(rightTicks);
        lr.setTargetPosition(leftTicks);
        rr.setTargetPosition(rightTicks);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(power, lf, lr, rf, rr);
        while (anyBusy() && KILLTIMER.seconds()<8) {
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.update();
        }

    }
    public void encoderStrafeDrive(int ticks, double power, String direction) {
        ElapsedTime KILLTIMER = new ElapsedTime();
        int multiplier = -1;
        if (direction.equals("Right") || direction.equals("right")){
            multiplier = 1;
        }
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(multiplier*ticks);
        rf.setTargetPosition(-multiplier*ticks);
        lr.setTargetPosition(-multiplier*ticks);
        rr.setTargetPosition(multiplier*ticks);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(power, lf, lr, rf, rr);
        while (anyBusy() && KILLTIMER.seconds()<8){
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.update();
        }
    }

    public void leftRangeSensorStrafe(int ticks, double distanceCM, double power, String direction){
        ElapsedTime KILLTIMER = new ElapsedTime();
        int multiplier = -1;
        if (direction.equals("Right") || direction.equals("right")){
            multiplier = 1;
        }
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(multiplier*ticks);
        rf.setTargetPosition(-multiplier*ticks);
        lr.setTargetPosition(-multiplier*ticks);
        rr.setTargetPosition(multiplier*ticks);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(power, lf, lr, rf, rr);
        double initialDistance= rightDistanceSensor.getDistance(DistanceUnit.CM)-distanceCM;
        while (anyBusy() && KILLTIMER.seconds()<8) {
            double currentDistanceInCM = leftDistanceSensor.getDistance(DistanceUnit.CM);
            // Option 1
            power*=(currentDistanceInCM-distanceCM)/initialDistance;
            power+=.1;
            if(power>1){
                power=1;
            }
            Log.d("leftRangeSensorStrafe", "Distance "+ currentDistanceInCM + " Goal: " + distanceCM);
            if(currentDistanceInCM < distanceCM){
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                break;
            }
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.addData("distance", currentDistanceInCM);
            telemetry.update();
        }
    }

    public void rightRangeSensorStrafe(int ticks, double distanceCM, double power, String direction){
        ElapsedTime KILLTIMER = new ElapsedTime();
        int multiplier = -1;
        if (direction.equals("Right") || direction.equals("right")){
            multiplier = 1;
        }
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(multiplier*ticks);
        rf.setTargetPosition(-multiplier*ticks);
        lr.setTargetPosition(-multiplier*ticks);
        rr.setTargetPosition(multiplier*ticks);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(power, lf, lr, rf, rr);
        double initialDistance= rightDistanceSensor.getDistance(DistanceUnit.CM)-distanceCM;
        while (anyBusy() && KILLTIMER.seconds()<8) {
            double currentDistanceInCM = rightDistanceSensor.getDistance(DistanceUnit.CM);
            // Option 1
            power*=(currentDistanceInCM-distanceCM)/initialDistance;
            power+=.1;
            if(power>1){
                power=1;
            }

            /*Option 2
            if(distanceCM-currentDistanceInCM<10){
                power=0.3;
            }
            */
            multiSetPower(power, lf, lr, rf, rr);
            Log.d("rightRangeSensorStrafe", "Distance "+ currentDistanceInCM + " Goal: " + distanceCM);
            if(currentDistanceInCM < distanceCM){
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                break;
            }
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.addData("distance", currentDistanceInCM);
            telemetry.update();
        }
    }

    public void colorSensorDrive(int ticks, double power){
        ElapsedTime KILLTIMER = new ElapsedTime();
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(ticks);
        rf.setTargetPosition(ticks);
        lr.setTargetPosition(ticks);
        rr.setTargetPosition(ticks);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(power, lf, lr, rf, rr);
        while (anyBusy() && KILLTIMER.seconds()<8) {
            Log.d("colorSensorDrive: ", "Red: "+colorSensor.red()+ " Blue: " + colorSensor.blue() + " Ticks: " + lf.getCurrentPosition());
            if(colorSensor.red()>90 || colorSensor.blue()>80){
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                Log.d("colorSensorDrive: ", "Detected: Red: "+colorSensor.red()+ " Blue: " + colorSensor.blue() + " Ticks: " + lf.getCurrentPosition());
                telemetry.addData("colorSensorDrive: ", "Success");
                telemetry.update();
                break;
            }
        }
    }
    public void encoderTurn(double degrees, double close, double enuff, double speed){
        //Note: These first two parts are just encoderTankDrive.
        ElapsedTime KILLTIMER = new ElapsedTime();
        double currentDegrees = getHorizontalAngle();
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        if(degrees-currentDegrees>0){
            lf.setTargetPosition(-10000000);
            rf.setTargetPosition(1000000);
            lr.setTargetPosition(-1000000);
            rr.setTargetPosition(1000000);
        }
        if(degrees-currentDegrees<=0){
            lf.setTargetPosition(1000000);
            rf.setTargetPosition(-1000000);
            lr.setTargetPosition(1000000);
            rr.setTargetPosition(-1000000);
        }

        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);

        //This part needs to be different though
        while (anyBusy() && KILLTIMER.seconds()<8) {
            //telemetry.addData("encoderPosition", getEncoderPosition());
            //telemetry.addData("gyroPosition", getHorizontalAngle());

            double adoptivelySlowdownSpeed = speed;
            double angle = getHorizontalAngle();
            if(angle < (degrees + close) && angle > (degrees - close)) {
                adoptivelySlowdownSpeed = 0.1; //tankDrive(0.1, 0.1);
                if (angle < (degrees + enuff) && angle > (degrees - enuff)) {
                    telemetry.update();
                    br8kMotors();
                    multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                    break;
                }
            }
            multiSetPower(adoptivelySlowdownSpeed, lf, lr, rf, rr);
            Log.d("EncoderTurn", Double.toString(angle)+ "target:" + Double.toString(degrees) + "speed" + adoptivelySlowdownSpeed);
            telemetry.update();
        }
    }
    public void elevatorEncoderDriveStart(int ticks, double power){
        emotor.setPower(0);
        emotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        emotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        emotor.setTargetPosition(ticks);
        emotor.setPower(power);
        int position = emotor.getCurrentPosition();
        /*while(position<ticks-10 || position>ticks+10){
            position = emotor.getCurrentPosition();
            telemetry.addData("emotor", position);
            Log.d("emotorrunning", ""+position);
            telemetry.update();
        }
        emotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }
    public void elevatorEncoderDriveEnd(int ticks){
        ElapsedTime killTimer = new ElapsedTime();
        /*emotor.setPower(0);
        emotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        emotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        emotor.setTargetPosition(ticks);
        emotor.setPower(power);*/
        int position = emotor.getCurrentPosition();
        while((position<ticks-10 || position>ticks+10) && killTimer.milliseconds()<1000){
            position = emotor.getCurrentPosition();
            telemetry.addData("emotor", position);
            Log.d("emotorrunning", ""+position);
            telemetry.update();
        }
        emotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void elevatorEncoderDrive(int ticks, double power){
        ElapsedTime killTimer=new ElapsedTime();
        emotor.setPower(0);
        emotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        emotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        emotor.setTargetPosition(ticks);
        emotor.setPower(power);
        int position = emotor.getCurrentPosition();
        while((position<ticks-8 || position>ticks+8) && killTimer.milliseconds()<1500){
            position = emotor.getCurrentPosition();
            telemetry.addData("emotor", position);
            Log.d("emotorrunning", ""+position);
            telemetry.update();
        }
        emotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void driveUntilCraterLeft(double speed /*, double targetDistance*/){
        double startingHorizontalAngle = getHorizontalAngle();
        double startingVerticalAngle = getVerticalAngle();
        double startingThirdAngle = getThirdAngle();
        int multiplier=1;
        if(speed<0){
            multiplier = -1;
        }
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(multiplier*10000000);
        lr.setTargetPosition(multiplier*10000000);
        rf.setTargetPosition(multiplier*10000000);
        rr.setTargetPosition(multiplier*10000000);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(speed, lf, lr, rf, rr);
        while(anyBusy()){
            if(Math.abs(getHorizontalAngle()-startingHorizontalAngle)>5){
                encoderTurn(startingHorizontalAngle, 10,3, 0.1);
            }
            if(     getVerticalAngle()-startingVerticalAngle>2
                    || getThirdAngle()-startingThirdAngle>2){
                telemetry.update();
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                break;
            }
        }
    }
    public void driveUntilCraterRight(double speed/*, double targetDistance*/){
        double startingHorizontalAngle = getHorizontalAngle();
        double startingVerticalAngle = getVerticalAngle();
        double startingThirdAngle = getThirdAngle();
        int multiplier=1;
        if(speed<0){
            multiplier = -1;
        }
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(multiplier*10000000);
        lr.setTargetPosition(multiplier*10000000);
        rf.setTargetPosition(multiplier*10000000);
        rr.setTargetPosition(multiplier*10000000);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(speed, lf, lr, rf, rr);
        while(anyBusy()){
            if(Math.abs(getHorizontalAngle()-startingHorizontalAngle)>5){
                encoderTurn(startingHorizontalAngle, 10,3, 0.1);
            }
            if(     getVerticalAngle()-startingVerticalAngle>2
                    || getThirdAngle()-startingThirdAngle>2){
                telemetry.update();
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                break;
            }
        }
    }

    //Sensor Stuff
    public double getHorizontalAngle () {
        double angleX;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleX = angles.firstAngle;  //ToDo: assume the 3rd angle is the Robot front dir
        //convert the angle to be within the +/-180 degree rangeection
        if (angleX > 180)  angleX -= 360;
        if (angleX <= -180) angleX += 360;
        return (angleX);
    }
    public double getVerticalAngle(){
        double angleX;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleX = angles.secondAngle;  //ToDo: assume the 3rd angle is the Robot front dir
        //convert the angle to be within the +/-180 degree rangeection
        if (angleX > 180)  angleX -= 360;
        if (angleX <= -180) angleX += 360;
        return (angleX);
    }
    public double getThirdAngle(){
        double angleX;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleX = angles.thirdAngle;  //ToDo: assume the 3rd angle is the Robot front dir
        //convert the angle to be within the +/-180 degree rangeection
        if (angleX > 180)  angleX -= 360;
        if (angleX <= -180) angleX += 360;
        return (angleX);
    }
    public double getLeftDistanceSensor(){
        return leftDistanceSensor.getDistance(DistanceUnit.CM);
    }
    public double getRightDistanceSensor(){
        return rightDistanceSensor.getDistance(DistanceUnit.CM);
    }
    public double getElevatorDistanceSensor(){
        return elevatorDistanceSensor.getDistance(DistanceUnit.CM);
    }

    public int getColorSensorRed(){
        return colorSensor.red();
    }
    public int getColorSensorBlue(){
        return colorSensor.blue();
    }
    public double getExtenderDistanceSensor(){
        return extenderDistanceSensor.getDistance(DistanceUnit.CM);
    }
    public int[] getLeftColorSensor(){
        int[] output = new int[3];
        output[0]=leftColorSensor.red();
        output[1]=leftColorSensor.blue();
        output[2]=leftColorSensor.green();
        return output;
    }
    public int[] getRightColorSensor(){
        int[] output = new int[3];
        output[0]=rightColorSensor.red();
        output[1]=rightColorSensor.blue();
        output[2]=rightColorSensor.green();
        return output;
    }
    public boolean isLeftFull(){
        if(leftColorSensor.red()>1000 && leftColorSensor.blue()>1000 && leftColorSensor.green()>1000){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isRightFull(){
        if(rightColorSensor.red()>1000 && rightColorSensor.blue()>1000 && rightColorSensor.green()>1000){
            return true;
        }
        else{
            return false;
        }
    }

    //Space savers
    private void multiSetMode (DcMotor.RunMode mode, DcMotor... ms){
        for(DcMotor m : ms){
            m.setMode(mode);
        }
    }
    private void multiSetPower (double power, DcMotor...motors){
        for(DcMotor m : motors){
            m.setPower(power);
        }
    }

    public void SetOpmodeLiveCheck(OpModeCheck check){
        opModeCheck = check;
    }

    private boolean anyBusy(){
        if(opModeCheck != null && !opModeCheck.CheckOpModeIsActive()){
            // stop running if opmode isn't running
            return false;
        }
        return busy(lf, lr, rf, rr);
    }
    private boolean busy(DcMotor... ms) {
        int total = 0;
        for (DcMotor m : ms) {
            if (m.isBusy()) {
                final int c = Math.abs(m.getCurrentPosition());
                final int t = Math.abs(m.getTargetPosition());
                total += Math.max(0, t - c);
            }
        }
        return total > ENCODERS_CLOSE_ENOUGH;
    }
    public void br8kMotors(){
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }





    //Drive Stuff
    //Preferably Do Not Touch
    public void drive(double direction, double velocity, double rotationVelocity) {
        RoverRuckusClass.Wheels w = getWheels(direction, velocity, rotationVelocity);
        lf.setPower(w.lf);
        rf.setPower(w.rf);
        lr.setPower(w.lr);
        rr.setPower(w.rr);
        telemetry.addData("Powers", String.format(Locale.US, "%.2f %.2f %.2f %.2f", w.lf, w.rf, w.lr, w.rr));
    }

    private static class Wheels {
        double lf, lr, rf, rr;

        Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }
    private RoverRuckusClass.Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        //final double vd = velocity;
        //final double td = direction;
        //final double vt = rotationVelocity;

        double s = Math.sin(direction + Math.PI / 4.0);
        double c = Math.cos(direction + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = velocity * s + rotationVelocity;
        final double v2 = velocity * c - rotationVelocity;
        final double v3 = velocity * c + rotationVelocity;
        final double v4 = velocity * s - rotationVelocity;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);

        return new RoverRuckusClass.Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }
    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    //Tensorflow stuff
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void initTensorFlow (HardwareMap hardwareMap){
        /* Activate Tensor Flow Object Detection. */
        initVuforia(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }
    }
    //TODO: Figure out what's going on here
    public String runTensorFlow () {
        String output = "";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position3", "Left");
                                output = "Left";
                                Log.d("Order: ", "Gold, White, White");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position3", "Right");
                                output = "Right";
                                Log.d("Order: ", "White, White, Gold");
                            } else {
                                telemetry.addData("Gold Mineral Position3", "Center");
                                output = "Center";
                                Log.d("Order: ", "White, Gold, White");
                            }
                    }
                }
                else {
                    int goldMineralX = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        int oneThirdImageSize = recognition.getImageWidth()/3;
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        }
                        if(goldMineralX != -1){
                            if(goldMineralX>2*oneThirdImageSize){
                                output = "Right";
                                telemetry.addData("Gold Mineral Position", "Left");
                                Log.d("Order: ", "Gold, ?, ?");
                            }
                            else if(goldMineralX>oneThirdImageSize){
                                output = "Center";
                                telemetry.addData("Gold Mineral Position", "Center");
                                Log.d("Order: ", "?, Gold, ?");
                            }
                            else{
                                output = "Left";
                                telemetry.addData("Gold Mineral Position", "Right");
                                Log.d("Order: ", "?, ?, Gold");
                            }
                            break;
                        }
                    }
                }
                /*
                if(output == "" && updatedRecognitions.size()==2){

                }
                */
                telemetry.update();
            }
            if(!output.equals("")){
                return output;
            }
            else{
                Log.d("Order: ", "?, ?, ?");
                telemetry.addData("Gold Mineral Position:Assumed", "Left");
                return "Left";
            }

        }
        return "Center";
    }
    public void getImageSize(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() != 0) {
                    //int goldMineralX = -1;
                    //int silverMineral1X = -1;
                    //int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("Width", recognition.getImageWidth());
                        telemetry.addData("Height", recognition.getImageHeight());
                    }
                    /*
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                        }
                    }
                    */
                }
                telemetry.update();
            }
        }
    }
    public void stopTensorFlow(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    ElapsedTime time = new ElapsedTime();
    private double elevatorServoPosition;
    public int autoDump(int stage, boolean fast){
        double retractExtender = -1;
        double raiseCollector = -1;
        double lowerCollector = 0.4;
        //double runCollector = 0.79;
        double raiseElevator = -1;
        double targetDistance = 23;
        double slowDistance = 22;
        double tiltPosition=0.65;
        double rotateCollector = 0.25;
        //double elevatorTargetDistance = 49;
        int TICKS_PER_ROTATION = 1120;
        //Assume collector is down and reset encoders
        if(stage==0){
            time.reset();
            double extenderDistance = getExtenderDistanceSensor();
            cflip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cflip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Log.d("AutoDumpState", "raiseCollector: " + raiseCollector);
            cFlipDrive(raiseCollector);
            if(extenderDistance<slowDistance){
                exMotor.setPower(retractExtender);
                Log.d("AutoDumpState", "retractExtender: " + retractExtender);
            }
            else if(extenderDistance>slowDistance && extenderDistance<targetDistance){
                exMotor.setPower(retractExtender*.8);
                Log.d("AutoDumpState", "retractExtender: " + retractExtender);
            }
            stage++;
        }
        // Lift collector halfway off ground
        else if(stage==1){
            double extenderDistance = getExtenderDistanceSensor();
            int currentPosition = cflip.getCurrentPosition();
            Log.d("AutoDumpState", "Collector Lifting: " + currentPosition);
            if(currentPosition < (-TICKS_PER_ROTATION * rotateCollector)){
                cFlipDrive(0);
                stage++;
            }
            if(extenderDistance>=targetDistance || time.milliseconds()>500){
                exMotor.setPower(-0.15);
                Log.d("AutoDumpState", "Extender Retracted");
            }
        }
        // Retract the extender
        else if(stage==2){
            double extenderDistance = getExtenderDistanceSensor();
            Log.d("AutoDumpState", "Extender Distance: " + extenderDistance);
            if(extenderDistance>=targetDistance || time.milliseconds()>1000){
                cFlipDrive(raiseCollector);
                exMotor.setPower(-0.15);
                stage++;
                Log.d("AutoDumpState", "Extender Retracted");
            }
        }
        // Retract flipper to dump minerals into basket
        else if(stage==3){
            ElapsedTime time = new ElapsedTime();
            int currentPosition = cflip.getCurrentPosition();
            Log.d("AutoDumpState", "Collector Retracted: " + currentPosition);
            telemetry.addData("CurrentPosition", currentPosition);
            if(currentPosition<-TICKS_PER_ROTATION*1.0 || time.milliseconds()>500){
                cFlipDrive(0);
                exMotor.setPower(0);
                stage++;
                //Reset the timer
                time.reset();
                //cServo.setPower(runCollector);
            }
        }
        //Rotate collector until timer reaches 200 milliseconds
        else if(stage == 4){
            /*
            if(time.milliseconds() > 800 || fast){
                cServo.setPower(0);
                Log.d("AutoDumpState", "Rotated");
                stage ++;
            }
            */

            if(time.milliseconds()>30){
                stage++;
            }
        }
        //Lower down collector to get ready
        else if(stage == 5){
            int currentPosition = cflip.getCurrentPosition();
            cFlipDrive(lowerCollector);
            Log.d("AutoDumpState", "LoweringCollector: " + currentPosition);
            if(currentPosition>-7*TICKS_PER_ROTATION/12){
                Log.d("AutoDumpState", "Lowered");
                cFlipDrive(-0.05);
                //eMotorDrive(raiseElevator);
                time.reset();
                stage++;
            }
        }
        //Raise up basket and reset
        else if (stage == 6){
            stage = 0;
            double elevatorDistance = getElevatorDistanceSensor();
            Log.d("AutoDumpState", "Raising Elevator: " + elevatorDistance);
            if(elevatorDistance > 49 && elevatorDistance < 800){
                Log.d("AutoDumpState", "Initial Elevator Servo");
                elevatorServoPosition = tiltPosition;
                elevatorServoDrive(elevatorServoPosition);
                eMotorDrive(raiseElevator/2);
                //stage++;
            }
            if (time.milliseconds()>3000){
                stage = 0;
                eMotorDrive(0);
                Log.d("AutoDumpState", "AutoStop");
            }
        }
        else if(stage==7){
            double elevatorDistance = getElevatorDistanceSensor();
            if(elevatorDistance>50) {
                if(fast && elevatorServoPosition==0.45){
                    elevatorServoPosition = 0.45;
                }
                else{
                    elevatorServoPosition = 0.7;
                }
                Log.d("State", "Raised");
                eMotorDrive(0);
                stage++;
            }
            if(time.milliseconds()>3000){
                stage = 0;
                eMotorDrive(0);
                Log.d("AutoDumpState", "AutoStop");
            }
        }
        else if(stage==8){
            stage = 0;
        }
        telemetry.addData("Stage", stage);
        return stage;
    }

    public void cServoDrive(double power){
        cServo.setPower(power);
    }
    public int getExtenderEncoder(){
        return exMotor.getCurrentPosition();
    }
    public void resetExtenderEncoder(){
        exMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        exMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean autoRetract(double time){
        if(time<1000){
            elevatorServo.setPosition(0.59);
            return true;
        }
        else{
            elevatorServo.setPosition(1);
            if(elevatorDistanceSensor.getDistance(DistanceUnit.CM)>7 && elevatorDistanceSensor.getDistance(DistanceUnit.CM)<800){
                eMotorDrive(1);
                return true;
            }
            else{
                eMotorDrive(0);
                return false;
            }
        }
    }
    public void ledControl(double input){
        led.setPosition(input);
    }
    public void removeBrake(){
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void addBrake(){
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void teleOpSleep(long time){
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
//Archive
/*
    public void allDrive(double lfPower, double lrPower, double rfPower, double rrPower){
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setPower(lfPower);
        lr.setPower(lrPower);
        rf.setPower(rfPower);
        rr.setPower(rrPower);
    }
        public void exServoDrive(double power){

        if(power>0.51){
            exMotor.setPower(.79);
            //exservoback.setPower(.02);
        }
        else if(power<0.49){
            //exservoback.setPower(-.79);
            //exservo.setPower(-.02);
            exMotor.setPower(-.79);
        }
        else{
            exMotor.setPower(0);
        }
        /*
        else{
            exservo.setPower(0);
            exservoback.setPower(0);
        }
            public void singleDrive(double power, DcMotor motor){
        motor.setPower(power);
    }
    public void strafeDrive(double power){
        lf.setPower(power);
        lr.setPower(-power);
        rf.setPower(-power);
        rr.setPower(power);
    }
    public boolean isIMUCalibrated(){
        return imu.isGyroCalibrated();
    }
    public String getEncoderPosition()
    {
        return String.format(Locale.US, "lf: %d,rf: %d, lr:%d, rr:%d",
                lf.getCurrentPosition(),
                rf.getCurrentPosition(),
                lr.getCurrentPosition(),
                rr.getCurrentPosition()
        );
    }
        public boolean isElevatorLimitSwitchIsNOTPressed(){
        if(extenderDistanceSensor.getDistance(DistanceUnit.CM) > 0.5) {
            return true;
        }
        return false;
    }

    public boolean isExtenderLimitSwitchNOTPressed() {
        if(!(elevatorDistanceSensor.getDistance(DistanceUnit.CM) <= 0.5)) {
            return true;
        }
        return false;
    }
        public double autoDeposit(double time){
        if(time<700){
            return 0.66;
        }
        else{
            return 0.59;
        }
    }
*/
