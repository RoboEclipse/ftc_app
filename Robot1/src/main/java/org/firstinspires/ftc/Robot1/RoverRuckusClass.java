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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;
import java.util.Locale;

public class RoverRuckusClass {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AcszqaP/////AAABmQPh2+SHAkBwsXAKy4LdjLEctzmZIadppxAnjn5ubFiLREbyOyViDtItmB2qAtRfbfJ1GRhhAXPEl992rkY/XW50xxWEVrQ+FGKMC1m6PDC1ropQyBiufMSvx81nz+XF6eSHp6Ct2rrT4YutN9a81bcvGVNA+4EfTu98lzP2HrPUiv0SMlQVq+ze6Fw107r8e7ULdv7dbdfxVtS0X+H4toGS+gxJFyWlgcdHmchQ++I7n8RdaBqoVgItHzjZDDo3lMbPkHIMwsTbWlBYJDBoNGMiFnIBUm1t0J6Yu45dldLZ8eeTPn7R9M9MkwYyWLmAr8Ijs0bxmqDuY4NgvmDqeCIzbfyH7uEximJiwZaGu58u";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private DcMotor lf, lr, rf, rr, leadScrew, cmotor, cflip, emotor;
    private CRServo exservo, exservoback;
    private DistanceSensor leftDistanceSensor, rightDistanceSensor, elevatorDistanceSensor;
    private ColorSensor colorSensor;
    private Servo elevatorServo, markerServo;
    private Telemetry telemetry;
    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    private BNO055IMU imu;
    private Orientation angles;
    private DigitalChannel limitSwitch;
    public static final int ENCODERS_CLOSE_ENOUGH = 10;
    int TICKS_PER_ROTATION = 1120;
    public int TICKS_PER_INCH = (int)(1120/(6*Math.PI));
    public int TICKS_PER_CENTIMETER =(int)(TICKS_PER_INCH*2.54);
    public int leadScrewTime=5000;
    RoverRuckusConfiguration config = new RoverRuckusConfiguration();

    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        lf = hardwareMap.dcMotor.get(config.LeftFrontMotorName);
        lr = hardwareMap.dcMotor.get(config.LeftRearMotorName);
        rf = hardwareMap.dcMotor.get(config.RightFrontMotorName);
        rr = hardwareMap.dcMotor.get(config.RightRearMotorName);
        elevatorServo = hardwareMap.servo.get(config.ElevatorServoName);
        markerServo = hardwareMap.servo.get(config.TeamMarkerServoName);
        cmotor = hardwareMap.dcMotor.get(config.CollectorMotorName);
        emotor = hardwareMap.dcMotor.get(config.ElevatorMotorName);
        exservo = hardwareMap.crservo.get(config.ExtenderMotorName);
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, config.Left2MeterDistanceSensorName);
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, config.Right2MeterDistanceSensorName);
        elevatorDistanceSensor = hardwareMap.get(DistanceSensor.class, config.Elevator2MeterDistanceSensorName);
        exservoback = hardwareMap.crservo.get(config.ExtenderBackMotorName);
        cflip = hardwareMap.dcMotor.get(config.CollectionFlipperName);
        imu = hardwareMap.get(BNO055IMU.class, config.IMUNAme);
        limitSwitch = hardwareMap.digitalChannel.get(config.LimitSwitchName);
        //touchSensor = hardwareMap.get(DigitalChannel.class, config.TouchSensor);
        leadScrew = hardwareMap.dcMotor.get(config.LeadScrewMotorName);
        colorSensor = hardwareMap.colorSensor.get(config.ColorSensorName);
        multiSetMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
        lr.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        emotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cflip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters imuSettings = new BNO055IMU.Parameters();;
        imuSettings.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuSettings.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuSettings.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuSettings.loggingEnabled      = false;
        imuSettings.loggingTag          = "IMU";
        imuSettings.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(imuSettings);
        /*
        imu.startAccelerationIntegration(
                new Position(),
                new Velocity(),
                1000);
        */
        imuSettings.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuSettings);

    }
    public void tankDrive(double leftPower, double rightPower){
        lf.setPower(leftPower);
        lr.setPower(leftPower);
        rr.setPower(rightPower);
        rf.setPower(rightPower);
    }
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
    public void cMotorDrive(double power)
    {
        cmotor.setPower(power);
    }

    public void leadScrewDrive(double power){
        leadScrew.setPower(power);
    }
    public void extendLeadScrew(double runtime){
        ElapsedTime time = new ElapsedTime();
        leadScrew.setPower(1);
        while(limitSwitch.getState()==true){
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
    public void exServoDrive(double power){

        if(power>0.51){
            exservo.setPower(.79);
            exservoback.setPower(.02);
        }
        else if(power<0.49){
            exservoback.setPower(-.79);
            exservo.setPower(-.02);
        }
        else{
            exservo.setPower(0);
            exservoback.setPower(0);
        }

    }
    public void markerServoDrive(double position){
        markerServo.setPosition(position);
    }
    public void cFlipDrive(double power)
    {
        cflip.setPower(power);
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
    public void elevatorServoDrive(double position){
        elevatorServo.setPosition(position);
    }
    public boolean returnLimitSwitch(){
        return limitSwitch.getState();
    }


    //Encoder stuff
    public void readEncoders(){
        telemetry.addData("Encoders", "lf: " + lf.getCurrentPosition() + " lr: " + lr.getCurrentPosition() + " rf: " +rf.getCurrentPosition() + " rr: "+ rr.getCurrentPosition());
    }
    public void encoderTankDrive(int leftTicks, int rightTicks, double power) {
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(leftTicks);
        rf.setTargetPosition(rightTicks);
        lr.setTargetPosition(leftTicks);
        rr.setTargetPosition(rightTicks);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(power, lf, lr, rf, rr);
        while (anyBusy()) {
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.update();
        }

    }
    public void encoderStrafeDrive(int ticks, double power, String direction) {
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
        while (anyBusy()) {
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.update();
        }
    }

    public void leftRangeSensorStrafe(int ticks, double distanceCM, double power, String direction){
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
        while (anyBusy()) {
            if(leftDistanceSensor.getDistance(DistanceUnit.CM)<distanceCM){
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                break;
            }
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.addData("distance", leftDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    public void rightRangeSensorStrafe(int ticks, double distanceCM, double power, String direction){
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
        while (anyBusy()) {
            if(rightDistanceSensor.getDistance(DistanceUnit.CM)<distanceCM){
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                break;
            }
            readEncoders();
            telemetry.addData("gyroPosition", getHorizontalAngle());
            telemetry.addData("distance", rightDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
    public void colorSensorDrive(int ticks, double power){
        multiSetPower(0.0, lf, lr, rf, rr);
        multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        lf.setTargetPosition(ticks);
        rf.setTargetPosition(ticks);
        lr.setTargetPosition(ticks);
        rr.setTargetPosition(ticks);
        multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        multiSetPower(power, lf, lr, rf, rr);
        while (anyBusy()) {
            Log.d("colorSensorDrive: ", "Red: "+colorSensor.red()+ " Blue: " + colorSensor.blue() + " Ticks: " + lf.getCurrentPosition());
            if(colorSensor.red()>100 || colorSensor.blue()>100){
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                Log.d("colorSensorDrive: ", "Detected: Red: "+colorSensor.red()+ " Blue: " + colorSensor.blue() + " Ticks: " + lf.getCurrentPosition());
                break;
            }
        }
    }
    public void encoderTurn(double degrees, double close, double enuff, double speed){
        //Note: These first two parts are just encoderTankDrive.
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
        while (anyBusy()) {
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
            Log.d("EncoderTurn", Double.toString(angle) + "speed" + adoptivelySlowdownSpeed);
            telemetry.update();
        }
    }

    public void driveUntilCraterLeft(double speed, double targetDistance){
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
            /*
            if(leftDistanceSensor.getDistance(DistanceUnit.CM)>targetDistance+5){
                encoderStrafeDrive((int)(leftDistanceSensor.getDistance(DistanceUnit.CM)-targetDistance), 0.5, "Left");

            }
            if(leftDistanceSensor.getDistance(DistanceUnit.CM)<targetDistance-5){
                encoderStrafeDrive((int)(targetDistance-leftDistanceSensor.getDistance(DistanceUnit.CM)), 0.5, "Right");
            }
            */
            if(     getVerticalAngle()-startingVerticalAngle>2
                    || getThirdAngle()-startingThirdAngle>2){
                telemetry.update();
                br8kMotors();
                multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                break;
            }
        }
    }
    public void driveUntilCraterRight(double speed, double targetDistance){
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
            /*
            if(rightDistanceSensor.getDistance(DistanceUnit.CM)>targetDistance+5){
                encoderStrafeDrive(5*TICKS_PER_CENTIMETER, 0.5, "Left");

            }
            if(rightDistanceSensor.getDistance(DistanceUnit.CM)<targetDistance-5){
                encoderStrafeDrive(5*TICKS_PER_CENTIMETER, 0.5, "Right");
            }
            */
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
    public boolean isIMUCalibrated(){
        return imu.isGyroCalibrated();
    }
    public int getColorSensorRed(){
        return colorSensor.red();
    }
    public int getColorSensorBlue(){
        return colorSensor.blue();
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
    private boolean anyBusy(){
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
    public String getEncoderPosition()
    {
        return String.format("lf: %d,rf: %d, lr:%d, rr:%d",
                lf.getCurrentPosition(),
                rf.getCurrentPosition(),
                lr.getCurrentPosition(),
                rr.getCurrentPosition()
        );
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
        public double lf, lr, rf, rr;

        public Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }
    private RoverRuckusClass.Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        final double vd = velocity;
        final double td = direction;
        final double vt = rotationVelocity;

        double s = Math.sin(td + Math.PI / 4.0);
        double c = Math.cos(td + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = vd * s + vt;
        final double v2 = vd * c - vt;
        final double v3 = vd * c + vt;
        final double v4 = vd * s - vt;

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
    public void initVuforia(HardwareMap hardwareMap) {
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
    public void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void initTensorFlow (HardwareMap hardwareMap){
        /** Activate Tensor Flow Object Detection. */
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
                            telemetry.addData("Gold Mineral Position", "Left");
                            output = "Left";
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            output = "Right";
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            output = "Center";
                        }
                    }
                }
                telemetry.update();
            }
            if(output != ""){
                return output;
            }
            else{
                return "Center";
            }

        }
        return "Center";
    }
    public void stopTensorFlow(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
