package org.firstinspires.ftc.Robot1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class RoverRuckusClass {
    private DcMotor lf, lr, rf, rr, leadScrew, cmotor, cflip, emotor;
    private CRServo exservo;
    private Servo elevatorServo, markerServo;
    private Telemetry telemetry;
    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    private BNO055IMU imu;
    private Orientation angles;
    private DigitalChannel limitSwitch;
    public static final int ENCODERS_CLOSE_ENOUGH = 10;
    int TICKS_PER_ROTATION = 1120;
    public int TICKS_PER_INCH = (int)(1120/(6*Math.PI));
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
        cflip = hardwareMap.dcMotor.get(config.CollectionFlipperName);
        imu = hardwareMap.get(BNO055IMU.class, config.IMUNAme);
        limitSwitch = hardwareMap.digitalChannel.get(config.LimitSwitchName);
        //touchSensor = hardwareMap.get(DigitalChannel.class, config.TouchSensor);
        leadScrew = hardwareMap.dcMotor.get(config.LeadScrewMotorName);
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

        BNO055IMU.Parameters imuSettings = new BNO055IMU.Parameters();;
        imuSettings.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuSettings.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuSettings.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuSettings.loggingEnabled      = true;
        imuSettings.loggingTag          = "IMU";
        imuSettings.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(imuSettings);
        imu.startAccelerationIntegration(
                new Position(),
                new Velocity(),
                1000);

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
    public void eMotorDrive(double power)
    {
        emotor.setPower(power);
    }
    public void exServoDrive(double power){
        exservo.setPower(power);
        //power = 0.5+0.25*power;
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
    public void encoderTurn(double degrees, double close, double enuff, double speed){
        //Note: These first two parts are just encoderTankDrive.

        if(degrees>0){
            multiSetPower(0.0, lf, lr, rf, rr);
            multiSetMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
            lf.setTargetPosition(-10000000);
            rf.setTargetPosition(1000000);
            lr.setTargetPosition(-1000000);
            rr.setTargetPosition(1000000);
            multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
            multiSetPower(speed, lf, lr, rf, rr);
        }
        if(degrees<=0){
            multiSetPower(0.0, lf, lr, rf, rr);
            multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
            lf.setTargetPosition(1000000);
            rf.setTargetPosition(-1000000);
            lr.setTargetPosition(1000000);
            rr.setTargetPosition(-1000000);
            multiSetMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
            multiSetPower(speed, lf, lr, rf, rr);
        }
        //This part needs to be different though
        while (anyBusy()) {
            telemetry.addData("encoderPosition", getEncoderPosition());
            telemetry.addData("gyroPosition", getHorizontalAngle());

            if(getHorizontalAngle()<degrees+close && getHorizontalAngle()>degrees-close) {
                tankDrive(0.1, 0.1);
                if (getHorizontalAngle() < degrees + enuff && getHorizontalAngle() > degrees - enuff) {
                    telemetry.update();
                    br8kMotors();
                    multiSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
                    break;
                }
            }
            telemetry.update();
        }
    }
    public void driveUntilCrater(double speed){
        double startingAngle = getHorizontalAngle();
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
            if(Math.abs(getHorizontalAngle()-startingAngle)>10){
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
}
