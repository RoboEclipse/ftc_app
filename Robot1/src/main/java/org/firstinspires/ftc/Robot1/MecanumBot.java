package org.firstinspires.ftc.Robot1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.Locale;

class MecanumBot {
    private DcMotor lf, lr, rf, rr, armMotor, slideMotor;
    private Telemetry telemetry;
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private ColorSensor jewelColorSensor, bottomColorSensor;
    private Servo jewelServo, sidebarleft, sidebarright ,flicker;

    private static final double TICKS_PER_INCH = 1120 * (16./24.) / (Math.PI * 4.0);
    private static final double TICKS_PER_CM = TICKS_PER_INCH / 2.54;
    private static final double ENCODER_DRIVE_POWER = 0.3;

    private double encoder_drive_power = ENCODER_DRIVE_POWER;

    RobotConfiguration myRobotConfig = new RobotConfiguration();

    public void initMecanumBot(HardwareMap hardwareMap, Telemetry _telemetry) {
        telemetry = _telemetry;
        lf = hardwareMap.dcMotor.get(myRobotConfig.LeftFrontMotorName);
        lr = hardwareMap.dcMotor.get(myRobotConfig.LeftRearMotorName);
        rf = hardwareMap.dcMotor.get(myRobotConfig.RightFrontMotorName);
        rr = hardwareMap.dcMotor.get(myRobotConfig.RightRearMotorName);
        armMotor = hardwareMap.dcMotor.get(myRobotConfig.ArmDriveMotorName);
        slideMotor = hardwareMap.dcMotor.get(myRobotConfig.LinearSlideMotorName);
        jewelColorSensor = hardwareMap.get(ColorSensor.class, (RobotConfiguration.JewelColorSensorName));
        //bottomColorSensor = hardwareMap.colorSensor.get(myRobotConfig.BottomColorSensorName);
        imu = hardwareMap.get(BNO055IMU.class, myRobotConfig.IMUName);
        sidebarleft = hardwareMap.servo.get(myRobotConfig.SideBarLeftName);
        sidebarright = hardwareMap.servo.get(myRobotConfig.SideBarRightName);
        jewelServo = hardwareMap.servo.get(myRobotConfig.JewelServoName);
        flicker = hardwareMap.servo.get(myRobotConfig.JewelServo2Name);

        resetDirection();

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // These are the wheel motors
        // I am here! LOLOLOLOLOLOLOLOL
        //Catmeow? :3
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(
                new org.firstinspires.ftc.robotcore.external.navigation.Position(),
                new Velocity(),
                1000);

        jewelColorSensor.setI2cAddress(I2cAddr.create8bit(0x44));
        //bottomColorSensor.setI2cAddress(I2cAddr.create8bit(0x42));

    }

    private void resetDirection() {
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getAngle () {
        double angleX;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleX = angles.firstAngle;  //ToDo: assume the 3rd angle is the Robot front dir
        //convert the angle to be within the +/-180 degree rangeection
        if (angleX > 180)  angleX -= 360;
        if (angleX <= -180) angleX += 360;
        return (angleX);
        }

    public double getGravity (){
        gravity=imu.getGravity();
        return(gravity.zAccel);
        }

    public void controlArm (double armPower) {
        armMotor.setPower (armPower);
    }

    public void controlSlide ( double slidePower) { slideMotor.setPower (slidePower) ; }

    public int[] readJewelColor() {
        int[] rgb = {jewelColorSensor.red(), jewelColorSensor.green(),jewelColorSensor.blue()};
        return (rgb);
    }

    public int[] readFloorColor() {
        int[] rgb = {bottomColorSensor.red(), bottomColorSensor.green(),bottomColorSensor.blue()};
        return (rgb);
    }

    public void moveJewelServo (double position) {
        jewelServo.setPosition(position);
    }

    public void moveSideBar ( double position) {
        sidebarleft.setPosition(position);
        sidebarright.setPosition(1 - position);
    }

    public void onStart() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rr, rf);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rr, rf);
    }

    public void encoderDriveTiles(double direction, double tiles) {
        encoderDriveInches(direction, 24.0 * tiles);
    }

    void setEncoderDrivePower(double p) {
        encoder_drive_power = p;
    }

    void clearEncoderDrivePower() {
        encoder_drive_power = 0.0;
    }

    private double total_light_meter_reading = 0.0;
    private int light_meter_readings_tooken = 0;
    private boolean collect_light_meter_info = false;

    public void startCollectingLightMeter() {
        collect_light_meter_info = true;
        total_light_meter_reading = 0.0;
        light_meter_readings_tooken = 0;
    }

    // Maximum absolute value of some number of arguments
    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
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

    private Wheels getWheels(double direction, double velocity, double rotationVelocity) {
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

        return new Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }
    public void drive(double direction, double velocity, double rotationVelocity) {
         Wheels w = getWheels(direction, velocity, rotationVelocity);
        lf.setPower(w.lf);
        rf.setPower(w.rf);
        lr.setPower(w.lr);
        rr.setPower(w.rr);
        telemetry.addData("Powers", String.format(Locale.US, "%.2f %.2f %.2f %.2f", w.lf, w.rf, w.lr, w.rr));
    }

    // Shut down all motors
    public void stopDriveMotors() {
        lf.setPower(0.0);
        lr.setPower(0.0);
        rf.setPower(0.0);
        rr.setPower(0.0);
    }

    private void setMode(DcMotor.RunMode mode, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setMode(mode);
        }
    }

    private void setPower(double p, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setPower(p);
        }
    }

    private void setTargetPosition(int pos, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setTargetPosition(pos);
        }
    }

    public static final int ENCODERS_CLOSE_ENOUGH = 10;

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

    public boolean driveMotorsBusy() {
        return busy(lf, lr, rf, rr);

    }

    public void encoderDriveInches(double direction, double inches) {
        final Wheels w = getWheels(direction, 1.0, 0.0);
        final int ticks = (int)(inches * TICKS_PER_INCH);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    public void encoderDriveCM(double direction, double cm) {
        direction %= Math.PI * 2.0;
        final Wheels w = getWheels(direction, 1, 0);
        final int ticks = (int)(cm * TICKS_PER_CM);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    public void encoderDriveCM(double direction, double cm, double turningSpeed) {
        direction %= Math.PI * 2.0;
        final Wheels w = getWheels(direction, 0, turningSpeed);
        final int ticks = (int)(cm * TICKS_PER_CM);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    public void encoderDrive(double lft, double rft, double lrt, double rrt) {
        encoderDrive((int) lft, (int) rft, (int) lrt, (int) rrt);
    }

    private static int SLOW_DOWN_HERE = 1120;
    private static double ARBITRARY_SLOW_SPEED = .3;
    private boolean slowedDown = false;

    private int averageRemainingTicks(DcMotor... ms) {
        int total = 0;
        int count = 0;
        for (DcMotor m : ms) {
            if (m.getMode() == DcMotor.RunMode.RUN_TO_POSITION && 100 < Math.abs(m.getTargetPosition())) {
                total += Math.abs(m.getTargetPosition() - m.getCurrentPosition());
                count += 1;
            }
        }
        return 0 == count ? 0 : total / count;
    }

    private void encoderDriveSlowdown() {
        if (! slowedDown) {
            if (lf.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                int remaining = averageRemainingTicks(lf, lr, rf, rr);
                if (remaining < SLOW_DOWN_HERE) {
                    slowedDown = true;
                    setPower(ARBITRARY_SLOW_SPEED, lf, lr, rf, rr);
                }
            }
        }
    }

    private void encoderDrive(int lft, int rft, int lrt, int rrt) {
        setPower(0.0, lf, lr, rf, rr);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        setTargetPosition(lft, lf);
        setTargetPosition(rft, rf);
        setTargetPosition(lrt, lr);
        setTargetPosition(rrt, rr);
        setMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        setPower(encoder_drive_power, lf, lr, rf, rr);
        slowedDown = false;
    }
    public void encoderTankDrive(int leftTicks, int rightTicks, double power) {
        setPower(0.0, lf, lr, rf, rr);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        setTargetPosition(leftTicks, lf);
        setTargetPosition(rightTicks, rf);
        setTargetPosition(leftTicks, lr);
        setTargetPosition(rightTicks, rr);
        setMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        setPower(power, lf, lr, rf, rr);
        slowedDown = false;
        while (driveMotorsBusy()) {
            telemetry.addData("encoderPosition", getEncoderPosition());
            telemetry.addData("gyroPosition", getAngle());
            telemetry.update();
        }
    }
    public void encoderTurn(double degrees, double close, double enuff, double speed){
        //Note: These first two parts are just encoderTankDrive.
        if(degrees>0){
            setPower(0.0, lf, lr, rf, rr);
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
            setTargetPosition(-10000000, lf);
            setTargetPosition(1000000, rf);
            setTargetPosition(-1000000, lr);
            setTargetPosition(1000000, rr);
            setMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
            setPower(speed, lf, lr, rf, rr);
        }
        if(degrees<0){
            setPower(0.0, lf, lr, rf, rr);
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
            setTargetPosition(1000000, lf);
            setTargetPosition(-1000000, rf);
            setTargetPosition(1000000, lr);
            setTargetPosition(-1000000, rr);
            setMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
            setPower(speed, lf, lr, rf, rr);
        }
        //This part needs to be different though
        while (driveMotorsBusy()) {
            telemetry.addData("encoderPosition", getEncoderPosition());
            telemetry.addData("gyroPosition", getAngle());

            if(getAngle()<degrees+close && getAngle()>degrees-close) {
                tankDrive(0.1, 0.1);
                if (getAngle() < degrees + enuff && getAngle() > degrees - enuff) {
                    telemetry.update();
                    break;
                }
            }
            telemetry.update();
        }
    }


    // All motors start out at ENCODER_DRIVE_POWER power. Once we get one revolution
    // in, go ahead and speed up. When we get within a revolution of the end of our
    // run, start slowing down. The idea here is to avoid slip.
    private static final int ACCEL_THRESHOLD = 1120 * 24 / 18; // one wheel revolution, for starters
    private boolean atSteadyState = false;
    private void manageEncoderAccelleration(DcMotor... ms) {
        if (encoder_drive_power > ENCODER_DRIVE_POWER) {
            int current = 0, remaining = 0, count = 0;

            ArrayList<DcMotor> driving = new ArrayList<>();
            for (DcMotor m : ms) {
                if (m.getMode() == DcMotor.RunMode.RUN_TO_POSITION && 0 != m.getTargetPosition()) {
                    driving.add(m);
                    current += m.getCurrentPosition();
                    remaining += Math.abs(m.getCurrentPosition() - m.getTargetPosition());
                    count++;
                }
            }

            if (0 < driving.size()) {
                current /= count;
                remaining /= count;

                double power = encoder_drive_power;
                double dp = encoder_drive_power - ENCODER_DRIVE_POWER;

                if (remaining < ACCEL_THRESHOLD) {
                    atSteadyState = false;
                    power = ENCODER_DRIVE_POWER + dp * ((double)remaining / (double)ACCEL_THRESHOLD);
                    for (DcMotor m : driving) {
                        m.setPower(power);
                    }
                } else if (current < ACCEL_THRESHOLD) {
                    atSteadyState = false;
                    power = ENCODER_DRIVE_POWER + dp * ((double)current / (double)ACCEL_THRESHOLD);
                    for (DcMotor m : driving) {
                        m.setPower(power);
                    }
                }  else {
                    if (! atSteadyState) {
                        for (DcMotor m : driving) {
                            m.setPower(power);
                        }
                        atSteadyState = true;
                    }
                }

            }
        }
    }

    public void resetDriveMotorModes() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
    }

    public void disableEncoders() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, lf, lr, rf, rr);
    }
    public int GetJewelSensorBlue(){
        return jewelColorSensor.blue();
    }
    public int GetJewelSensorRed(){
            return jewelColorSensor.red();
        }
    public void setJewelArm(double jewelPosition){
        jewelServo.setPosition(jewelPosition);
    }
    public void tankDrive(double leftpower, double rightpower) {
        lf.setPower(leftpower);
        lr.setPower(leftpower);
        rf.setPower(rightpower);
        rr.setPower(rightpower);
        telemetry.addData("Powers:", "Left: "+leftpower,"Right: "+rightpower);
    }
    // Things that need to happen in the teleop loop to accommodate long-running
    // tasks like running the flipper one at a time.
    public void loop() {

        encoderDriveSlowdown();
        manageEncoderAccelleration(lf, lr, rf, rr);
    }

    public String getEncoderPosition()
    {
        return String.format("lf: %d,rf: %d, lr:%d, rr:%d",
                lf.getCurrentPosition(),
                rf.getCurrentPosition(),
                lr.getCurrentPosition(),
                rr.getCurrentPosition());
    }
    public void extenderDrive (double power){
        slideMotor.setPower(power);
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
    public void holdArm(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void flick (double direction){
        flicker.setPosition(direction);
    }
}

