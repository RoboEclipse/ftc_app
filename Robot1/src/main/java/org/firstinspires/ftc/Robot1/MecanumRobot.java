package org.firstinspires.ftc.Robot1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;
import java.util.Locale;

class MecanumRobot {
    private DcMotor lf, lr, rf, rr, ad, ls;

    private NormalizedColorSensor jewelsensor, bottomsensor;
    private Gyroscope gyrosensor;
    private Telemetry telemetry;
    private VuMark vuMark;


    public MecanumRobot(HardwareMap hardwareMap, Telemetry _telemetry) {
        telemetry = _telemetry;
        lf = hardwareMap.dcMotor.get(RobotConfiguration.LeftFrontMotorName);
        lr = hardwareMap.dcMotor.get(RobotConfiguration.LeftRearMotorName);
        rf = hardwareMap.dcMotor.get(RobotConfiguration.RightFrontMotorName);
        rr = hardwareMap.dcMotor.get(RobotConfiguration.RightRearMotorName);
        ad = hardwareMap.dcMotor.get(RobotConfiguration.ArmDriveMotorName);
        ls = hardwareMap.dcMotor.get(RobotConfiguration.LinearSlideMotorName);

        jewelsensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfiguration.JewelColorSensorName);
        bottomsensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfiguration.BottomColorSensorName);
        gyrosensor = hardwareMap.get(Gyroscope.class, RobotConfiguration.GyroSensorName);

        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vuMark = new VuMark(_telemetry, hardwareMap);
    }

    public AngularVelocity getGyrosensorValue()
    {
        return gyrosensor.getAngularVelocity(AngleUnit.DEGREES);
    }

    public void onStart() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rr, rf);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rr, rf);
        vuMark.onInit(VuforiaLocalizer.CameraDirection.BACK);
    }

    public void onStop() {
        stopDriveMotors();
    }

    private interface Stoppable {
        public boolean stopped();
    }


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

    private static int SLOW_DOWN_HERE = 1120;
    private static double ARBITRARY_SLOW_SPEED = .3;
    private boolean slowedDown = false;
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

    // Things that need to happen in the teleop loop to accommodate long-running
    // tasks like running the flipper one at a time.
    public void loop() {

        encoderDriveSlowdown();
        manageEncoderAccelleration(lf, lr, rf, rr);
        vuMark.onLoop();
    }


    public void updateSensorTelemetry() {

        telemetry.addData("Encoder Remain", averageRemainingTicks(lf, lr, rf, rr));
        telemetry.addData("EncodersC", String.format(Locale.US, "%d\t%d\t%d\t%d\t%d",
                lf.getCurrentPosition(),
                lr.getCurrentPosition(),
                rf.getCurrentPosition(),
                rr.getCurrentPosition()));
        telemetry.addData("EncodersT", String.format(Locale.US, "%d\t%d\t%d\t%d\t%d",
                lf.getTargetPosition(),
                lr.getTargetPosition(),
                rf.getTargetPosition(),
                rr.getTargetPosition()));
    }

    /// Maximum absolute value of some number of arguments
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

        double s =  Math.sin(td + Math.PI / 4.0);
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

    /// Shut down all motors
    public void stopDriveMotors() {
        lf.setPower(0.0);
        lr.setPower(0.0);

        rf.setPower(0.0);
        rr.setPower(0.0);
    }

    // Encoder Driving

    // Assuming 4" wheels
    private static final double TICKS_PER_INCH = 1120 * (16./24.) / (Math.PI * 4.0);
    private static final double TICKS_PER_CM = TICKS_PER_INCH / 2.54;
    private static final double ENCODER_DRIVE_POWER = .3; // .35;

    private double encoder_drive_power = ENCODER_DRIVE_POWER;

    void setEncoderDrivePower(double p) {
        encoder_drive_power = p;
    }

    void clearEncoderDrivePower() {
        encoder_drive_power = ENCODER_DRIVE_POWER;
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

    public void encoderDriveTiles(double direction, double tiles) {
        encoderDriveInches(direction, 24.0 * tiles);
    }

    public void encoderDriveInches(double direction, double inches) {
        final Wheels w = getWheels(direction, 1.0, 0.0);
        final int ticks = (int)(inches * TICKS_PER_INCH);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    public void encoderDriveCM(double direction, double cm) {
        direction %= Math.PI * 2.0;
        final Wheels w = getWheels(direction, 1.0, 0.0);
        final int ticks = (int)(cm * TICKS_PER_CM);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    private void encoderDrive(double lft, double rft, double lrt, double rrt) {
        encoderDrive((int) lft, (int) rft, (int) lrt, (int) rrt);
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

    private double total_light_meter_reading = 0.0;
    private int light_meter_readings_tooken = 0;
    private boolean collect_light_meter_info = false;

    public void startCollectingLightMeter() {
        collect_light_meter_info = true;
        total_light_meter_reading = 0.0;
        light_meter_readings_tooken = 0;
    }

    public void resetGyro() {
        //gyro.resetZAxisIntegrator();
    }

    public int getHeading() {
        return 0;
//        int angle = gyro.getIntegratedZValue() % 360;
//        return angle;
    }

    public NormalizedRGBA getJewelSensorColor()
    {
        return jewelsensor.getNormalizedColors();
    }

    public NormalizedRGBA getBottomSensorColor()
    {
        return bottomsensor.getNormalizedColors();
    }

    public void disableEncoders() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, lf, lr, rf, rr);
    }
}

