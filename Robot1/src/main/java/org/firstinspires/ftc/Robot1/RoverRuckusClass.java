package org.firstinspires.ftc.Robot1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class RoverRuckusClass {
    private DcMotor lf, lr, rf, rr, leadScrew;
    private Telemetry telemetry;
    private HardwareMap HardwareMap;
    private BNO055IMU imu;
    private Orientation angles;
    public static final int ENCODERS_CLOSE_ENOUGH = 10;

    RoverRuckusConfiguration config = new RoverRuckusConfiguration();

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        lf = hardwareMap.dcMotor.get(config.LeftFrontMotorName);
        lr = hardwareMap.dcMotor.get(config.LeftRearMotorName);
        rf = hardwareMap.dcMotor.get(config.RightFrontMotorName);
        rr = hardwareMap.dcMotor.get(config.RightRearMotorName);
        leadScrew = hardwareMap.dcMotor.get(config.LeadScrewMotorName);
        multiSetMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    public void singleDrive(double power, DcMotor motor){
        motor.setPower(power);
    }
    public void readEncoders(){
        telemetry.addData("Encoders", "lf: " + lf.getCurrentPosition() + " lr: " + lr.getCurrentPosition() + " rf: " +rf.getCurrentPosition() + " rr: "+ rr.getCurrentPosition());
    }

    //Encoder stuff
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
            telemetry.addData("gyroPosition", getAngle());
            telemetry.update();
        }
    }
    public void encoderStrafeDrive(int ticks, double power, String direction) {
        int multiplier = 1;
        if (direction.equals("left")){
            multiplier = -1;
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
            telemetry.addData("gyroPosition", getAngle());
            telemetry.update();
        }
    }

    //Sensor Stuff
    public double getAngle () {
        double angleX;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleX = angles.firstAngle;  //ToDo: assume the 3rd angle is the Robot front dir
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




    //Drive Stuff
    //Preferably Do Not Touch
    public void drive(double direction, double velocity, double rotationVelocity) {
        Wheels w = getWheels(direction, velocity, rotationVelocity);
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
    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

}
