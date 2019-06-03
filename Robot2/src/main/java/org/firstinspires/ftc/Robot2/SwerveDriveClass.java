package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class SwerveDriveClass {
    private Servo lfRotate, lbRotate, rfRotate, rbRotate;
    private DcMotor lf, lb, rf, rb;
    public void drive() {
        lf.setPower(1);
        lb.setPower(1);
        rf.setPower(1);
        rb.setPower(1);
    }
    public void setMidPosition() {
        lbRotate.setPosition(0.25);
        lfRotate.setPosition(0.25);
        rbRotate.setPosition(0.25);
        rfRotate.setPosition(0.25);
    }
    public void setSidePosition() {
        lbRotate.setPosition(0.5);
        lfRotate.setPosition(0.5);
        rbRotate.setPosition(0.5);
        rfRotate.setPosition(0.5);
    }
    public void cat()
    {
        // MEOW !!!
    }
    public void rotateRight() {
        lb.setPower(1);
        lf.setPower(1);
        rb.setPower(-1);
        rf.setPower(-1);
    }
    public void rotateLeft() {
        lb.setPower(1);
        lf.setPower(1);
        rb.setPower(-1);
        rf.setPower(-1);
    }

}
