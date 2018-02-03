package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Eric on 2/2/2018.
 */

public class DogeFunctions {
    //Import the configuration
    DogeConfiguration config = new DogeConfiguration();
    //Initialize the motors
    private DcMotor mainMotor;
    private Servo tailServo, mouthServo;
    //You have to put in hardware map as one of the inputs for the function
    public void initialize(HardwareMap cat){
        //Define the names of the motors
        mainMotor=cat.dcMotor.get(config.MainMotorName);
        tailServo=cat.servo.get(config.TailServoName);
        mouthServo=cat.servo.get(config.MouthServoName);
    }
    public void MoveMainMotor(double power){
        mainMotor.setPower(power);
    }
    public void SetTailPosition (double position){
        tailServo.setPosition(position);
    }
    public void SetMouthPosition (double position) {
        mouthServo.setPosition(position);
    }
}
