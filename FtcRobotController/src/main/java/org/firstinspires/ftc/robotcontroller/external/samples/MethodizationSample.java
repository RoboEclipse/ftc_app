package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MethodizationSample {
    private ColorSensor Sensor_Name;
    private DcMotor Motor_Name;
    private Servo Servo_Name;
    public void initialize(HardwareMap hardwareMap){
        Sensor_Name = hardwareMap.colorSensor.get("Sensor_Configuration_Name");
        Motor_Name = hardwareMap.dcMotor.get("Motor_Configuration_Name");
        Servo_Name = hardwareMap.servo.get("Servo_Name");
    }
    public void setMotorPower(double power){
        Motor_Name.setPower(power);
    }
    public void setServoPower(double position){
        Servo_Name.setPosition(position);
    }
    public double readColorSensorRed(){
        return Sensor_Name.red();
    }
}

