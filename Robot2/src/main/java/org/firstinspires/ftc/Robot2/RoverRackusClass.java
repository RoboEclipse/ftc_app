package org.firstinspires.ftc.Robot2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class RoverRackusClass {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AcszqaP/////AAABmQPh2+SHAkBwsXAKy4LdjLEctzmZIadppxAnjn5ubFiLREbyOyViDtItmB2qAtRfbfJ1GRhhAXPEl992rkY/XW50xxWEVrQ+FGKMC1m6PDC1ropQyBiufMSvx81nz+XF6eSHp6Ct2rrT4YutN9a81bcvGVNA+4EfTu98lzP2HrPUiv0SMlQVq+ze6Fw107r8e7ULdv7dbdfxVtS0X+H4toGS+gxJFyWlgcdHmchQ++I7n8RdaBqoVgItHzjZDDo3lMbPkHIMwsTbWlBYJDBoNGMiFnIBUm1t0J6Yu45dldLZ8eeTPn7R9M9MkwYyWLmAr8Ijs0bxmqDuY4NgvmDqeCIzbfyH7uEximJiwZaGu58u";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private DcMotor lf, lr, rf, rr, leadScrew, cflip, emotor;
    private CRServo exservo, exservoback;
    private DistanceSensor leftDistanceSensor, rightDistanceSensor, elevatorDistanceSensor, extenderDistanceSensor;
    private ColorSensor colorSensor;
    private Servo elevatorServo, markerServo;
    private Telemetry telemetry;
    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    private BNO055IMU imu;
    private Orientation angles;
    private DigitalChannel elevatorLimitSwitch;
    public static final int ENCODERS_CLOSE_ENOUGH = 10;
    int TICKS_PER_ROTATION = 1120;
    public int TICKS_PER_INCH = (int)(1120/(6*Math.PI));
    public int TICKS_PER_CENTIMETER =(int)(TICKS_PER_INCH*2.54);
    public int leadScrewTime=5000;
    RoverRackusConfiguration config = new RoverRackusConfiguration();
    private DcMotor cMotor;
    private DcMotor screwUpPower;


    public void getHorizontalAngle() {
    
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
    }
    public boolean isElevatorLimitSwitchNOTPressed(){
        return elevatorLimitSwitch.getState();
    }
    public double getElevatorDistanceSensor(){
        return elevatorDistanceSensor.getDistance(DistanceUnit.CM);
    }
    public void elevatorServoDrive(double elevatorServoPosition) {
    }
    public void cMotorDrive(double power)
    {
        cMotor.setPower(power);
    }

    public void leadScrewDrive(double power)
    {
        screwUpPower.setPower(power);
    }
    public void extendLeadScrew(double runtime){
        ElapsedTime time = new ElapsedTime();
        screwUpPower.setPower(1);
        while(elevatorLimitSwitch.getState()==true){
            if(time.seconds() > runtime){
                break;
            }
        }
        screwUpPower.setPower(0);
    }
    public void drive(double angle, double speed, double rotations){

    }
    public void readEncoders(){
    }
}
