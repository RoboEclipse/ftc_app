package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RoverRackusClass {
    private DcMotor cMotor, screwUpPower;
    private DigitalChannel elevatorLimitSwitch;

    public void getHorizontalAngle() {
    
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
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
    public void readEncoders(){
    }
}
