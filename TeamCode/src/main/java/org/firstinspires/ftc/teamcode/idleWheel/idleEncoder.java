package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import java.util.*;


@TeleOp(name="idleEncoder", group="Iterative Opmode")
//@Disabled
public class idleEncoder extends OpMode
{
    //final numbers
    final double maxVolt = 3.3;

    //create elapsed time object
    private ElapsedTime runtime = new ElapsedTime();

    //create Analog Input controller object
    AnalogInput controller;

    //voltage and angle changes
    double currentVoltage;
    double currentAngleChange;
    double currentTimeChange;
    double totalAngleChange = 0.0;

    //create lists for storing voltage and angle changes
    ArrayList<Double> voltageLevels = new ArrayList<Double>();
    ArrayList<Double> angleChanges = new ArrayList<Double>();
    ArrayList<Double> timeStamps = new ArrayList<Double>();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        //find controller in hardware map
        controller = hardwareMap.get( AnalogInput.class, "encoder");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        //create buffer for the voltage list
        voltageLevels.add(0.0);
        voltageLevels.add(0.0);
        timeStamps.add(0.1);
        timeStamps.add(0.2);
        voltageLevels.add(returnVoltage());

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

            currentVoltage = returnVoltage();
            currentAngleChange = angleChange();

            totalAngleChange += currentAngleChange;

            voltageLevels.add(currentVoltage);
            angleChanges.add(currentAngleChange);
            timeStamps.add(runtime.seconds());

            currentTimeChange = timeStamps.get(timeStamps.size() - 1) - timeStamps.get(timeStamps.size()-2);

            System.out.println(angleChanges.toString());

            if (voltageLevels.size() > 50){
                voltageLevels.remove(0);
            }
            if (angleChanges.size() > 50){
                angleChanges.remove(0);
            }

            telemetry.addData("Current Voltage", currentVoltage);
            telemetry.addData("Current Angle Change", currentAngleChange);
            telemetry.addData("Total Angle Change", (int) totalAngleChange);
            telemetry.addData("Angular Velocity", (int) (currentAngleChange/currentTimeChange));
            telemetry.addData("Update Hertz", (int)(1/currentTimeChange));
            telemetry.update();
        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    //small method to get input voltage of encoder
    public double returnVoltage(){

        //return the current voltage
        return controller.getVoltage();
    }

    //calculate the change in angle using the change in voltage
    public double angleChange(){

        double volt1 = voltageLevels.get(voltageLevels.size() - 1);
        double volt2 = voltageLevels.get(voltageLevels.size() - 2);

        double voltDif = 0.0;
        double angleDif = 0.0;

        //initial calculation difference
        voltDif = (volt2 - volt1);


        if (voltDif < -maxVolt/2){
            //angle goes over the apex of the bump from FLATTER side -> angle gets larger
            voltDif += maxVolt;
        }
        else if (voltDif > maxVolt/2){
            //angle goes over the apex of the bump from STEEP side -> angle gets smaller
            voltDif -= maxVolt;
        }

        angleDif = (voltDif/3.3) * 360.0;

        return angleDif;
    }

}
