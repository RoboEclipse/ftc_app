package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.*;

public class AnalogEncoderClass {

    //final numbers
    final double maxVolt = 3.3;

    //create elapsed time object
    private ElapsedTime runtime = new ElapsedTime();

    //create Analog Input object
    AnalogInput controller;

    //create Bulk Data object
    RevBulkData bulkData;

    //expansion hub object that we will use to access voltage values
    ExpansionHubEx expansionHub;

    //voltage and angle changes
    double currentVoltage;
    double currentAngleChange;
    double currentTimeChange;
    double totalAngleChange = 0.0;

    //create lists for storing voltage and angle changes
    ArrayList<Double> voltageLevels = new ArrayList<Double>();
    ArrayList<Double> angleChanges = new ArrayList<Double>();
    ArrayList<Double> timeStamps = new ArrayList<Double>();

    public AnalogEncoderClass(){
        
    }
    
    public void init (HardwareMap hardwareMap, String name){
        
        //find encoder on expansion hub using Analog Input class
        controller = hardwareMap.get(AnalogInput.class, name);

        //create extra values for the voltage list to prevent crash in loop
        voltageLevels.add(0.0);
        voltageLevels.add(0.0);
        timeStamps.add(0.1);
        timeStamps.add(0.2);

        //to create new objects in the expansion hub (singleton method)
        RevExtensions2.init();

        //get new expansion hub object for obtaining voltage values
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
    }
    
    public void start (){
        //reset time right before starting program loop
        runtime.reset();
    }
    
    public void loop(){

        //get current voltage and angle change
        currentVoltage = getVoltage();
        currentAngleChange = angleChange();

        //update lists of values
        voltageLevels.add(currentVoltage);
        angleChanges.add(currentAngleChange);
        timeStamps.add(runtime.seconds());

        //update total angle change
        totalAngleChange += currentAngleChange;

        //find the change in time since last run of loop
        currentTimeChange = timeStamps.get(timeStamps.size() - 1) - timeStamps.get(timeStamps.size()-2);

        //save memory
        listLengthCheck();

    }

    //current voltage level
    public double getVoltage(){

        //return the current voltage
        // return controller.getVoltage();

        //define bulk data object
        bulkData = expansionHub.getBulkInputData();
        //use it to find the voltage in milli-volts (then divide by 1000 to get volts)
        return bulkData.getAnalogInputValue(1) / 1000.0;
    }

    //current angle
    public double getAngle(){

        //return the current angle
        return getVoltage()/3.3 * 360.0;
    }

    //calculate the change in angle using the change in voltage
    public double angleChange(){

        double voltage1 = voltageLevels.get(voltageLevels.size() - 1);
        double voltage2 = voltageLevels.get(voltageLevels.size() - 2);

        double voltageDif = 0.0;
        double angleDif = 0.0;

        //initial calculation difference
        voltageDif = (voltage2 - voltage1);


        if (voltageDif < -maxVolt/2){
            //angle goes over the apex of the bump from FLATTER side -> angle gets larger
            voltageDif += maxVolt;
        }
        else if (voltageDif > maxVolt/2){
            //angle goes over the apex of the bump from STEEP side -> angle gets smaller
            voltageDif -= maxVolt;
        }

        angleDif = (voltageDif/3.3) * 360.0;

        return angleDif;
    }

    public void listLengthCheck(){

        //ensure we don't run out of memory
        if (voltageLevels.size() > 50){
            voltageLevels.remove(0);
        }
        if (angleChanges.size() > 50){
            angleChanges.remove(0);
        }
        if (timeStamps.size() > 50){
            timeStamps.remove(0);
        }
    }

}
