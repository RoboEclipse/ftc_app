package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;


@TeleOp(name="BulkReadTest", group="Iterative Opmode")
//@Disabled
public class BulkReadTest extends OpMode
{
    //create Bulk Data object
    RevBulkData bulkData;

    //expansion hub object that we will use to access voltage values
    ExpansionHubEx expansionHub;

    ExpansionHubMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    ArrayList<Double> timeStamps = new ArrayList<Double>();

    double currentTimeChange;

    private ElapsedTime runtime = new ElapsedTime();

     //* Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        //to create new objects in the expansion hub (singleton method)
        RevExtensions2.init();

        //get new expansion hub object for obtaining voltage values
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        frontRightDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("frontRightDrive");
        frontLeftDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("frontLeftDrive");
        backRightDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("backRightDrive");
        backLeftDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("backLeftDrive");

        timeStamps.add(0.1);
        timeStamps.add(0.2);




        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


     //* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }


     //* Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {



    }


     //* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        bulkData = expansionHub.getBulkInputData();

        timeStamps.add(runtime.seconds());

        currentTimeChange = timeStamps.get(timeStamps.size() - 1) - timeStamps.get(timeStamps.size()-2);

        if (timeStamps.size() > 50){
            timeStamps.remove(0);
        }

        expansionHub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);

        telemetry.addData("M0 enocder", bulkData.getMotorCurrentPosition(frontRightDrive));
        telemetry.addData("M1 encoder", bulkData.getMotorCurrentPosition(frontLeftDrive));
        telemetry.addData("M2 encoder", bulkData.getMotorCurrentPosition(backRightDrive));
        telemetry.addData("M3 encoder", bulkData.getMotorCurrentPosition(backLeftDrive));
        telemetry.addData("Analog encoder", bulkData.getAnalogInputValue(1) / 1000.0);
        telemetry.addData("Update Hertz", (int)(1/currentTimeChange));

        telemetry.update();




        }


     //* Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }



}
