package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;


@TeleOp(name="idleEncoderRefactored", group="Iterative Opmode")
//@Disabled
public class idleEncoderRefactored extends OpMode
{

    //create encoder object using the class (create all variables)
    AnalogEncoderClass encoder = new AnalogEncoderClass();


     //* Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        //find encoder in hardware map and set up bulk data reader
        encoder.init(hardwareMap, "encoder");

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

        //reset the runtime
        encoder.start();

    }


     //* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //read the values on the encoder and find calculate other numbers based on them
        encoder.loop();

        //display data that we measured
        telemetry.addData("Current Voltage", encoder.currentVoltage);
        telemetry.addData("Current Angle Change", encoder.currentAngleChange);
        telemetry.addData("Total Angle Change", (int) encoder.totalAngleChange);
        telemetry.addData("Angular Velocity", (int) (encoder.currentAngleChange / encoder.currentTimeChange));
        telemetry.addData("Update Hertz", (int)(1/encoder.currentTimeChange));

        }


     //* Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }



}
