package org.firstinspires.ftc.Robot1;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.Writer;
import java.io.OutputStreamWriter;
import java.io.Serializable;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;


import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Nathan on 4/19/2018.
 */

public class ButtonPositions implements Serializable{
    static String filename = "/storage/emulated/0/FIRST/buttonPositions.txt";

    static private Telemetry telemetry;
    private double xposition = 0.34;
    private double yposition = 0.46;

    public static void setTelemetry(Telemetry val) {
        telemetry = val;
    }

    public double getXPosition () {
        return xposition;
    }

    public void setXPosition(double val) {
        xposition = val;
    }

    public double getYPosition() {
        return yposition;
    }

    public void setYPosition(double val) {
        yposition = val;
    }

    public static ButtonPositions ReadPositions()
    {

        ButtonPositions bp = null;

           // Deserialization
        try {
            // Reading the object from a file
            FileInputStream file = new FileInputStream(filename);
            Reader reader = new InputStreamReader(file);

            // Method for deserialization of object
            Gson gson = new GsonBuilder().create();
            bp = gson.fromJson(reader, ButtonPositions.class);

            reader.close();
            file.close();
        }
        catch(IOException ex) {
            telemetry.addData("Failed to read: ", ex.toString());
        }

        return bp;
}

    public static void WritePositions(ButtonPositions obj) {
        // Serialization
        try {
            //Saving of object in a file
            FileOutputStream file = new FileOutputStream(filename);
            Writer writer = new OutputStreamWriter(file);

            // Method for serialization of object
            Gson gson = new GsonBuilder().create();
            gson.toJson(obj, writer);

            writer.close();
            file.close();
        }
        catch (IOException ex) {
            telemetry.addData("Failed to write: ", ex.toString());
        }
    }
    //Converts information from ¬í sr ,org.firstinspires.ftc.Robot1.ButtonPositionsdÇ]Ö5ZIW to actually useful stuff
    @Override
    public String toString(){
        return  String.format("xposition=%f, yposition=%f", xposition,yposition);
    }

    public String xPositiontoString() {
        return String.format("%f", xposition);
    }

    public String yPositiontoString() {
        return String.format("%f", yposition);
    }

}