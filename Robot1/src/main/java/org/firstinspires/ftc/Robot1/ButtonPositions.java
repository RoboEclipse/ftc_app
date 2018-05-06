package org.firstinspires.ftc.Robot1;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

/**
 * Created by Nathan on 4/19/2018.
 */

public class ButtonPositions implements Serializable{

    static String filename = "/storage/emulated/0/FIRST/buttonPositions.txt";

    public double xposition;
    public double yposition;

    public static ButtonPositions ReadPositions()
    {

        ButtonPositions object1 = null;

        // Deserialization
        try
        {
            // Reading the object from a file
            FileInputStream file = new FileInputStream(filename);
            ObjectInputStream in = new ObjectInputStream(file);

            // Method for deserialization of object
            object1 = (ButtonPositions)in.readObject();

            in.close();
            file.close();

            return object1;
        }

        catch(IOException ex) {

        }

        catch(ClassNotFoundException ex) {
        }

        return null;
    }

    public static void WritePositions(ButtonPositions obj) {
        // Serialization
        try {
            //Saving of object in a file
            FileOutputStream file = new FileOutputStream(filename);
            ObjectOutputStream out = new ObjectOutputStream(file);

            // Method for serialization of object
            out.writeObject(obj);

            out.close();
            file.close();
        }
        catch (IOException ex) {
        }
    }

    public String toString(){
        return  String.format("xposition=%f,yposition=%f", xposition,yposition);
    }

}