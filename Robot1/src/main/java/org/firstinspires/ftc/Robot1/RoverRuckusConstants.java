package org.firstinspires.ftc.Robot1;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RoverRuckusConstants {
    public static int ENCODERS_CLOSE_ENOUGH = 10;
    static public int TICKS_PER_ROTATION = 1120;
    static public int TICKS_PER_INCH = (int)(1120/(6*Math.PI));
    static public double leadScrewTime=5.1;
    static public int hookDetach = 3;
    static public int hookClear = 5;
    static public double landerClear = 7;
    static public double knockOff = 7.5;
    static public int wallDistance = 10;
    static public double park = 43;
    static public double flipOutPower = 0.25;
    static public double flipInPower = -.5;
    static public int tolerance = 3;
}
