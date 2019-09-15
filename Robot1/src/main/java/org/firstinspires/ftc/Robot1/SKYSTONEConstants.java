package org.firstinspires.ftc.Robot1;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SKYSTONEConstants {
    public static double TICKS_PER_ROTATION = 103.6;
    public static double GEAR_RATIO = 20;
    public static double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    public static double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4*Math.PI);
    public static double testDistance = 5;
}
