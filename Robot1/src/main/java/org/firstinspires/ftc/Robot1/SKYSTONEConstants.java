package org.firstinspires.ftc.Robot1;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SKYSTONEConstants {
    double TICKS_PER_ROTATION = 103.6;
    double GEAR_RATIO = 20;
    double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4*Math.PI);
    double testDistance = 5;
}
