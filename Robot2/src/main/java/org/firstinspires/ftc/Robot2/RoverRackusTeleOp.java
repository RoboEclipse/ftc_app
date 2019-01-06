package org.firstinspires.ftc.Robot2;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class RoverRackusTeleOp extends OpMode {
    public RoverRackusClass myRobot = new RoverRackusClass();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AcszqaP/////AAABmQPh2+SHAkBwsXAKy4LdjLEctzmZIadppxAnjn5ubFiLREbyOyViDtItmB2qAtRfbfJ1GRhhAXPEl992rkY/XW50xxWEVrQ+FGKMC1m6PDC1ropQyBiufMSvx81nz+XF6eSHp6Ct2rrT4YutN9a81bcvGVNA+4EfTu98lzP2HrPUiv0SMlQVq+ze6Fw107r8e7ULdv7dbdfxVtS0X+H4toGS+gxJFyWlgcdHmchQ++I7n8RdaBqoVgItHzjZDDo3lMbPkHIMwsTbWlBYJDBoNGMiFnIBUm1t0J6Yu45dldLZ8eeTPn7R9M9MkwYyWLmAr8Ijs0bxmqDuY4NgvmDqeCIzbfyH7uEximJiwZaGu58u";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private DcMotor lf, lr, rf, rr, leadScrew, cmotor, cflip, emotor;
    private CRServo exservo, exservoback;
    private DistanceSensor leftDistanceSensor, rightDistanceSensor, elevatorDistanceSensor, extenderDistanceSensor;
    private ColorSensor colorSensor;
    private Servo elevatorServo, markerServo;
    private Telemetry telemetry;
    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    private BNO055IMU imu;
    private Orientation angles;
    private DigitalChannel elevatorLimitSwitch;
    public static final int ENCODERS_CLOSE_ENOUGH = 10;
    int TICKS_PER_ROTATION = 1120;
    public int TICKS_PER_INCH = (int)(1120/(6*Math.PI));
    public int TICKS_PER_CENTIMETER =(int)(TICKS_PER_INCH*2.54);
    public int leadScrewTime=5000;
    private static double v_theta = 0;
    private static double v_rotation;
    RoverRackusConfiguration config = new RoverRackusConfiguration();
    private static double leadScrewPower = 1.0;
    private double tokenServoPosition = 0;
    public double elevatorServoPosition = elevatorServo.getPosition();
    private double collectorServoPower = 0.7;
    private double cFlipEncoder = cflip.getCurrentPosition();
    private double cFlipPower = cflip.getPower();
    private boolean cFlipCheck = cflip.isBusy();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        /*

        Old Robot

        runtime.reset();
        myRobot.exServoDrive(0);
        */
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop() {

        //Drive motor controls
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier;
        if (gamepad1.dpad_up) {
            ly = 1;
            speedMultiplier = 0.5;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            speedMultiplier = 0.5;
        } else if (gamepad1.dpad_left) {
            lx = -1;
            speedMultiplier = 0.5;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            speedMultiplier = 0.5;
        } else {
            speedMultiplier = 1;
        }
        double theta = Math.atan2(lx, ly);
        v_theta = Math.sqrt(lx * lx + ly * ly);
        v_rotation = gamepad1.right_stick_x;

        myRobot.drive(theta,  speedMultiplier*0.6*v_theta, 0.5*v_rotation); //move robot



        //Working on this

        //Lead Screw Controls
        if(gamepad1.left_bumper){
            leadScrewPower = 1;
        }
        else if(gamepad1.right_bumper && myRobot.isElevatorLimitSwitchNOTPressed()){
            leadScrewPower = -1;
        }
        //Cut this
        else if(-gamepad2.right_stick_y == 0){
            leadScrewPower = 0;
        }
        else{
            leadScrewPower = -gamepad2.right_stick_y;
            if(gamepad2.right_stick_y<0){
                if(!myRobot.isElevatorLimitSwitchNOTPressed()){
                    leadScrewPower = 0;
                    telemetry.addData("Meow", "Purr");
                }
            }
        }

        myRobot.leadScrewDrive(leadScrewPower);


        //Elevator Motor Controls
        double elevatorPower = 1;

        //Checks distance moved up
        double elevatorDistance = myRobot.getElevatorDistanceSensor();

        elevatorPower = gamepad2.left_stick_y;


        /*
        Take this all in one chunk

        if(gamepad2.left_stick_y  == 0){
            if(elevatorDistance>30){
                elevatorPower = -0.1;
            }
            else{
                elevatorPower = -0.05;
            }
        }

        if(elevatorDistance<6 && gamepad2.left_stick_y>0){
            elevatorPower = 0;
            telemetry.addData("DriveOptimization", "PowerCut");
        }
        if(elevatorDistance<40 && elevatorDistance>20 && gamepad2.left_stick_y<0){
            elevatorServoPosition = 0.7;
        }
        if(elevatorPower>0 && elevatorDistance<40){
            elevatorServoPosition = 1;
        }
        myRobot.eMotorDrive(elevatorPower);

        boolean extenderLimitSwitch = myRobot.isExtenderLimitSwitchNOTPressed();
        //Collector Motor Controls
        if(gamepad2.left_bumper){
            myRobot.cMotorDrive(0.8);
        }
        else if(gamepad2.right_bumper){
            myRobot.cMotorDrive(-0.8);
            myRobot.resetCFlipEncoder();
        }
        else{
            myRobot.cMotorDrive(0);
        }
        */

        //Collector Extender Controls
        //myRobot.exServoDrive(gamepad2.right_stick_y);
        /*
        if(gamepad2.dpad_up){
            collectorServoPower = 0.89;
            myRobot.exServoDrive(collectorServoPower);
        } else if(gamepad2.dpad_down){
            collectorServoPower = 0.11;
            myRobot.exServoDrive(collectorServoPower);
        }
        else{
            collectorServoPower = 0.5;
            myRobot.exServoDrive(collectorServoPower);
        }
        */

        /*
        myRobot.exServoDrive(.99*gamepad2.right_stick_y);
        */
        if (gamepad1.right_trigger > 0.7 && tokenServoPosition <= 1) {
            tokenServoPosition += 0.03;
        } else if (gamepad1.left_trigger > 0.7 && tokenServoPosition >= 0) {
            tokenServoPosition -= 0.03;
        }
        /*
        myRobot.markerServoDrive(tokenServoPosition);
        //Collector Flipper Controls
        double cFlipPower=0;
        int cFlipEncoder = myRobot.getCFlipEncoder();
        if(gamepad1.x){
            cFlipPower = 0.4;
        }
        else if(gamepad1.y){
            cFlipPower = -0.8;
        }
        else if(!gamepad2.a && !gamepad2.b){
            cFlipPower = 0;
        }
        else if(gamepad2.a) {
            cFlipPower = 0.4;
        }
        else if(gamepad2.b){
            cFlipPower = -0.8;
        }

        if(gamepad2.dpad_left && gamepad2.dpad_right){
            cFlipCheck = false;
        }

        if((Math.abs(cFlipEncoder)>RoverRuckusConstants.TICKS_PER_ROTATION / 4) && (cFlipPower > 0)){
            /*
            if(elevatorDistance>10 && elevatorDistance<800){
                cFlipPower = 0;
                telemetry.addData("StoppedFlipElevatorDistance", elevatorDistance);
                Log.d("StopFlip","cFlipEncoder" + Math.abs(cFlipEncoder) + "Elevator too High?" + elevatorDistance);
            }

            if(extenderLimitSwitch){
                cFlipPower = 0;
                telemetry.addData("StoppedFlipLimitSwitch", cFlipEncoder);
                Log.d("StopFlip","ExtenderLimitSwitch:"+extenderLimitSwitch + "cFlipEncoder" + Math.abs(cFlipEncoder));
            }

        }

        myRobot.cFlipDrive(cFlipPower);
        */

        //Elevator Flipper Controls
        if (gamepad2.x && elevatorServoPosition < 1) {
            elevatorServoPosition = 1;
        }
        if (gamepad2.y && elevatorServoPosition > 0) {
            elevatorServoPosition = 0.45;
        }
        if (gamepad2.right_trigger > .5) {
            elevatorServoPosition = 0.6;
        }

        if (gamepad2.left_trigger > .5) {
            elevatorServoPosition = 0.4;
        }

        myRobot.elevatorServoDrive(elevatorServoPosition);

        // Show the elapsed game time and wheel power.

        //telemetry.addData("", "Run Time: " + runtime.toString() + " Angle: " + myRobot.getHorizontalAngle());
        //telemetry.addData("", "LeftDistanceSensor: " + myRobot.getLeftDistanceSensor() + " RightDistanceSensor: "+myRobot.getRightDistanceSensor());
        //telemetry.addData("colorSensor", "Red: " + myRobot.getColorSensorRed() + " Blue: " + myRobot.getColorSensorBlue());
        telemetry.addData("exServoPower", collectorServoPower);
        telemetry.addData("ElevatorServoPosition", elevatorServoPosition);
        telemetry.addData("ElevatorSensor", elevatorDistance + "Elevator Power: " + elevatorPower);
        telemetry.addData("TokenServoPosition", tokenServoPosition);
        telemetry.addData("cFlipEncoder", cFlipEncoder);
        telemetry.addData("cFlipCheck", cFlipCheck);
        telemetry.addData("cFlipPower", cFlipPower);
        myRobot.readEncoders();
        telemetry.update();
        Log.d("exServoPower, ", "" + collectorServoPower);
        Log.d("ElevatorServoPosition", "" + elevatorServoPosition);
        Log.d("ElevatorSensor", elevatorDistance + "Elevator Power: " + elevatorPower);
        Log.d("TokenServoPosition", "" + tokenServoPosition);
        Log.d("cFlipEncoder", "" + cFlipEncoder);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}

