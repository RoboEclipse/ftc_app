package org.firstinspires.ftc.Robot2;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class RoverRackusTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RoverRackusClass myRobot = new RoverRackusClass();
    double theta = 0.0, v_theta = 0.0, v_rotation = 0.0;
    double elevatorServoPosition = 1;
    double speedMultiplier = 1;
    double tokenServoPosition = 0;
    double collectorServoPower = 0.5;
    double leadScrewPower = 0;
    boolean cFlipCheck = false;
    private String elevatorDistance;
    private String cFlipEncoder;
    private String cFlipPower;
    

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
        theta = Math.atan2(lx, ly);
        v_theta = Math.sqrt(lx * lx + ly * ly);
        v_rotation = gamepad1.right_stick_x;

        /*

        This part is having some errors due to myRobot not ddefined

        myRobot.drive(theta,  speedMultiplier*0.6*v_theta, 0.5*v_rotation); //move robot

        */
        /*
        Working on this

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
        */

        //Elevator Motor Controls
        double elevatorPower = 1;
        /*
        Checks distance moved up
        double elevatorDistance = myRobot.getElevatorDistanceSensor();
        */
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

