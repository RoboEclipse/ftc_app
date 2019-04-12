package org.firstinspires.ftc.Robot1;


import android.support.annotation.*;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

abstract class RoverRuckusAutonomousMethods extends LinearOpMode{
    private int ticksPerMineral = (int)(RoverRuckusConstants.TICKS_PER_INCH*13);
    int ticksPerInch = RoverRuckusConstants.TICKS_PER_INCH;
    private double leadScrewRunTime=RoverRuckusConstants.leadScrewTime;
    private int hookDetach = RoverRuckusConstants.hookDetach;
    private int hookClear = RoverRuckusConstants.hookClear;
    private int landerClear = (int)RoverRuckusConstants.landerClear;
    private int knockOff = RoverRuckusConstants.knockOff;
    //Declare OpMode members.
    String position = "";
    DetectGoldMineral goldVision;


    @NonNull
    public RoverRuckusClass initialize() {
        telemetry.addData("Status", "Initialized");
        RoverRuckusClass myRobot = new RoverRuckusClass();
        myRobot.initialize(hardwareMap, telemetry);
        myRobot.SetOpmodeLiveCheck(
            new OpModeCheck(){
                @Override
                public boolean CheckOpModeIsActive() {
                    return opModeIsActive();
                }
            }
        );
        return myRobot;
    }

    @NonNull
    List<MatOfPoint> SetPosition() {
        // start the vision system
        goldVision.enable();
        sleep(2000);
        List<MatOfPoint> contours = goldVision.getContours();
        for (int i = 0; i < contours.size(); i++) {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            telemetry.addData("contour" + Integer.toString(i),
                    String.format(Locale.getDefault(),
                            "(%d, %d)",
                            (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));
        }
        //Size of rectangle: (240,320)
        if(contours.isEmpty()){
            position = "Left";
            telemetry.addData("Position", position);
            telemetry.update();
        }
        else{
            Rect presumedParticle = Imgproc.boundingRect(contours.get(0));
            Rect challengerParticle;
            for (int i = 0; i<contours.size(); i++){
                challengerParticle = Imgproc.boundingRect(contours.get(i));
                if(challengerParticle.y>presumedParticle.y){
                    presumedParticle = challengerParticle;
                }
            }
            //If above top 10%
            if(presumedParticle.y<32){
                position = "Left";
            }
            else if((presumedParticle.x+presumedParticle.width)/2>=200){
                position = "Right";
                telemetry.addData("Position", position);
                telemetry.update();
            }
            else{
                position = "Center";
                telemetry.addData("Position", position);
                telemetry.update();
            }
        }
        return contours;
    }
 
    void waitForStartTensorFlow(RoverRuckusClass myRobot) {
        myRobot.initTensorFlow(hardwareMap);

        //Get mineral positions
        while (!isStarted()) {
           sleep(500);
            position = myRobot.runTensorFlow();
            //telemetry.addData("Wait", position);
           telemetry.update();
        }
        myRobot.stopTensorFlow();
    }

    void LandingFull(RoverRuckusClass myRobot) {
        //Lower the robot onto the field
        myRobot.markerServoDrive(0.3);
        myRobot.extendLeadScrew(leadScrewRunTime);
        myRobot.tankDrive(0.5,0.5);
        sleep(50);
        myRobot.tankDrive(0,0);
        sleep(100);

        //idealAngle = myRobot.getHorizontalAngle();
        //Move sideways to detach from the hook
        myRobot.encoderStrafeDrive(hookDetach*ticksPerInch, 0.4, "Left");
        if(Math.abs(myRobot.getHorizontalAngle())>10){
            myRobot.encoderTurn(0,10,3,0.1);
        }
    }
    // 2 inches to the left of start
    void rotateSample(RoverRuckusClass myRobot) {
        telemetry.update();
        //Drive forward to clear the hook
        myRobot.encoderTankDrive(ticksPerInch*hookClear,ticksPerInch*hookClear, 0.6);
        //Move sideways to realign
        myRobot.encoderStrafeDrive(hookDetach*ticksPerInch, 0.6, "Right");
        //Begin moving lead screw
        myRobot.leadScrewDrive(-1);
        /*
        if(Math.abs(myRobot.getHorizontalAngle())>5){
            //Reorient
            myRobot.encoderTurn(0,5,2,0.1);
        }
        */

        //Rotate to line up with the gold particle 5 seconds
        if(position.equals("Left")){
            myRobot.encoderTurn(22,15,5,0.6);
        }
        if(position.equals("Right")){
            myRobot.encoderTurn(-30,15,5,0.6);
        }
        //Lower collector
        myRobot.cFlipDrive(0.4);
        sleep(1300);
        myRobot.cFlipDrive(-0.6);
        sleep(200);
        myRobot.cFlipDrive(0);
        sleep(500);
        //Begin running collector
        myRobot.cServoDrive(0.49);
        //Stop lowering leadScrew
        myRobot.leadScrewDrive(0);
        //Extend slide
        while(myRobot.getExtenderDistanceSensor()>5 && opModeIsActive()){
            myRobot.newExMotor(1);
        }
        //Stop extender
        myRobot.newExMotor(0);
        //Wait a little before stopping collector
        sleep(500);
        myRobot.cServoDrive(0);
        //Return mineral to dumping basket
        int stage = 0;
        while(stage<=5 && opModeIsActive()){
            stage = myRobot.autoDump(stage, false);
        }
        myRobot.cFlipDrive(0);
        myRobot.elevatorEncoderDriveStart(-580, -1);
        //Realign
        myRobot.encoderTurn(0,15,5,0.1);
        //Back up
        myRobot.encoderTankDriveInches(-2,0.6);
        //Raise up the elevator
        //Equivalent encoder value: -525
        telemetry.addData("Before Raising elevator", myRobot.getElevatorEncoder());
        myRobot.elevatorEncoderDriveEnd(-580);
        telemetry.addData("After Raising elevator", myRobot.getElevatorEncoder());
        telemetry.update();
        Log.d("Status", "Elevator Done Raising");
        //Stop raising the elevator
        myRobot.eMotorDrive(-0.4);
        //Realign
        myRobot.encoderTurn(0,15,5,0.4);
        //Dump the minerals
        myRobot.elevatorServoDrive(myRobot.dumpingPosition);
        sleep(1000);
        myRobot.elevatorServoDrive(1);
        //Lower the elevator
        telemetry.addData("Before Lowering Elevator", myRobot.getElevatorEncoder());
        telemetry.update();
        myRobot.elevatorEncoderDriveStart(540, myRobot.elevatorModifier);
        //Drive forward again
        myRobot.encoderTankDriveInches(2,0.6);
        myRobot.elevatorEncoderDriveEnd(540);
        telemetry.addData(" After Lowering Elevator", myRobot.getElevatorEncoder());
        telemetry.update();
        Log.d("Status", "Elevator Done Lowering");
        myRobot.eMotorDrive(0);
        //Retract the collector
        myRobot.cFlipDrive(-0.8);
        //Drive forward to clear the lander
        myRobot.encoderTankDrive(landerClear*ticksPerInch,landerClear*ticksPerInch,0.6);
        // verify the time
        myRobot.cFlipDrive(0);
            sleep(100);

        /*
        myRobot.cFlipDrive(0.2);
        sleep(1500);
        */
        telemetry.addData("Angle", myRobot.getHorizontalAngle());
        telemetry.update();

        //Move far left
        myRobot.encoderStrafeDrive(ticksPerInch*5+ticksPerMineral, 0.4, "Left");
    }

    void ClaimFull(RoverRuckusClass myRobot){
        myRobot.markerServoDrive(1);
        telemetry.addData("Drop", "Blue" + myRobot.getColorSensorBlue() +"Red:" + myRobot.getColorSensorRed());
        telemetry.update();
        sleep(500);
    }

   void doubleSampleClaimFull(RoverRuckusClass myRobot, int maxTicks){
        double power= 0.3;
        myRobot.colorSensorDrive(maxTicks, power);
        myRobot.br8kMotors();
        switch (position) {
            case "Right":
                ClaimFull(myRobot);
                myRobot.encoderTankDriveInches(13, 0.5);
                myRobot.encoderStrafeDrive(8*RoverRuckusConstants.TICKS_PER_INCH, 0.5, "Right");
                myRobot.encoderStrafeDrive(8*RoverRuckusConstants.TICKS_PER_INCH, 0.5, "Left");
                myRobot.encoderTankDriveInches(-13, 0.5);
                break;
            case "Center":
                ClaimFull(myRobot);
                myRobot.encoderTankDriveInches(2, 0.3);
                myRobot.encoderStrafeDrive(17*RoverRuckusConstants.TICKS_PER_INCH, 0.3, "Right");
                myRobot.encoderStrafeDrive(17*RoverRuckusConstants.TICKS_PER_INCH, 0.5, "Left");
                myRobot.encoderTankDriveInches(-2,0.5);
                break;
            default:
                myRobot.encoderTankDriveInches(-6, 0.5);
                ClaimFull(myRobot);
                myRobot.encoderStrafeDrive(27*RoverRuckusConstants.TICKS_PER_INCH, 0.3, "Right");
                myRobot.leftRangeSensorStrafe(28*RoverRuckusConstants.TICKS_PER_INCH, 8, 0.5, "Left");
                myRobot.encoderTankDriveInches(6, 0.5);
                break;
        }
        /*
        if(position.equals("Right")){
            ClaimFull(myRobot);
            myRobot.encoderTankDriveInches(13, 0.5);
            myRobot.encoderStrafeDrive(8*RoverRuckusConstants.TICKS_PER_INCH, 0.5, "Right");
            myRobot.encoderStrafeDrive(8*RoverRuckusConstants.TICKS_PER_INCH, 0.5, "Left");
            myRobot.encoderTankDriveInches(-13, 0.5);
        }
        else if (position.equals("Center")){
            ClaimFull(myRobot);
            myRobot.encoderTankDriveInches(2, 0.3);
            myRobot.encoderStrafeDrive(17*RoverRuckusConstants.TICKS_PER_INCH, 0.3, "Right");
            myRobot.encoderStrafeDrive(17*RoverRuckusConstants.TICKS_PER_INCH, 0.5, "Left");
            myRobot.encoderTankDriveInches(-2,0.5);
        }
        else {
            myRobot.encoderTankDriveInches(-6, 0.5);
            ClaimFull(myRobot);
            myRobot.encoderStrafeDrive(27*RoverRuckusConstants.TICKS_PER_INCH, 0.3, "Right");
            myRobot.leftRangeSensorStrafe(28*RoverRuckusConstants.TICKS_PER_INCH, 8, 0.5, "Left");
            myRobot.encoderTankDriveInches(6, 0.5);

        }
        */

    }

    void leftParking(RoverRuckusClass myRobot, double angle, double targetDistance) {
        //Move sideways until you are an inch or two from the wall
        if(Math.abs(myRobot.getHorizontalAngle())>10){
            myRobot.encoderTurn(angle,10,4,0.1);
        }
        myRobot.driveUntilCraterLeft(0.5);
    }

    void rightParking(RoverRuckusClass myRobot, double targetDistance) {
        double angle = 135;
        //Move sideways until you are an inch or two from the wall
        if(Math.abs(myRobot.getHorizontalAngle())>10){
            myRobot.encoderTurn(angle,10,4,0.1);
        }
        myRobot.driveUntilCraterLeft(0.5);
    }

    void newParking(RoverRuckusClass myRobot, double angle, double driveDistance){
        if(Math.abs(myRobot.getHorizontalAngle()-angle)>10){
            myRobot.encoderTurn(angle,10,4,0.1);
        }
        myRobot.gyroTankDriveInches(driveDistance, 0.8);
        while(myRobot.getExtenderDistanceSensor()>15 && opModeIsActive()){
            myRobot.newExMotor(.89);
        }
        myRobot.newExMotor(0);
        myRobot.cFlipDrive(0.4);
        sleep(1000);
        myRobot.cFlipDrive(0);
        myRobot.cServoDrive(0.89);
        while(myRobot.getExtenderDistanceSensor()>5 && opModeIsActive()){
            myRobot.newExMotor(.89);
        }
        myRobot.newExMotor(0);
        sleep(500);
        myRobot.cServoDrive(0);
        /*
        int stage = 0;
        while(stage<=4){
            stage = myRobot.autoDump(stage,  false);
        }

        myRobot.cFlipDrive(0.4);
        sleep(1000);
        myRobot.cFlipDrive(0);
        */
    }
    void doubleSampleParking(RoverRuckusClass myRobot, double angle, double driveDistance){
        if(Math.abs(myRobot.getHorizontalAngle()-angle)>10){
            myRobot.encoderTurn(angle,10,4,0.1);
        }
        myRobot.encoderTankDrive((int)(RoverRuckusConstants.TICKS_PER_INCH*driveDistance), (int)(RoverRuckusConstants.TICKS_PER_INCH*driveDistance), 0.5);

        myRobot.cFlipDrive(0.4);
        sleep(1000);
        myRobot.cFlipDrive(0);

    }
    //Archive
    /*
    @NonNull
    public RoverRuckusClass oldInitialize() {
        telemetry.addData("Status", "Initialized");
        RoverRuckusClass myRobot = new RoverRuckusClass();

        myRobot.initialize(hardwareMap, telemetry);

        goldVision = new DetectGoldMineral();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldVision.setShowCountours(true);

        telemetry.addData("isCalibrated", myRobot.isIMUCalibrated());
        telemetry.update();
        return myRobot;
    }
    */
    // 2 inches to the left of start
    void SampleFullProcess(RoverRuckusClass myRobot) {
        telemetry.update();
        //Drive forward to clear the hook
        myRobot.encoderTankDrive(ticksPerInch*hookClear,ticksPerInch*hookClear, 0.5);
        //Move sideways to realign
        myRobot.encoderStrafeDrive(hookDetach*ticksPerInch, 0.4, "Right");
        if(Math.abs(myRobot.getHorizontalAngle())>5){
            //Reorient
            myRobot.encoderTurn(0,5,2,0.1);
        }
        myRobot.encoderTankDriveInches(landerClear,landerClear);
        if(Math.abs(myRobot.getHorizontalAngle())>5){
            //Reorient
            myRobot.encoderTurn(0,5,2,0.1);
        }

        telemetry.addData("Angle", myRobot.getHorizontalAngle());
        telemetry.update();


        //Retract the collector
        /*
        myRobot.cFlipDrive(-0.8);
        sleep(1000);
        myRobot.cFlipDrive(0);
        */
        //aligned to gold particle 5 inches from the lander
        //Drive forward to knock off the gold particle 2 seconds
        if(position.equals("Left")){
            myRobot.encoderStrafeDrive(ticksPerMineral, 0.4, "Left");
        }
        if(position.equals("Right")){
            myRobot.encoderStrafeDrive(ticksPerMineral,0.4,"Right");
        }
        myRobot.encoderTankDrive(knockOff*ticksPerInch,knockOff*ticksPerInch,0.5);
        sleep(100);
        myRobot.encoderTankDrive(-knockOff*ticksPerInch, -knockOff*ticksPerInch, 0.5);
        myRobot.leadScrewDrive(0);

        if(Math.abs(myRobot.getHorizontalAngle())>10){
            myRobot.encoderTurn(0,10,4,0.1);
        }
        //Retract the collector
        /*
        myRobot.cFlipDrive(-0.8);
        sleep(1000);
        myRobot.cFlipDrive(0);
        */
        //aligned to gold particle 5 inches from the lander
        //Move far left
        if(position.equals("Left")){
            myRobot.encoderStrafeDrive(ticksPerInch*5, 0.4, "Left");
        }
        if(position.equals("Center")){
            myRobot.encoderStrafeDrive(ticksPerInch*5+ticksPerMineral, 0.4, "Left");
        }
        if(position.equals("Right")){
            myRobot.encoderStrafeDrive(ticksPerInch*5+2*ticksPerMineral,0.4,"Left");
        }
    }
}
