package org.firstinspires.ftc.Robot1;


import android.support.annotation.NonNull;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

abstract class RoverRuckusAutonomousMethods extends LinearOpMode{
    int ticksPerMineral = (int)(RoverRuckusConstants.TICKS_PER_INCH*13);
    int ticksPerInch = RoverRuckusConstants.TICKS_PER_INCH;
    double leadScrewRunTime=RoverRuckusConstants.leadScrewTime;
    int hookDetach = RoverRuckusConstants.hookDetach;
    int hookClear = RoverRuckusConstants.hookClear;
    int landerClear = (int)RoverRuckusConstants.landerClear;
    int knockOff = RoverRuckusConstants.knockOff;
    double idealAngle = 0;
    //Declare OpMode members.
    String position = "";
    DetectGoldMineral goldVision;
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

    @NonNull
    public RoverRuckusClass initialize() {
        telemetry.addData("Status", "Initialized");
        RoverRuckusClass myRobot = new RoverRuckusClass();
        myRobot.initialize(hardwareMap, telemetry);
        return myRobot;
    }

    @NonNull
    public List<MatOfPoint> SetPosition() {
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
 
    public void waitForStartTensorFlow(RoverRuckusClass myRobot) {
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

    public void LandingFull(RoverRuckusClass myRobot) {
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
    public void SampleFullProcess(RoverRuckusClass myRobot) {
        telemetry.update();
        //Drive forward to clear the hook
        myRobot.encoderTankDrive(ticksPerInch*hookClear,ticksPerInch*hookClear, 0.5);
        //Move sideways to realign
        myRobot.encoderStrafeDrive(hookDetach*ticksPerInch, 0.4, "Right");

        if(Math.abs(myRobot.getHorizontalAngle())>5 || Math.abs(myRobot.getHorizontalAngle())<15){
            //Reorient
            //TODO: Possibly remove idealAngle
            myRobot.encoderTurn(idealAngle,5,2,0.1);
        }

        //Drive forward to clear the lander
        myRobot.encoderTankDrive(landerClear*ticksPerInch,landerClear*ticksPerInch,0.5);
        sleep(100);
        myRobot.leadScrewDrive(-1);
        //Drive sideways to line up with the gold particle 5 seconds
        if(position == "Left"){
            myRobot.encoderStrafeDrive(ticksPerMineral, 0.4, "Left");
        }
        if(position == "Right"){
            myRobot.encoderStrafeDrive(ticksPerMineral,0.4,"Right");
        }
        /*
        myRobot.cFlipDrive(0.2);
        sleep(1500);
        */
        telemetry.addData("Angle", myRobot.getHorizontalAngle());
        telemetry.update();

        //Drive forward to knock off the gold particle 2 seconds
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
        if(position == "Left"){
            myRobot.encoderStrafeDrive(ticksPerInch*5, 0.4, "Left");
        }
        if(position == "Center"){
            myRobot.encoderStrafeDrive(ticksPerInch*5+ticksPerMineral, 0.4, "Left");
        }
        if(position == "Right"){
            myRobot.encoderStrafeDrive(ticksPerInch*5+2*ticksPerMineral,0.4,"Left");
        }

    }

    public void ClaimFull(RoverRuckusClass myRobot){
        myRobot.markerServoDrive(1);
        telemetry.addData("Drop", "Blue" + myRobot.getColorSensorBlue() +"Red:" + myRobot.getColorSensorRed());
        telemetry.update();
        sleep(1500);
    }

    public void doubleSampleClaimFull(RoverRuckusClass myRobot, int maxTicks, double power){
        myRobot.colorSensorDrive(maxTicks, power);
        myRobot.br8kMotors();
        if(position == "Right"){
            ClaimFull(myRobot);
            myRobot.encoderTankDriveInches(13, 0.5);
            myRobot.encoderStrafeDrive(8*RoverRuckusConstants.TICKS_PER_INCH, 0.5, "Right");
            myRobot.encoderStrafeDrive(8*RoverRuckusConstants.TICKS_PER_INCH, 0.5, "Left");
            myRobot.encoderTankDriveInches(-13, 0.5);
        }
        else if (position == "Center"){
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

    }

    public void leftParking(RoverRuckusClass myRobot, double angle, double targetDistance) {
        //Move sideways until you are an inch or two from the wall
        if(Math.abs(myRobot.getHorizontalAngle())>10){
            myRobot.encoderTurn(angle,10,4,0.1);
        }
        myRobot.driveUntilCraterLeft(0.5, targetDistance);
    }

    public void rightParking(RoverRuckusClass myRobot, double angle, double targetDistance) {
        //Move sideways until you are an inch or two from the wall
        if(Math.abs(myRobot.getHorizontalAngle())>10){
            myRobot.encoderTurn(angle,10,4,0.1);
        }
        myRobot.driveUntilCraterLeft(0.5, targetDistance);
    }

    public void newParking(RoverRuckusClass myRobot, double angle, double driveDistance){
        if(Math.abs(myRobot.getHorizontalAngle()-angle)>10){
            myRobot.encoderTurn(angle,10,4,0.1);
        }
        myRobot.encoderTankDrive((int)(RoverRuckusConstants.TICKS_PER_INCH*driveDistance), (int)(RoverRuckusConstants.TICKS_PER_INCH*driveDistance), 0.5);
        myRobot.cFlipDrive(0.4);
        sleep(1000);
        myRobot.cFlipDrive(0);
        myRobot.exServoDrive(.89);
        sleep(500);
        myRobot.exServoDrive(0.5);
        myRobot.cServoDrive(0.79);
        sleep(1000);
        myRobot.cServoDrive(0);
        myRobot.cFlipDrive(-0.1);
        for(int stage = 0; stage <=4; stage++){
            myRobot.autoDump(stage,  false);
        }
        myRobot.cFlipDrive(0.4);
        sleep(1000);
        myRobot.cFlipDrive(0);
    }

}
