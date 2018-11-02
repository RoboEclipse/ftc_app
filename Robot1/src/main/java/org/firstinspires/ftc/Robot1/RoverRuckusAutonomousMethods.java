package org.firstinspires.ftc.Robot1;


import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

abstract class RoverRuckusAutonomousMethods extends LinearOpMode{
    int ticksPerMineral = (int)(RoverRuckusConstants.TICKS_PER_INCH*13);
    int ticksPerInch = RoverRuckusConstants.TICKS_PER_INCH;
    int leadScrewRunTime=RoverRuckusConstants.leadScrewTime;
    //Declare OpMode members.
    String position = "";
    DetectGoldMineral goldVision;
    @NonNull
    public RoverRuckusClass initialize() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RoverRuckusClass myRobot = new RoverRuckusClass();

        myRobot.initialize(hardwareMap, telemetry);
        goldVision = new DetectGoldMineral();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldVision.setShowCountours(true);
        // start the vision system
        goldVision.enable();
        return myRobot;
    }

    @NonNull
    public List<MatOfPoint> SetPosition() {
        List<MatOfPoint> contours = goldVision.getContours();
        for (int i = 0; i < contours.size(); i++) {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            telemetry.addData("contour" + Integer.toString(i),
                    String.format(Locale.getDefault(), "(%d, %d)", (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));
            telemetry.update();
        }
        //Size of rectangle: (240,320)
        if(contours.isEmpty()){
            position = "Left";
            telemetry.addData("Position", position);
            telemetry.update();
        }
        else{
            Rect presumedParticle = Imgproc.boundingRect(contours.get(0));
            if((presumedParticle.x+presumedParticle.width)/2>=200){
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

    public void LandingFull(RoverRuckusClass myRobot) {
        //Lower the robot onto the field
        myRobot.leadScrewDrive(1);
        sleep(leadScrewRunTime);
        myRobot.leadScrewDrive(0);
        //Move sideways to detach from the hook
        myRobot.encoderStrafeDrive(3*ticksPerInch, 0.4, "Left");
    }

    // 2 inches to the left of start
    public void SampleFullProcess(RoverRuckusClass myRobot) {
        telemetry.update();
        //Drive forward to clear the hook
        myRobot.encoderTankDrive(ticksPerInch*3,ticksPerInch*3, 0.5);
        //Move sideways to realign
        myRobot.encoderStrafeDrive(3*ticksPerInch, 0.4, "Right");
        //Reorient
        myRobot.encoderTurn(0,5,2,0.3);

        //Drive forward to clear the lander
        myRobot.encoderTankDrive(5*ticksPerInch,5*ticksPerInch,0.5);
        //Drive sideways to line up with the gold particle 5 seconds
        if(position == "Left"){
            myRobot.encoderStrafeDrive(ticksPerMineral, 0.4, "Left");
        }
        if(position == "Right"){
            myRobot.encoderStrafeDrive(ticksPerMineral,0.4,"Right");
        }
        myRobot.cFlipDrive(0.2);
        sleep(2000);
        //Rotate to realign
        telemetry.addData("Angle", myRobot.getHorizontalAngle());
        telemetry.update();
        //Drive forward to knock off the gold particle 2 seconds
        myRobot.encoderTankDrive(5*ticksPerInch,5*ticksPerInch,0.5);
        //Retract the collector
        myRobot.cFlipDrive(-0.8);
        sleep(1000);
        myRobot.cFlipDrive(0);
        //aligned to gold particle 5 inches from the lander
        //Move far left
        if(position == "Left"){
            myRobot.encoderStrafeDrive(ticksPerInch*2, 0.4, "Left");
        }
        if(position == "Center"){
            myRobot.encoderStrafeDrive(ticksPerInch*2+ticksPerMineral, 0.4, "Left");
        }
        if(position == "Right"){
            myRobot.encoderStrafeDrive(ticksPerInch*2+2*ticksPerMineral,0.4,"Left");
        }
    }

    public void ClaimFull(RoverRuckusClass myRobot) {

        myRobot.encoderTankDrive(-23*RoverRuckusConstants.TICKS_PER_INCH, -23*RoverRuckusConstants.TICKS_PER_INCH, 0.5);
        myRobot.markerServoDrive(1);

        sleep(100);
    }

    public void Parking(RoverRuckusClass myRobot) {
        //Move sideways until you are an inch or two from the wall
        sleep(1000);
        myRobot.driveUntilCrater(0.4);
    }
}
