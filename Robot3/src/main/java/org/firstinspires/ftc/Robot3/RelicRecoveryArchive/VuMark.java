package org.firstinspires.ftc.Robot3.RelicRecoveryArchive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Nathan on 10/6/2017.
 */

public class VuMark {
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private Telemetry telemetry = null;
    private HardwareMap hardwareMap = null;

    public RelicRecoveryVuMark vuMark = null;

    public VuMark(Telemetry t, HardwareMap h)
    {
        telemetry = t;
        hardwareMap = h;
    }

    public void onInit(VuforiaLocalizer.CameraDirection cameraDirection)
    {
        telemetry.addData("Vuforia status", "Initialized");
        telemetry.addData("Vuforia camera direction", cameraDirection.toString());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUI5q5H/////AAAAGSEMjpyGgUQShcsWbRdVD+cPPRp2R7h0YRM3n/Eggr2RZiAqfjKjP4wAOXL+LAxThIsShaNGaO267fLKUQhFCd44x+YEl3QtulDmhLfUWrKT+RBkW4pg0m0z0CrFRYl2b4Z1Rq/tZbEmAO04etGfC2SolxfIqRovS1zn0QOjtLyKMsYyeHQFUXY3wSHbAh7f3cMulOH1CvFnOhuyYgL3h7YFDo9UdGftCcaoR1aQNC0+1oV60LvuCFOyBN64r5SfA/68uokuWTVNBbO8Ri+KRdUDIcn8r8mC0u4LUiqU8qWA1tK8oMqzxz6vUyeC9B+Xgg2J4XPxmDLXqauxEJHmEJfPp7hUTX1JOmpENHDIJKV8";
        parameters.cameraDirection = cameraDirection;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        // Tell the driver that initialization is complete.
        telemetry.addData("Vuforia status", "Initialized");

    }

    public void onLoop() {

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        }
/*
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
        telemetry.addData("Pose", format(pose));

        // We further illustrate how to decompose the pose into useful rotational and
        // translational components
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);

            // Extract the rotational components of the target relative to the robot
            double rX = rot.firstAngle;
            double rY = rot.secondAngle;
            double rZ = rot.thirdAngle;
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }
*/
        telemetry.update();
    }

    public boolean isCenterRelicVisable()
    {
        return vuMark == RelicRecoveryVuMark.CENTER;
    }

    public boolean isLeftRelicVisable()
    {
        return vuMark == RelicRecoveryVuMark.LEFT;
    }

    public boolean isRightRelicVisable()
    {
        return vuMark == RelicRecoveryVuMark.RIGHT;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
