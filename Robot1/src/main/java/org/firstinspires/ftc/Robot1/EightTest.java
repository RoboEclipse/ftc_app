package org.firstinspires.ftc.Robot1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.Robot1.SampleMecanumDriveBase;
import org.firstinspires.ftc.Robot1.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class EightTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30,30,0))
                        .splineTo(new Pose2d(0, 0, 0))
                        .build()
        );

        /*
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(30, 15, 0)).build());
        drive.turnSync(-Math.PI/2);
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(15).build());
        drive.turnSync(-Math.PI/2);
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(0, 15, -Math.PI)).build());
        drive.turnSync(-Math.PI/2);
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(15).build());
*/


    }
}
