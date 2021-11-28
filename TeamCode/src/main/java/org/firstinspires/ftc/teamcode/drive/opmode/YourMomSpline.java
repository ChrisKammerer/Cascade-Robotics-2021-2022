package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@Autonomous(group = "drive")
public class YourMomSpline extends LinearOpMode{
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-33.51, 10.25, 0);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-10.35, 10.66), 0)
                .splineTo(new Vector2d(10.87, 34.02), 0)
                .splineTo(new Vector2d(12.5, -57.27), 0)
                .splineTo(new Vector2d(39.81, -14.37),0)
                .splineTo(new Vector2d(60.75, -14.39), 0)
                .splineTo(new Vector2d(44.91, 33.19), 0)
                .splineTo(new Vector2d(63.60, 52.48), 0)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
    }
}
