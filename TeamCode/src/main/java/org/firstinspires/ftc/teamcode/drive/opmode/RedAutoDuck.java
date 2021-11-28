package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.DuckDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Duck Side", group = "Auto")
public class RedAutoDuck extends LinearOpMode {
    private ElapsedTime period = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    public void runToPosition(int count, DcMotor motor, double speed){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(count);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        period.reset();
        motor.setPower(1);
        while((period.seconds()<3) && motor.isBusy()) {
            //wait
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class,
                "webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        DuckDetector detector = new DuckDetector(telemetry);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        Servo bucketServo = hardwareMap.servo.get("bucket");
        bucketServo.setPosition(0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0,0);
        Trajectory dropoffTraj = null;
        switch (detector.getLocation()){
            case LEFT:
                drive.setPoseEstimate(startPose);
                Trajectory traj1 = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-24.3, 24.241)).build();
                drive.followTrajectory(traj1);
                liftMotor.setPower(1);
                delay(.9);
                liftMotor.setPower(0.1);
                bucketServo.setPosition(1);
                delay(1.5);
                bucketServo.setPosition(0);
                liftMotor.setPower(-1);
                delay(.7);
                liftMotor.setPower(0.1);
                dropoffTraj = traj1;
                break;
            case MIDDLE:
                drive.setPoseEstimate(startPose);
                Trajectory traj2 = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-24.722, 24.241)).build();
                drive.followTrajectory(traj2);
                liftMotor.setPower(1);
                delay(1.3);
                liftMotor.setPower(0.1);
                bucketServo.setPosition(1);
                delay(1.5);
                bucketServo.setPosition(0);
                liftMotor.setPower(-1);
                delay(.8);
                liftMotor.setPower(0.1);
                dropoffTraj = traj2;
                break;
            case RIGHT:
                drive.setPoseEstimate(startPose);
                Trajectory traj3 = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-25.722, 24.241)).build();
                drive.followTrajectory(traj3);
                liftMotor.setPower(1);
                delay(2);
                liftMotor.setPower(0.1);
                bucketServo.setPosition(1);
                delay(1.5);
                bucketServo.setPosition(0);
                liftMotor.setPower(-1);
                delay(1.6);
                liftMotor.setPower(0.1);
                dropoffTraj = traj3;

//                Trajectory duckTraj = drive.trajectoryBuilder(traj3.end())
//                        .lineTo(new Vector2d(-.777, -14.896)).build();
//                drive.followTrajectory(duckTraj);
//                Trajectory duckTraj2 = drive.trajectoryBuilder(duckTraj.end())
//                        .strafeRight(4).build();
//                drive.followTrajectory(duckTraj2);

                break;
        }
        Trajectory parkTraj = drive.trajectoryBuilder(dropoffTraj.end())
                .forward(12).build();
        drive.followTrajectory(parkTraj);
        Trajectory finalTraj = drive.trajectoryBuilder(parkTraj.end())
                .lineTo(new Vector2d(-40.638, -25.993)).build();
        drive.followTrajectory(finalTraj);

        Trajectory park = drive.trajectoryBuilder(finalTraj.end())
                .back(16).build();
        drive.followTrajectory(park);

    }
}