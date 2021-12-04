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

@Autonomous(name = "Blue Warehouse Side", group = "Auto")
public class BlueAutoWarehouse extends LinearOpMode {
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

        Trajectory dropoffBlock = null;
        double liftMotorTime = 0.0;

        switch (detector.getLocation()){
            case LEFT:
                liftMotorTime = 0.0;
                dropoffBlock = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-17, 23.262)).build();
                break;
            case MIDDLE:
                liftMotorTime = 1;
                dropoffBlock = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-14.675, 24.762)).build();
                break;
            case RIGHT:
                liftMotorTime = 2.2;
                dropoffBlock = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-14.175, 24.762)).build();
                break;
        }
        drive.setPoseEstimate(startPose);
        drive.followTrajectory(dropoffBlock);
        if(liftMotorTime!=0) {
            delay(0.5);
            liftMotor.setPower(-1);
            delay(liftMotorTime);
            liftMotor.setPower(-0.1);
            bucketServo.setPosition(0.67);
            delay(2);
            bucketServo.setPosition(0.2);
            liftMotor.setPower(1);
            delay(liftMotorTime-0.3);
            liftMotor.setPower(0);
        }
        else{
            bucketServo.setPosition(0.65);
            delay(1.7);
            bucketServo.setPosition(0.2);
        }
        Trajectory turnToWall = drive.trajectoryBuilder(dropoffBlock.end())
                .lineToLinearHeading(new Pose2d(0, 23.116, Math.toRadians(-90))).build();
        drive.followTrajectory(turnToWall);
        Trajectory moveToWarehouse = drive.trajectoryBuilder(turnToWall.end())
                .lineTo(new Vector2d(-1, -27.8)).build();
        drive.followTrajectory(moveToWarehouse);
    }
}
