package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.Runnable;

@Autonomous(name = "Blue WaffleHouse Sussy :flushed:", group = "Auto")
public class BlueAutoWarehouseExperimental extends LinearOpMode implements Runnable{
    public void run(){

    }
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
        DcMotor intakeMotor = hardwareMap.dcMotor.get("rightEncoder");
        bucketServo.setPosition(0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0,0);

        Trajectory dropoffBlock = null;
        double liftMotorTime = 0.0;

        switch (detector.getLocation()){
            case LEFT:
                liftMotorTime = 0.80;
                dropoffBlock = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-11.456, 28.090)).build();
                break;
            case MIDDLE:
                liftMotorTime = 1.25;
                dropoffBlock = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-14.675, 24.762)).build();
                break;
            case RIGHT:
                liftMotorTime = 1.80;
                dropoffBlock = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-14.175, 24.762)).build();
                break;
        }
        drive.setPoseEstimate(startPose);
        drive.followTrajectory(dropoffBlock);
        if(liftMotorTime!=0) {
            delay(0.1);
            liftMotor.setPower(-1);
            delay(liftMotorTime);
            liftMotor.setPower(-0.1);
            bucketServo.setPosition(0.8);
            delay(1.4);
            bucketServo.setPosition(0);
            liftMotor.setPower(1);
            delay(liftMotorTime-0.3);
            liftMotor.setPower(0);
        }
        else{
            bucketServo.setPosition(0.8);
            delay(1.7);
            bucketServo.setPosition(0);
        }
        Trajectory splineToWarehouse = drive.trajectoryBuilder(dropoffBlock.end())
                // SplineTo the wall
                .splineTo(new Vector2d(0,23.116), Math.toRadians(-100))
                // SplineTo the warehouse
                .splineTo(new Vector2d(2, -29.8), Math.toRadians(-90))
                .build();
        drive.followTrajectory(splineToWarehouse);
        intakeMotor.setPower(1);
        delay(1.7);
        intakeMotor.setPower(-1);
        delay(1);
        intakeMotor.setPower(0);
        Trajectory splineToDropoff = drive.trajectoryBuilder(splineToWarehouse.end())
                .lineTo(new Vector2d(3, 23.116))
                .splineTo(new Vector2d(-14.175, 23.762), Math.toRadians(10))
                .build();
        drive.followTrajectory(splineToDropoff);
        delay(0.1);
        liftMotor.setPower(-1);
        delay(1.7);
        liftMotor.setPower(-0.1);
        bucketServo.setPosition(0.8);
        delay(1.5);
        bucketServo.setPosition(0);
        liftMotor.setPower(1);
        delay(1.8);
        liftMotor.setPower(0);
        Trajectory splineToEnd = drive.trajectoryBuilder(splineToDropoff.end())
                .splineTo(new Vector2d(3, 23.116), Math.toRadians(-90))
                .splineTo(new Vector2d(3, -27.8), Math.toRadians(-90))
                .splineTo(new Vector2d(-25, -25), Math.toRadians(165))
                .build();
        drive.followTrajectory(splineToEnd);
    }
}
