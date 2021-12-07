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

@Autonomous(name = "RedAutoWarehouseJohnsCodeSucks", group = "Auto")
public class RedAutoWarehouseJohnsCodeSucks extends LinearOpMode {
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
        Trajectory dropoffTraj = null;

        double liftMotorTime = 0.0;
        switch (detector.getLocation()){
            case LEFT:
                liftMotorTime = 0.2;
                break;
            case MIDDLE:
                liftMotorTime = 1;
                break;
            case RIGHT:
                liftMotorTime = 2.2;
                break;
        }
        drive.setPoseEstimate(startPose);
        Trajectory dropoffBlock = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-13.775, -22.116)).build(); //og -15.775
        drive.followTrajectory(dropoffBlock);
        if(liftMotorTime>0.2) {
            delay(0.3);
            liftMotor.setPower(-1);
            delay(liftMotorTime);
            liftMotor.setPower(-0.1);
            bucketServo.setPosition(0.67);
            delay(1);
            bucketServo.setPosition(0.2);
            liftMotor.setPower(1);
            delay(liftMotorTime-0.3);
            liftMotor.setPower(0);
        }
        else{
            liftMotor.setPower(-1);
            delay(liftMotorTime);
            liftMotor.setPower(-0.1);
            delay(.5);
            bucketServo.setPosition(0.62);
            delay(1.5);
            bucketServo.setPosition(0.2);
            liftMotor.setPower(0);
        }
        Trajectory turnToWall = drive.trajectoryBuilder(dropoffBlock.end())
                .lineToLinearHeading(new Pose2d(-0, -23.116, Math.toRadians(90))).build();
        drive.followTrajectory(turnToWall);
        Trajectory moveToWarehouse = drive.trajectoryBuilder(turnToWall.end())
                .lineTo(new Vector2d(0, 42)).build();
        drive.followTrajectory(moveToWarehouse);
        intakeMotor.setPower(1);
        delay(.8);
        intakeMotor.setPower(-1);
        delay(.8);
        intakeMotor.setPower(0);
        Trajectory leaveWarehouse = drive.trajectoryBuilder(moveToWarehouse.end())
                .lineToLinearHeading(new Pose2d(-2.97, 4, Math.toRadians(70))).build();
        drive.followTrajectory(leaveWarehouse);
        Trajectory dropoffBlock2 = drive.trajectoryBuilder(leaveWarehouse.end())
                .lineToLinearHeading(new Pose2d(-13.775, -22.116, Math.toRadians(0))).build();
        drive.followTrajectory(dropoffBlock2);
        delay(0.5);
        liftMotor.setPower(-1);
        delay(2.4);
        liftMotor.setPower(-0.1);
        bucketServo.setPosition(0.65);
        delay(1);
        bucketServo.setPosition(0.2);
        liftMotor.setPower(1);
        delay(1.8);
        liftMotor.setPower(0);

        Trajectory turnToWall2 = drive.trajectoryBuilder(dropoffBlock2.end())
                .lineToLinearHeading(new Pose2d(-0, -23.116, Math.toRadians(90))).build(); //math.toradians is turning
        drive.followTrajectory(turnToWall2);
        Trajectory moveToWarehouse2 = drive.trajectoryBuilder(turnToWall2.end())
                .lineTo(new Vector2d(0, 44)).build();
        drive.followTrajectory(moveToWarehouse2);


    }
}