package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class regularDriveCodeJohnSucks extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        double subtractHeading = 0;
        double spinnerSpeed = 1.0;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor intakeMotor = hardwareMap.dcMotor.get("rightEncoder");
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        Servo bucketServo = hardwareMap.servo.get("bucket");
        DcMotor spinnerMotor = hardwareMap.dcMotor.get("frontEncoder");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if(gamepad1.right_bumper)
                bucketServo.setPosition(0.65);
            if(gamepad1.left_bumper)
                bucketServo.setPosition(0.2);


            //Controller 2
            if(gamepad2.b)
                intakeMotor.setPower(spinnerSpeed);
            if(!gamepad2.b&&!gamepad2.x)
                intakeMotor.setPower(0);
            if(gamepad2.x)
                intakeMotor.setPower(-1);
            if(gamepad2.right_bumper)
                spinnerMotor.setPower(1);
            else if(gamepad2.left_bumper)
                spinnerMotor.setPower(-1);
            else
                spinnerMotor.setPower(0);
            if(gamepad2.y)
                liftMotor.setPower(1);
            else if(gamepad2.a)
                liftMotor.setPower(-1);
            else
                liftMotor.setPower(.1);
//            if(gamepad2.dpad_up)
//                runToPosition(800, liftMotor, 1);
//            if(gamepad2.dpad_down)
//                runToPosition(-800, liftMotor, -1);



            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
