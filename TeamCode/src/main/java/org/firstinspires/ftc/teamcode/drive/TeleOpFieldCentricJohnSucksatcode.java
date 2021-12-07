package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "drive")
public class TeleOpFieldCentricJohnSucksatcode extends LinearOpMode {
    double clawOffset = 0.0;


    private ElapsedTime period = new ElapsedTime();

    public void runToPosition(int count, DcMotor motor, double speed){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(count);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        period.reset();
        motor.setPower(speed);
        while((period.seconds()<3) && motor.isBusy()) {
            //wait
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(.1);
    }



    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        double subtractHeading = 0;
        double spinnerSpeed = 1.0;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor intakeMotor = hardwareMap.dcMotor.get("rightEncoder");
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        Servo bucketServo = hardwareMap.servo.get("bucket");
        DcMotor spinnerMotor = hardwareMap.dcMotor.get("frontEncoder");

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(drive.getPoseEstimate());

        waitForStart();

        //bucketServo.setPosition(0);


        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading()+subtractHeading);
            if(gamepad1.right_stick_button)
                subtractHeading = drive.getPoseEstimate().getHeading();
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

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("speed", spinnerSpeed);
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}