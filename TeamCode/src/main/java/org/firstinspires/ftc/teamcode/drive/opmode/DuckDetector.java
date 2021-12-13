package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DuckDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location{
        LEFT,
        MIDDLE,
        RIGHT
    }

    private Location location;
    static final Rect LEFT_ROI = new Rect(
            new Point(465, 200),
            new Point(715, 650)
    );
    static final Rect MIDDLE_ROI = new Rect(
            new Point(920, 200),
            new Point(1170, 650)
    );
    static double PERCENT_COLOR_THRESHOLD = 0.14;
  
    public DuckDetector(Telemetry t){ telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);
        Scalar highHSV = new Scalar(180, 255, 255);
        Scalar lowHSV = new Scalar(120, 40, 40);

        Imgproc.GaussianBlur(mat, mat, new Size(5, 5), 0);
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);
        //Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area()/255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area()/255;
        //double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area()/255;

        left.release();
        middle.release();
        //right.release();

        telemetry.addData("Left Value Percentage", Math.round(leftValue*100) + "%");
        telemetry.addData("Middle Value Percentage", Math.round(middleValue*100) + "%");
        //telemetry.addData("Right Value Percentage", Math.round(rightValue*100) + "%");

        telemetry.update();

        boolean markerLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean markerMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        //boolean markerRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if(markerLeft) {
            telemetry.addData("Marker Location", "Left");
            location = Location.LEFT;
        }
        else if(markerMiddle){
            telemetry.addData("Marker Location", "Middle");
            location = Location.MIDDLE;
        }
        else{
            telemetry.addData("Marker Location", "Right");
            location = Location.RIGHT;
        }
        return mat;
    }

    public Location getLocation(){
        return location;
    }
}
