package org.firstinspires.ftc.teamcode.Subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BoeDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum Cone {
        RED,
        BLUE,
        UNDECIDED
    }

    // TOPLEFT anchor point for the bounding box
    private static Point FAR_TOPLEFT_ANCHOR_POINT = new Point(55, 120);
    private static Point MID_TOPLEFT_ANCHOR_POINT = new Point(65, 130);
    private static Point CLOSE_TOPLEFT_ANCHOR_POINT = new Point(75, 140);

    // Width and height for the bounding box
    public static int FAR_REGION_WIDTH = 25;
    public static int FAR_REGION_HEIGHT = 15;
    public static int MID_REGION_WIDTH = 45;
    public static int MID_REGION_HEIGHT = 35;
    public static int CLOSE_REGION_WIDTH = 65;
    public static int CLOSE_REGION_HEIGHT = 55;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_red_bounds  = new Scalar(100, 0, 0, 255),
            upper_red_bounds  = new Scalar(255, 150, 150, 255),
            lower_blue_bounds = new Scalar(0, 0, 100, 255),
            upper_blue_bounds = new Scalar(150, 150, 255, 255);

    // Color definitions
    private final Scalar
            RED  = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255),
            BLACK = new Scalar(0, 0, 0);

    // Percent and mat definitions
    private double redPercent, bluePercent;
    private Mat redMat = new Mat(), blueMat = new Mat(), blurredMatFar = new Mat(), blurredMatMid = new Mat(), blurredMatClose = new Mat(), kernel = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            FAR_TOPLEFT_ANCHOR_POINT.x,
            FAR_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            FAR_TOPLEFT_ANCHOR_POINT.x + FAR_REGION_WIDTH,
            FAR_TOPLEFT_ANCHOR_POINT.y + FAR_REGION_HEIGHT);

    Point sleeve_pointC = new Point(
            MID_TOPLEFT_ANCHOR_POINT.x,
            MID_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointD = new Point(
            MID_TOPLEFT_ANCHOR_POINT.x + MID_REGION_WIDTH,
            MID_TOPLEFT_ANCHOR_POINT.y + MID_REGION_HEIGHT);

    Point sleeve_pointE = new Point(
            CLOSE_TOPLEFT_ANCHOR_POINT.x,
            CLOSE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointF = new Point(
            CLOSE_TOPLEFT_ANCHOR_POINT.x + CLOSE_REGION_WIDTH,
            CLOSE_TOPLEFT_ANCHOR_POINT.y + CLOSE_REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile Cone position = Cone.UNDECIDED;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMatClose, new Size(5, 5));
        blurredMatFar = blurredMatClose.submat(new Rect(sleeve_pointC, sleeve_pointD));
        blurredMatClose = blurredMatClose.submat(new Rect(sleeve_pointE, sleeve_pointF));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatFar, blurredMatFar, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMatFar, lower_red_bounds, upper_red_bounds, redMat);
        Core.inRange(blurredMatFar, lower_blue_bounds, upper_blue_bounds, blueMat);

        // Gets color specific values
        redPercent = Core.countNonZero(redMat);
        bluePercent = Core.countNonZero(blueMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(redPercent, bluePercent);

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == redPercent && redPercent >= 50) {
            position = Cone.RED;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    RED,
                    2
            );
        } else if (maxPercent == bluePercent && bluePercent >= 50) {
            position = Cone.BLUE;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BLUE,
                    2
            );
        } else {
            position = Cone.UNDECIDED;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BLACK,
                    2
            );
        }

        // Memory cleanup
        blurredMatFar.release();
        blurredMatClose.release();
        blurredMatMid.release();
        redMat.release();
        blueMat.release();
        kernel.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public Cone getPosition() {
        return position;
    }
}
