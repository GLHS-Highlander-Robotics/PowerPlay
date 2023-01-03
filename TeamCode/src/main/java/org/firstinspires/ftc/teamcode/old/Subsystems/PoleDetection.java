package org.firstinspires.ftc.teamcode.old.Subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PoleDetection extends OpenCvPipeline {
    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds = new Scalar(150, 150, 0, 255),
            upper_yellow_bounds = new Scalar(255, 255, 150, 255);
    // TOPLEFT anchor point for the bounding box
    private static final Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(45, 60);
    // Width and height for the bounding box
    public static int REGION_WIDTH = 25;
    public static int REGION_HEIGHT = 15;
    // Color definitions
    private final Scalar
            YELLOW = new Scalar(255, 255, 0),
            CYAN = new Scalar(0, 255, 255);
    private final Mat yelMat = new Mat();
    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    private volatile boolean isPole = false;
    // Percent and mat definitions
    private double yelPercent;
    private Mat blurredMat = new Mat();
    private Mat kernel = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);

        // Gets color specific values
        yelPercent = Core.countNonZero(yelMat);

        // Calculates the highest amount of pixels being covered on each side
        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (yelPercent >= 60) {
            isPole = true;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else {
            isPole = false;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        }

        // Memory cleanup
        blurredMat.release();
        yelMat.release();
        kernel.release();

        return input;
    }

    public boolean getPole() {
        return isPole;
    }

    public double getPercent() {
        return yelPercent;
    }
}
