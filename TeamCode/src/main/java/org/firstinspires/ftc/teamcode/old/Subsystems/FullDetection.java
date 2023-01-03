package org.firstinspires.ftc.teamcode.old.Subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FullDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(55, 120);
    private static Point POLE_TOPLEFT_ANCHOR_POINT = new Point(75, 100);

    // Width and height for the bounding box
    public static int SLEEVE_REGION_WIDTH = 25;
    public static int SLEEVE_REGION_HEIGHT = 15;
    public static int POLE_REGION_WIDTH = 25;
    public static int POLE_REGION_HEIGHT = 15;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds  = new Scalar(150, 150, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 150, 255),
            lower_cyan_bounds    = new Scalar(0, 120, 120, 255),
            upper_cyan_bounds    = new Scalar(150, 255, 255, 255),
            lower_magenta_bounds = new Scalar(120, 0, 120, 255),
            upper_magenta_bounds = new Scalar(255, 170, 255, 255),
            lower_pole_bounds = new Scalar(150, 150, 0, 255),
            upper_pole_bounds = new Scalar(255, 255, 150, 255);

    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Percent and mat definitions
    private double yelPercentS, cyaPercentS, magPercentS, yelPercentP;
    private Mat yelMatS = new Mat(), cyaMatS = new Mat(), magMatS = new Mat(), yelMatP = new Mat(), blurredMat = new Mat(), sleeveMat = new Mat(), poleMat = new Mat(), kernel = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + SLEEVE_REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + SLEEVE_REGION_HEIGHT);
    Point pole_pointA = new Point(
            POLE_TOPLEFT_ANCHOR_POINT.x,
            POLE_TOPLEFT_ANCHOR_POINT.y);
    Point pole_pointB = new Point(
            POLE_TOPLEFT_ANCHOR_POINT.x + POLE_REGION_WIDTH,
            POLE_TOPLEFT_ANCHOR_POINT.y + POLE_REGION_HEIGHT);

    // Running variable storing the parking position and pole detection
    private volatile ParkingPosition position = ParkingPosition.LEFT;
    private volatile boolean isPole = false;
    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        sleeveMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));
        poleMat = blurredMat.submat(new Rect(pole_pointA, pole_pointB));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(sleeveMat, sleeveMat, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(poleMat, poleMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(sleeveMat, lower_yellow_bounds, upper_yellow_bounds, yelMatS);
        Core.inRange(sleeveMat, lower_cyan_bounds, upper_cyan_bounds, cyaMatS);
        Core.inRange(sleeveMat, lower_magenta_bounds, upper_magenta_bounds, magMatS);
        Core.inRange(poleMat, lower_pole_bounds, upper_pole_bounds, yelMatP);

        // Gets color specific values
        yelPercentS = Core.countNonZero(yelMatS);
        cyaPercentS = Core.countNonZero(cyaMatS);
        magPercentS = Core.countNonZero(magMatS);
        yelPercentP = Core.countNonZero(yelMatP);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(yelPercentS, Math.max(cyaPercentS, magPercentS));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == yelPercentS) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == cyaPercentS) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else if (maxPercent == magPercentS) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        }
        if (yelPercentP >= 60) {
            isPole = true;
            Imgproc.rectangle(
                    input,
                    pole_pointA,
                    pole_pointB,
                    YELLOW,
                    2
            );
        } else {
            isPole = false;
            Imgproc.rectangle(
                    input,
                    pole_pointA,
                    pole_pointB,
                    CYAN,
                    2
            );
        }
        // Memory cleanup
        blurredMat.release();
        yelMatS.release();
        cyaMatS.release();
        magMatS.release();
        yelMatP.release();
        kernel.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
    public boolean getPole() {
        return isPole;
    }
    public double getPercent() {
        return yelPercentP;
    }
}
