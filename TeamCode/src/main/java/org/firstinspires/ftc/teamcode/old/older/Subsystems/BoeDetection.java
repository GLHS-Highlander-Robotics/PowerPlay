package org.firstinspires.ftc.teamcode.old.older.Subsystems;

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

    public enum Distance {
        CLOSE,
        MID,
        FAR,
        UNSEEN
    }

    // TOPLEFT anchor point for the bounding box
    private static final int X = 55;
    private static final int Y = 120;
    private static final int STEP = 10;
    private static final int WIDTH = 25;
    private static final int HEIGHT = 15;
    private static final Point FAR_TOPLEFT_ANCHOR_POINT = new Point(X, Y);
    private static final Point CLOSE_TOPLEFT_ANCHOR_POINT = new Point(X - (2 * STEP), Y - (2 * STEP));

    // Width and height for the bounding box
    public static int FAR_REGION_WIDTH = WIDTH;
    public static int FAR_REGION_HEIGHT = HEIGHT;
    public static int MID_REGION_WIDTH = WIDTH + STEP + STEP;
    public static int MID_REGION_HEIGHT = HEIGHT + STEP + STEP;
    public static int CLOSE_REGION_WIDTH = WIDTH + STEP + STEP + STEP + STEP;
    public static int CLOSE_REGION_HEIGHT = HEIGHT + STEP + STEP + STEP + STEP;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_red_bounds = new Scalar(100, 0, 0, 255),
            upper_red_bounds = new Scalar(255, 150, 150, 255),
            lower_blue_bounds = new Scalar(0, 0, 100, 255),
            upper_blue_bounds = new Scalar(150, 150, 255, 255);

    // Color definitions
    private final Scalar
            RED = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255),
            BLACK = new Scalar(0, 0, 0);

    // Percent and mat definitions
    private double redPercentC, bluePercentC, redPercentM, bluePercentM, redPercentF, bluePercentF;
    private final Mat redMatC = new Mat();
    private final Mat blueMatC = new Mat();
    private Mat redMatF = new Mat();
    private Mat blueMatF = new Mat();
    private Mat redMatM = new Mat();
    private Mat blueMatM = new Mat();
    private Mat blurredMatClose = new Mat();
    private Mat kernel = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point((2 * STEP), (2 * STEP));
    Point sleeve_pointB = new Point(
            (2 * STEP) + FAR_REGION_WIDTH,
            (2 * STEP) + FAR_REGION_HEIGHT);

    Point sleeve_pointC = new Point(STEP, STEP);
    Point sleeve_pointD = new Point(
            STEP + MID_REGION_WIDTH,
            STEP + MID_REGION_HEIGHT);

    Point sleeve_pointE = new Point(
            CLOSE_TOPLEFT_ANCHOR_POINT.x,
            CLOSE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointF = new Point(
            CLOSE_TOPLEFT_ANCHOR_POINT.x + CLOSE_REGION_WIDTH,
            CLOSE_TOPLEFT_ANCHOR_POINT.y + CLOSE_REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile Cone cone = Cone.UNDECIDED;
    private volatile Distance distance = Distance.UNSEEN;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMatClose, new Size(5, 5));
        blurredMatClose = blurredMatClose.submat(new Rect(sleeve_pointE, sleeve_pointF));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatClose, blurredMatClose, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMatClose, lower_red_bounds, upper_red_bounds, redMatC);
        Core.inRange(blurredMatClose, lower_blue_bounds, upper_blue_bounds, blueMatC);

        redMatM = redMatC.submat(new Rect(sleeve_pointC, sleeve_pointD));
        blueMatM = blueMatC.submat(new Rect(sleeve_pointC, sleeve_pointD));

        redMatF = redMatC.submat(new Rect(sleeve_pointA, sleeve_pointB));
        blueMatF = blueMatC.submat(new Rect(sleeve_pointA, sleeve_pointB));


        // Gets color specific values
        redPercentC = Core.countNonZero(redMatC);
        bluePercentC = Core.countNonZero(blueMatC);

        redPercentM = Core.countNonZero(redMatM);
        bluePercentM = Core.countNonZero(blueMatM);

        redPercentF = Core.countNonZero(redMatF);
        bluePercentF = Core.countNonZero(blueMatF);


        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(redPercentF, bluePercentF);

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == redPercentF) {
            if (redPercentC >= 150) {
                cone = Cone.RED;
                distance = Distance.CLOSE;
            } else if (redPercentM >= 100) {
                cone = Cone.RED;
                distance = Distance.MID;
            } else if (redPercentF >= 50) {
                cone = Cone.RED;
                distance = Distance.FAR;
            } else {
                cone = Cone.UNDECIDED;
                distance = Distance.UNSEEN;
            }
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    RED,
                    2
            );
        } else if (maxPercent == bluePercentC && bluePercentC >= 50) {
            cone = Cone.BLUE;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BLUE,
                    2
            );
        } else {
            cone = Cone.UNDECIDED;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BLACK,
                    2
            );
        }

        // Memory cleanup
        redMatC.release();
        blueMatC.release();
        kernel.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public Cone getPosition() {
        return cone;
    }
}