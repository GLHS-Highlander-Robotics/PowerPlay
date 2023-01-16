package org.firstinspires.ftc.teamcode;

public class Utils {
    public static int driveTicks(double inches) {
        return (int) Math.round(inches * 54);
    }

    public static int strafeTicks(double inches) {
        return (int) Math.round(inches * 56.25);
    }

    public static int clamp(int val, int min, int max) {
        if (val < min) {
            return min;
        } else if (val > max) {
            return max;
        }
        return val;
    }

    public static float clamp(float val, float min, float max) {
        if (val < min) {
            return min;
        } else if (val > max) {
            return max;
        }
        return val;
    }

    public static double clamp(double val, double min, double max) {
        if (val < min) {
            return min;
        } else if (val > max) {
            return max;
        }
        return val;
    }
}
