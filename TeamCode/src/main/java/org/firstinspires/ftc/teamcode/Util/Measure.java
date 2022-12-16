package org.firstinspires.ftc.teamcode.Util;

public class Measure {

    public static int dTicks(double inches) {
        return (int)Math.round(inches * 54);
    }

    public static int sTicks(double inches) {
        return (int)Math.round(inches * 56.25);
    }
}
