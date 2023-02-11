package org.firstinspires.ftc.teamcode.slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideConstants {
    // Minimum power of the arm motor
    public static final double MIN_POWER = 0;

    // Holding power of the arm motor (when it is holding a position)
    public static final double HOLD_POWER = 0.1;

    // Maximum power of the arm motor
    public static final double MAX_POWER = 1;

    // Height (in encoder ticks) that the arm motor should go to reach the floor
    public static final int MIN_HEIGHT = 0;

    // Height (in encoder ticks) that the arm motor should go to reach the low junction
    public static final int LOW_HEIGHT = 1600;

    // Height (in encoder ticks) that the arm motor should go to reach the medium junction
    public static final int MEDIUM_HEIGHT = 2760;

    // Height (in encoder ticks) that the arm motor should go to reach the high junction
    public static final int MAX_HEIGHT = 3870;

    // Number of encoder ticks the arm should go up when the player presses the DPAD
    public static final int INCREMENT_STEPS = 12;

    // Minimum servo grip position
    public static final double GRIP_MIN = 0;

    // Maximum servo grip position
    public static final double GRIP_MAX = 0.6;
}
