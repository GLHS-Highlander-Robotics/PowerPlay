package org.firstinspires.ftc.teamcode.subsystem.slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class LinearSlide {
    public static double MIN_POWER = 0;
    public static double HOLD_POWER = 0.1;
    public static double MAX_POWER = 1;

    public static int MIN_HEIGHT = 0;
    public static int LOW_HEIGHT = 1600;
    public static int MEDIUM_HEIGHT = 2760;
    public static int MAX_HEIGHT = 3870;
    public static int INCREMENT_STEPS = 20;

    public static double GRIP_MIN = 0;
    public static double GRIP_MAX = 0.6;

    public DcMotorEx slideMotor;
    public Servo leftGripper, rightGripper;

    private boolean gripperClosed = false;
    private boolean dPadPressed = false;
    private int armMotorSteps = 0;

    public LinearSlide(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "motor_slide");
        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        slideMotor.setTargetPosition(MIN_HEIGHT);
        slideMotor.setPower(MAX_POWER);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftGripper = hardwareMap.get(Servo.class, "grip1");
        rightGripper = hardwareMap.get(Servo.class, "grip2");
    }

    public void setSlide(int steps) {
        armMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        slideMotor.setTargetPosition(armMotorSteps);
    }

    public void ungrab() {
        leftGripper.setPosition(GRIP_MIN);
        rightGripper.setPosition(GRIP_MAX);
    }

    public void grab() {

    }
}
