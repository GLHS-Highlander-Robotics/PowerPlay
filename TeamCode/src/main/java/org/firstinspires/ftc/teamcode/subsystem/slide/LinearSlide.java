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

    public static double ACTUATOR_MIN = 0.4;
    public static double ACTUATOR_MAX = 0.65;

    public DcMotorEx slideMotor;
    public Servo leftGripper, rightGripper;
    public Servo linearActuator;

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

        linearActuator = hardwareMap.get(Servo.class, "linear_actuator");
    }

    public void setSlide(int steps) {
        armMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        slideMotor.setTargetPosition(armMotorSteps);
    }

    public void update() {
        // If motor is busy, run motor at max power.
        // If motor is not busy, hold at current position or stop at lowest height
        if (slideMotor.isBusy()) {
            slideMotor.setPower(MAX_POWER);
        } else {
            slideMotor.setPower(HOLD_POWER);
            if (slideMotor.getCurrentPosition() == MIN_HEIGHT) {
                slideMotor.setPower(MIN_POWER);
            }
        }
    }

    public void ungrab() {
        leftGripper.setPosition(GRIP_MIN);
        rightGripper.setPosition(GRIP_MAX);
    }

    public void grab() {
        leftGripper.setPosition(GRIP_MAX + 0.1);
        rightGripper.setPosition(GRIP_MIN - 0.1);
    }

    public void setLinearActuator(double steps) {
        linearActuator.setPosition(steps);
    }

    public void setActuatorDirection(Servo.Direction direction) {
        linearActuator.setDirection(direction);
    }

    public void grabAndRaise(int steps) {
        leftGripper.setPosition(GRIP_MAX + 0.1);
        rightGripper.setPosition(GRIP_MIN - 0.1);
//        while (leftGripper.getPosition() < GRIP_MAX - 0.1 || rightGripper.getPosition() > GRIP_MIN + 0.1) {
//            wait();
//        }
        armMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        slideMotor.setTargetPosition(armMotorSteps);
    }
}
