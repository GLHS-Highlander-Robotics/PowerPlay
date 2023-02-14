package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotOpMode;

public class LinearSlide extends Subsystem {
    // Useful Constants
    public static final double MIN_POWER = 0;
    public static final double HOLD_POWER = 0.1;
    public static final double MAX_POWER = 1;
    public static final int MIN_HEIGHT = 0;
    public static final int LOW_HEIGHT = 1600;
    public static final int MEDIUM_HEIGHT = 2760;
    public static final int MAX_HEIGHT = 3870;
    public static final int INCREMENT_STEPS = 20;
    public static final double GRIP_MIN = 0;
    public static final double GRIP_MAX = 0.6;

    // Hardware
    public DcMotorEx slideMotor;
    public Servo leftGripper, rightGripper;

    private boolean gripperClosed = false;
    private boolean dPadPressed = false;
    private int armMotorSteps = 0;

    public LinearSlide(RobotOpMode opMode) {
        super(opMode);
    }

    @Override
    public void setup() {
        // Initialize slide motor
        slideMotor = opMode.hardwareMap.get(DcMotorEx.class, "motor_slide");
        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        slideMotor.setTargetPosition(MIN_HEIGHT);
        slideMotor.setPower(MAX_POWER);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize gripper servos
        leftGripper = opMode.hardwareMap.get(Servo.class, "grip1");
        rightGripper = opMode.hardwareMap.get(Servo.class, "grip2");
    }

    public void updateTelemetry() {
        // Add some debug information
        opMode.telemetry.addData("Target arm motor steps:", armMotorSteps);
        opMode.telemetry.addData("Actual arm motor steps:", slideMotor.getCurrentPosition());
        opMode.telemetry.addData("Arm Power:", slideMotor.getPower());
        opMode.telemetry.addData("Arm Current (A):", slideMotor.getCurrent(CurrentUnit.AMPS));
        opMode.telemetry.addData("Arm ZeroPower:", slideMotor.getZeroPowerBehavior());
    }

    public void readGamepad() {
        // Set arm steps to predefined height with buttons
        if (opMode.gamepad2.a) {
            armMotorSteps = MIN_HEIGHT;
        } else if (opMode.gamepad2.b) {
            armMotorSteps = LOW_HEIGHT;
        } else if (opMode.gamepad2.x) {
            armMotorSteps = MEDIUM_HEIGHT;
        } else if (opMode.gamepad2.y) {
            armMotorSteps = MAX_HEIGHT;
        }

        // Increase arm steps by DPAD increments
        if (opMode.gamepad2.dpad_up) {
            armMotorSteps += INCREMENT_STEPS;
            dPadPressed = true;
        } else if (opMode.gamepad2.dpad_down) {
            armMotorSteps -= INCREMENT_STEPS;
            dPadPressed = true;
        } else if (dPadPressed) {
            armMotorSteps = slideMotor.getCurrentPosition();
            dPadPressed = false;
        }

        setSlide(armMotorSteps);

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

        if (opMode.gamepad1.right_trigger > 0.5) {
            gripperClosed = true;
        } else if (opMode.gamepad1.left_trigger > 0.5) {
            gripperClosed = false;
        }

        if (gripperClosed) {
            grab();
        } else {
            release();
        }
    }

    public void grab() {
        leftGripper.setPosition(GRIP_MAX + 0.1);
        rightGripper.setPosition(GRIP_MIN - 0.1);
    }

    public void grabAndWait() {
        grab();
        while (leftGripper.getPosition() < GRIP_MAX || rightGripper.getPosition() > GRIP_MIN) {
            opMode.idle();
        }
        opMode.sleep(300);
    }

    public void release() {
        leftGripper.setPosition(GRIP_MIN);
        rightGripper.setPosition(GRIP_MAX);
    }

    public void releaseAndWait() {
        release();
        while (leftGripper.getPosition() > GRIP_MIN || rightGripper.getPosition() < GRIP_MAX) {
            opMode.idle();
        }
    }

    public void setSlide(int steps) {
        armMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        slideMotor.setTargetPosition(armMotorSteps);
    }

    public void setSlideAndWait(int steps) {
        setSlide(steps);
        opMode.blockOn(slideMotor);
    }
}
