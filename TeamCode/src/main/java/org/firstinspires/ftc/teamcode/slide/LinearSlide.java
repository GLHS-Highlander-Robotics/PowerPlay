package org.firstinspires.ftc.teamcode.slide;

import static org.firstinspires.ftc.teamcode.slide.SlideConstants.GRIP_MAX;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.GRIP_MIN;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.HOLD_POWER;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.INCREMENT_STEPS;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.LOW_HEIGHT;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.MAX_POWER;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.MEDIUM_HEIGHT;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.slide.SlideConstants.MIN_POWER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LinearSlide {
    public DcMotorEx slideMotor;
    public Servo leftGripper, rightGripper;

    private boolean gripperClosed = false;
    private boolean dPadPressed = false;
    private int targetArmMotorSteps = 0;
    private final LinearOpMode opMode;

    public LinearSlide(LinearOpMode opMode) {
        this.opMode = opMode;

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
        opMode.telemetry.addData("Target arm motor steps:", targetArmMotorSteps);
        opMode.telemetry.addData("Actual arm motor steps:", slideMotor.getCurrentPosition());
        opMode.telemetry.addData("Arm Power:", slideMotor.getPower());
        opMode.telemetry.addData("Arm Current (A):", slideMotor.getCurrent(CurrentUnit.AMPS));
        opMode.telemetry.addData("Arm ZeroPower:", slideMotor.getZeroPowerBehavior());
    }

    public void readGamepad() {
        // Set arm steps to predefined height with buttons
        if (opMode.gamepad2.a) {
            targetArmMotorSteps = MIN_HEIGHT;
        } else if (opMode.gamepad2.b) {
            targetArmMotorSteps = LOW_HEIGHT;
        } else if (opMode.gamepad2.x) {
            targetArmMotorSteps = MEDIUM_HEIGHT;
        } else if (opMode.gamepad2.y) {
            targetArmMotorSteps = MAX_HEIGHT;
        }

        // Increase arm steps by DPAD increments
        if (opMode.gamepad2.dpad_up) {
            targetArmMotorSteps += INCREMENT_STEPS;
            dPadPressed = true;
        } else if (opMode.gamepad2.dpad_down) {
            targetArmMotorSteps -= INCREMENT_STEPS;
            dPadPressed = true;
        } else if (dPadPressed) {
            targetArmMotorSteps = slideMotor.getCurrentPosition();
            dPadPressed = false;
        }

        setSlide(targetArmMotorSteps);

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

    public int getTargetMotorSteps() {
        return targetArmMotorSteps;
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
        targetArmMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        slideMotor.setTargetPosition(targetArmMotorSteps);
    }

    public void setSlideAndWait(int steps) {
        setSlide(steps);
        //opMode.blockOn(slideMotor);
    }
}
