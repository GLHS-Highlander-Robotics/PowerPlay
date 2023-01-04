package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;

public class LinearSlide implements Subsystem {
    public DcMotor slideMotor;
    public Servo leftGripper, rightGripper;
    public final int minHeight;
    public final int maxHeight;
    public final double gripMin;
    public final double gripMax;
    private final RobotOpMode opMode;
    private double gripPos = 1;
    private boolean dPadPressed = false;
    private int armMotorSteps = 0;

    public LinearSlide(RobotOpMode opMode, int maxHeight, int minHeight, double gripMin, double gripMax) {
        this.opMode = opMode;
        this.maxHeight = maxHeight;
        this.minHeight = minHeight;
        this.gripMin = gripMin;
        this.gripMax = gripMax;
    }

    @Override
    public void setup() {
        slideMotor = opMode.hardwareMap.get(DcMotor.class, "motor_slide");
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // do we need this line?
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftGripper = opMode.hardwareMap.get(Servo.class, "grip1");
        rightGripper = opMode.hardwareMap.get(Servo.class, "grip2");
    }

    public void updateByGamepad() {
        // Arm
        if (opMode.gamepad1.a) {
            armMotorSteps = 0;
        } else if (opMode.gamepad1.b) {
            armMotorSteps = 86;
        } else if (opMode.gamepad1.x) {
            armMotorSteps = 270;
        } else if (opMode.gamepad1.y) {
            armMotorSteps = 460;
        }

        if (opMode.gamepad1.dpad_up) {
            armMotorSteps += 4;
            dPadPressed = true;
        } else if (opMode.gamepad1.dpad_down) {
            armMotorSteps -= 4;
            dPadPressed = true;
        } else if (dPadPressed) {
            armMotorSteps = slideMotor.getCurrentPosition();
            dPadPressed = false;
        }

        armMotorSteps = Utils.clamp(armMotorSteps, minHeight, maxHeight);

        slideMotor.setTargetPosition(armMotorSteps);

        // Gripper
        if (opMode.gamepad1.right_trigger > 0.5) {
            gripPos = gripMin;
        } else if (opMode.gamepad1.left_trigger > 0.5) {
            gripPos = gripMax;
        }

        gripPos = Utils.clamp(gripPos, gripMin, gripMax);

        leftGripper.setPosition(1 - gripPos);
        rightGripper.setPosition(gripPos);
    }

    public void grab() {
        leftGripper.setPosition(1);
        rightGripper.setPosition(0);
    }

    public void ungrab() {
        leftGripper.setPosition(gripMin);
        rightGripper.setPosition(1 - gripMin);
    }

    public void setSlide(int steps) {
        slideMotor.setTargetPosition(Utils.clamp(steps, minHeight, maxHeight));
        opMode.blockOn(slideMotor);
    }
}
