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
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftGripper = opMode.hardwareMap.get(Servo.class, "grip1");
        rightGripper = opMode.hardwareMap.get(Servo.class, "grip2");

    }

    public void updateByGamepad() {
        // Arm
        if (opMode.gamepad1.a) {
            armMotorSteps = 0;
        } else if (opMode.gamepad1.b) {
            armMotorSteps = 425;
        } else if (opMode.gamepad1.x) {
            armMotorSteps = 750;
        } else if (opMode.gamepad1.y) {
            armMotorSteps = 1150;
        }

        if (opMode.gamepad1.dpad_up) {
            armMotorSteps += 2;
            dPadPressed = true;
        } else if (opMode.gamepad1.dpad_down) {
            armMotorSteps -= 2;
            dPadPressed = true;
        } else if (dPadPressed) {
            armMotorSteps = slideMotor.getCurrentPosition();
            dPadPressed = false;
        }

        // TODO: find the correct minHeight and maxHeight since we want to safely limit the motor

//        armMotorSteps = Utils.clamp(armMotorSteps, minHeight, maxHeight);

        slideMotor.setTargetPosition(armMotorSteps);

        // Gripper
        if (opMode.gamepad1.right_trigger > 0.5) {
            gripPos = 0;
        } else if (opMode.gamepad1.left_trigger > 0.5) {
            gripPos = 0.5;
        }

        //gripPos = Utils.clamp(gripPos, gripMin, gripMax);

        leftGripper.setPosition(gripPos);
        rightGripper.setPosition(gripPos - 0.5);

        opMode.telemetry.addData("arm motor steps:", armMotorSteps);
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
//        slideMotor.setTargetPosition(Utils.clamp(steps, minHeight, maxHeight));
        armMotorSteps+=steps;
        slideMotor.setTargetPosition(armMotorSteps);

    }
}
