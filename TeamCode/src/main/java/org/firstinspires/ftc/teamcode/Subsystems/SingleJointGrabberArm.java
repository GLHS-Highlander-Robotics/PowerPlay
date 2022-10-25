package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SingleJointGrabberArm {
    private final Telemetry telemetry;
    public final DcMotor armMotor;
    public final Servo gripper1Servo;
    public final Servo gripper2Servo;
    public final Gamepad gamepad;
    public final float gripMin;
    public final int gripMax;
    private int tp = 0;
    private float gripPos = 1;
    private boolean dPadPressed = false;

    public SingleJointGrabberArm(Telemetry telemetry, DcMotor armMotor, Servo gripper1Servo, Servo gripper2Servo, Gamepad gamepad, float gripMin, int gripMax) {
        this.telemetry = telemetry;
        this.armMotor = armMotor;
        this.gripper1Servo = gripper1Servo;
        this.gripper2Servo = gripper2Servo;
        this.gamepad = gamepad;
        this.gripMin = gripMin;
        this.gripMax = gripMax;
    }

    public void respondToGamepad() {
        armRespondToGamepad();
        gripRespondToGamepad();
    }

    public void gripRespondToGamepad() {
        if (gamepad.right_trigger > 0.5) {
            gripPos = gripMin;
        } else if (gamepad.left_trigger > 0.5) {
            gripPos = gripMax;
        } else if (gamepad.right_bumper) {
            gripPos -= 0.1;
            sleep(200);
        } else if (gamepad.left_bumper) {
            gripPos += 0.1;
            sleep(200);
        }

        if (gripPos < gripMin) {
            gripPos = gripMin;
        } else if (gripPos > gripMax) {
            gripPos = gripMax;
        }

        gripper1Servo.setPosition(1 - gripPos);
        gripper2Servo.setPosition(gripPos);
    }

    public void armRespondToGamepad() {
        if (gamepad.a) {
            tp = 0;
        } else if (gamepad.b) {
            tp = 86;
        } else if (gamepad.x) {
            tp = 290;
        } else if (gamepad.y) {
            tp = 399;
        }

        if (gamepad.dpad_up) {
            tp += 4;
            dPadPressed = true;
        } else if (gamepad.dpad_down) {
            tp -= 4;
            dPadPressed = true;
        } else if (dPadPressed) {
            tp = armMotor.getCurrentPosition();
            dPadPressed = false;
        }

        if (tp < 0) {
            tp = 0;
        } else if (tp > 400) {
            tp = 400;
        }

        armMotor.setTargetPosition(tp);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}