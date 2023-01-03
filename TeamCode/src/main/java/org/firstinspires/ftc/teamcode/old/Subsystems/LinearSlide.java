package org.firstinspires.ftc.teamcode.old.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.old.Util.Maths;

public class LinearSlide {
    private final Telemetry telemetry;
    public final DcMotor slideMotor;
    public final Servo gripper1Servo;
    public final Servo gripper2Servo;
    public final Gamepad gamepad;
    public final int minHeight;
    public final int maxHeight;
    public final double gripMin;
    public final double gripMax;
    private double gripPos = 1;
    private double height = 0;
    private int armMotorSteps =0;
    private boolean dpadpressed = false;
    private LinearOpMode linearOpMode;

    public LinearSlide(LinearOpMode linearOpMode, Telemetry telemetry, DcMotor slideMotor, Servo gripper1, Servo gripper2, Gamepad gamepad, int maxHeight, int minHeight, double gripMin, double gripMax){
        this.linearOpMode = linearOpMode;
        this.telemetry = telemetry;
        this.slideMotor = slideMotor;
        this.gripper1Servo = gripper1;
        this.gripper2Servo = gripper2;
        this.gamepad = gamepad;
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        this.gripMin = gripMin;
        this.gripMax = gripMax;
    }

    public void respondToGamepad() {
        slideRespondToGamepad();
        gripRespondToGamepad();
    }

    public void gripRespondToGamepad() {
        if (gamepad.right_trigger > 0.5) {
            gripPos = gripMin;
        } else if (gamepad.left_trigger > 0.5) {
            gripPos = gripMax;
        }

        gripPos = Maths.clamp(gripPos, gripMin, gripMax);

        gripper1Servo.setPosition(1 - gripPos);
        gripper2Servo.setPosition(gripPos);
    }

    public void slideRespondToGamepad(){
        if (gamepad.a) {
            armMotorSteps = 0;
        } else if (gamepad.b) {
            armMotorSteps = 86;
        } else if (gamepad.x) {
            armMotorSteps = 270;
        } else if (gamepad.y) {
            armMotorSteps = 460;
        }

        // TODO: check if this sensitivity is right
        if (gamepad.dpad_up) {
            armMotorSteps += 4;
            dpadpressed = true;
        } else if (gamepad.dpad_down) {
            armMotorSteps -= 4;
            dpadpressed = true;
        } else if (dpadpressed) {
            armMotorSteps = slideMotor.getCurrentPosition();
            dpadpressed = false;
        }

        armMotorSteps = Maths.clamp(armMotorSteps, minHeight, maxHeight);

        slideMotor.setTargetPosition(armMotorSteps);
    }

    public void grab() {
        gripper1Servo.setPosition(1);
        gripper2Servo.setPosition(0);
    }

    public void ungrab() {
        gripper1Servo.setPosition(gripMin);
        gripper2Servo.setPosition(1-gripMin);
    }

    public void setSlide(int steps) {
        slideMotor.setTargetPosition(Maths.clamp(steps, minHeight, maxHeight));
        while (linearOpMode.opModeIsActive() && slideMotor.isBusy()) {
            linearOpMode.idle();
        }
    }
}
