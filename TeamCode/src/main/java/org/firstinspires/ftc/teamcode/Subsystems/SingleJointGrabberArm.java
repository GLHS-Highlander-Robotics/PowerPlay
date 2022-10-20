package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SingleJointGrabberArm {
    private final Telemetry telemetry;

    public final DcMotor armMotor;
    public final Servo grabber1Servo;
    public final Servo grabber2Servo;

    public final Gamepad gamepad;

    public SingleJointGrabberArm(Telemetry telemetry, DcMotor armMotor, Servo grabber1Servo, Servo grabber2Servo, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.armMotor = armMotor;
        this.grabber1Servo = grabber1Servo;
        this.grabber2Servo = grabber2Servo;
        this.gamepad = gamepad;
    }

    public void respondToGamepad() {

    }
}