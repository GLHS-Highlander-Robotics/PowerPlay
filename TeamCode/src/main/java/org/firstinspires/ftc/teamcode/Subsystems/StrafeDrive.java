package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StrafeDrive {
    private final Telemetry telemetry;
    public final DcMotor frontLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor backLeftMotor;
    public final DcMotor backRightMotor;
    public final LinearOpMode linearOpMode;
    public final Gamepad gamepad;
    public int flPos = 0;
    public int frPos = 0;
    public int blPos = 0;
    public int brPos = 0;
    public final int FULLTURN = 1560;

    public StrafeDrive(LinearOpMode linearOpMode, Telemetry telemetry, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.gamepad = gamepad;
    }

    public void respondToGamepad() {
        float x = gamepad.left_stick_x;
        float y = -gamepad.left_stick_y;
        float turn = gamepad.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos / max + turn;
        double frontRightPower = power * sin / max - turn;
        double backLeftPower = power * sin / max + turn;
        double backRightPower = power * cos / max - turn;

        if (power + Math.abs(turn) > 1) {
            frontLeftPower /= power + turn;
            frontRightPower /= power + turn;
            backLeftPower /= power + turn;
            backRightPower /= power + turn;
        }

        frontLeftPower *= frontLeftPower * frontLeftPower;
        frontRightPower *= frontRightPower * frontRightPower;
        backLeftPower *= backLeftPower * backLeftPower;
        backRightPower *= backRightPower * backRightPower;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    public void respondToGamepad2() {

        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double rotate = gamepad.right_stick_x;

        frontLeftMotor.setPower(forward + strafe + rotate);
        backLeftMotor.setPower(forward - strafe + rotate);
        frontRightMotor.setPower(forward - strafe - rotate);
        backRightMotor.setPower(forward + strafe - rotate);
    }
    public void drive(int leftMove, int rightMove, float speed) {
        flPos += leftMove;
        frPos += rightMove;
        blPos += leftMove;
        brPos += rightMove;
        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(blPos);
        frontLeftMotor.setTargetPosition(flPos);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);

        while (linearOpMode.opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy()) {
            linearOpMode.idle();
        }
    }

    public void strafe(int move, float speed) {
        flPos += move;
        frPos -= move;
        blPos -= move;
        brPos -= move;
        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);

        while (linearOpMode.opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy()) {
            linearOpMode.idle();
        }
    }
}
