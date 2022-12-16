package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Util.Measure;

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

        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double rotate = gamepad.right_stick_x;
        if (Math.abs(-gamepad.left_stick_y) < 0.01) {
            forward = 0;
        }

        if (Math.abs(gamepad.left_stick_x) < 0.01) {
            strafe = 0;
        }

        if (Math.abs(gamepad.right_stick_x) < 0.01) {
            rotate = 0;
        }
        frontLeftMotor.setPower(forward + strafe + rotate);
        backLeftMotor.setPower(forward - strafe + rotate);
        frontRightMotor.setPower(forward + strafe - rotate);
        backRightMotor.setPower(forward - strafe - rotate);
    }

    public void drive(int leftMove, int rightMove, float speed) {

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos = leftMove;
        frPos = rightMove;
        blPos = leftMove;
        brPos = rightMove;


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

    public void drivein(double leftinches, int rightinches, float speed) {

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos = Measure.sTicks(leftinches);
        frPos = Measure.sTicks(rightinches);
        blPos = Measure.sTicks(leftinches);
        brPos = Measure.sTicks(rightinches);


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

    public void strafe(int move, float speed) {

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos = move;
        frPos = move;
        blPos = move * -1;
        brPos = move * -1;
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

    public void strafein(double inches, float speed) {

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos = Measure.sTicks(inches);
        frPos = Measure.sTicks(inches);
        blPos = Measure.sTicks(inches) * -1;
        brPos = Measure.sTicks(inches) * -1;
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
