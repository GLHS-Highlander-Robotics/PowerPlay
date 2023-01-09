package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;

public class StrafeDrive implements Subsystem {
    private final RobotOpMode opMode;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private int flPos = 0;
    private int frPos = 0;
    private int blPos = 0;
    private int brPos = 0;

    public StrafeDrive(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void setup() {
        frontLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_left");
        frontRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_right");
        backLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_left");
        backRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_right");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(int leftMove, int rightMove, float speed) {
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos = leftMove;
        frPos = rightMove;
        blPos = leftMove;
        brPos = rightMove;

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void driveInches(double leftInches, int rightInches, float speed) {
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos += Utils.driveTicks(leftInches);
        frPos += Utils.driveTicks(rightInches);
        blPos += Utils.driveTicks(leftInches);
        brPos += Utils.driveTicks(rightInches);

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void strafe(int move, float speed) {
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos = move;
        frPos = move;
        blPos = -move;
        brPos = -move;

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void strafeInches(double inches, float speed) {
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos += Utils.strafeTicks(inches);
        frPos += Utils.strafeTicks(inches);
        blPos += -Utils.strafeTicks(inches);
        brPos += -Utils.strafeTicks(inches);
        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void updateByGamepad() {
        double forward = -opMode.gamepad1.left_stick_y;
        double strafe = opMode.gamepad1.left_stick_x;
        double rotate = opMode.gamepad1.right_stick_x;

        if (Math.abs(-opMode.gamepad1.left_stick_y) < 0.01) {
            forward = 0;
        }
        if (Math.abs(opMode.gamepad1.left_stick_x) < 0.01) {
            strafe = 0;
        }
        if (Math.abs(opMode.gamepad1.right_stick_x) < 0.01) {
            rotate = 0;
        }

        frontLeftMotor.setPower(forward + strafe + rotate);
        backLeftMotor.setPower(forward - strafe + rotate);
        frontRightMotor.setPower(forward + strafe - rotate);
        backRightMotor.setPower(forward - strafe - rotate);
    }

    public void setModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    public void setPowers(float speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }
}
