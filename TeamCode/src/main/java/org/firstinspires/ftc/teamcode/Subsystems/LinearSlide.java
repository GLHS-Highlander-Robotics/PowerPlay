package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;

public class LinearSlide implements Subsystem {
    public final int minHeight;
    public final int maxHeight;
    public final double maxPower = 1;
    public final double gripMin;
    public final double gripMax;
    private final RobotOpMode opMode;
    public DcMotor slideMotor;
    public Servo leftGripper, rightGripper;
    private double gripPos = 0;
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
        slideMotor.setPower(maxPower);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftGripper = opMode.hardwareMap.get(Servo.class, "grip1");
        rightGripper = opMode.hardwareMap.get(Servo.class, "grip2");
    }

    public void updateByTwoGamepads() {
        // Arm
        if (opMode.gamepad2.a) {
            armMotorSteps = 0;
        } else if (opMode.gamepad2.b) {
            armMotorSteps = 1600;
        } else if (opMode.gamepad2.x) {
            armMotorSteps = 2760;
        } else if (opMode.gamepad2.y) {
            armMotorSteps = 3870;
        }


        if (opMode.gamepad2.dpad_up) {
            armMotorSteps += 12;
            dPadPressed = true;
        } else if (opMode.gamepad2.dpad_down) {
            armMotorSteps -= 12;
            dPadPressed = true;
        } else if (dPadPressed) {
            armMotorSteps = slideMotor.getCurrentPosition();
            dPadPressed = false;
        }

        // TODO: find the correct minHeight and maxHeight since we want to safely limit the motor

        armMotorSteps = Utils.clamp(armMotorSteps, -20, 4500);

        slideMotor.setTargetPosition(armMotorSteps);

        if (!slideMotor.isBusy()) {
            slideMotor.setPower(0.1);
            if (slideMotor.getCurrentPosition() == 0) {
                slideMotor.setPower(0);
            }
        } else {
            slideMotor.setPower(maxPower);
        }

        // Gripper
        if (opMode.gamepad1.right_trigger > 0.5) {
            //close
            gripPos = 1;
        } else if (opMode.gamepad1.left_trigger > 0.5) {
            //open
            gripPos = 0;
        }

        //gripPos = Utils.clamp(gripPos, gripMin, gripMax);
        //right is left, left is right
        if (gripPos == 0) {
            leftGripper.setPosition(0);
            rightGripper.setPosition(0.6);
//            rightGripper.setPosition(1 - gripPos);

        }
        if (gripPos == 1) {
            leftGripper.setPosition(0.6);
            rightGripper.setPosition(0);
//            rightGripper.setPosition(1 - gripPos);

        }

        opMode.telemetry.addData("target arm motor steps:", armMotorSteps);
        opMode.telemetry.addData("actual arm motor steps:", slideMotor.getCurrentPosition());
        opMode.telemetry.addData("Power:", slideMotor.getPower());
        opMode.telemetry.addData("ZeroPower:", slideMotor.getZeroPowerBehavior());
    }
    
    public void updateByGamepad() {
        // Arm
        if (opMode.gamepad1.a) {
            armMotorSteps = 0;
        } else if (opMode.gamepad1.b) {
            armMotorSteps = 1500;
        } else if (opMode.gamepad1.x) {
            armMotorSteps = 2900;
        } else if (opMode.gamepad1.y) {
            armMotorSteps = 4000;
        }


        if (opMode.gamepad1.dpad_up) {
            armMotorSteps += 12;
            dPadPressed = true;
        } else if (opMode.gamepad1.dpad_down) {
            armMotorSteps -= 12;
            dPadPressed = true;
        } else if (dPadPressed) {
            armMotorSteps = slideMotor.getCurrentPosition();
            dPadPressed = false;
        }

        armMotorSteps = Utils.clamp(armMotorSteps, -10, 4500);

        slideMotor.setTargetPosition(armMotorSteps);

        if (!slideMotor.isBusy()) {
            slideMotor.setPower(0.1);
        } else {
            slideMotor.setPower(maxPower);
        }

        // Gripper
        if (opMode.gamepad1.right_trigger > 0.5) {
            //close
            gripPos = 1;
        } else if (opMode.gamepad1.left_trigger > 0.5) {
            //open
            gripPos = 0;
        }

        //gripPos = Utils.clamp(gripPos, gripMin, gripMax);
        //right is left, left is right
        if (gripPos == 0) {
            leftGripper.setPosition(0);
            rightGripper.setPosition(0.6);
//            rightGripper.setPosition(1 - gripPos);
        }
        if (gripPos == 1) {
            leftGripper.setPosition(0.6);
            rightGripper.setPosition(0);
//            rightGripper.setPosition(1 - gripPos);

        }

        opMode.telemetry.addData("target arm motor steps:", armMotorSteps);
        opMode.telemetry.addData("actual arm motor steps:", slideMotor.getCurrentPosition());
    }


    public void grab() {
        leftGripper.setPosition(0.6);
        rightGripper.setPosition(-0.1);
        while (leftGripper.getPosition() != 0.6 || rightGripper.getPosition() != 0) {
            opMode.idle();
        }
        opMode.sleep(300);
    }

    public void ungrab() {
        leftGripper.setPosition(0);
        rightGripper.setPosition(0.6);
        while (leftGripper.getPosition() != 0 || rightGripper.getPosition() != 0.6) {
            opMode.idle();
        }
    }

    public void setSlide(int steps, boolean wait) {
//        slideMotor.setTargetPosition(Utils.clamp(steps, minHeight, maxHeight));
        armMotorSteps = steps;
        slideMotor.setTargetPosition(armMotorSteps);

        if (wait) {
            opMode.blockOn(slideMotor);
        }
        // repeated code just put here for testing, refactor later
        opMode.telemetry.addData("target arm motor steps:", armMotorSteps);
        opMode.telemetry.addData("actual arm motor steps:", slideMotor.getCurrentPosition());
    }
}
