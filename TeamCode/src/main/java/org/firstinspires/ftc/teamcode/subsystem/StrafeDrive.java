package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotOpMode;

public class StrafeDrive extends Subsystem {
    //Constants
    static final double WHEEL_DIAMETER = 3.0;
    static final double TICKS_PER_ROTATION = 28 * 5 * 4;
    static final double HIGH_POWER = 0.85;
    static final double LOW_POWER = 0.35;
    static final double DEAD_ZONE_P1 = 0.05;
    static final double DEAD_ZONE_P2 = 0.05;
    //Variables
    public IMU imu;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private int flPos = 0;
    private int frPos = 0;
    private int blPos = 0;
    private int brPos = 0;
    public double botHeading;
    private double limiter = HIGH_POWER;
    private boolean field = false;

    //Copypasted constants/variables
    static final double HEADING_THRESHOLD = 1.0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;

    //Constructor
    public StrafeDrive(RobotOpMode opMode) {
        super(opMode);
    }

    @Override
    public void setup() {
        frontLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_left");
        frontRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_right");
        backLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_left");
        backRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_right");
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static int driveTicks(double inches) {
//        return (int) Math.round(inches * WHEEL_RADIUS * TICKS_PER_ROTATION * Math.PI);
        return (int) Math.round(inches / (WHEEL_DIAMETER * Math.PI) * TICKS_PER_ROTATION);
    }


    public static int strafeTicks(double inches) {
        return (int) Math.round(inches / (WHEEL_DIAMETER * Math.PI) * TICKS_PER_ROTATION);
    }

    //Set all motor modes
    public void setModes(DcMotor.RunMode mode) {
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
    }

    //Set all motor powers
    public void setPowers(float speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }

    //Set motor targets to their variables
    public void setMotorTargets() {
        backLeftMotor.setTargetPosition(blPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);
        frontRightMotor.setTargetPosition(frPos);
    }

    //For teleop
    public void driveBot(double forward, double strafe, double rotate) {
        frontLeftMotor.setPower((forward + strafe + rotate));
        backLeftMotor.setPower((forward - strafe + rotate));
        frontRightMotor.setPower((forward + strafe - rotate));
        backRightMotor.setPower((forward - strafe - rotate));
    }

    //Updates bot heading
    public void updateHeadingDeg() {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void updateHeadingRad() {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    //Drive in ticks
    public void drive(int leftMove, int rightMove, float speed) {

        blPos = backLeftMotor.getCurrentPosition() + leftMove;
        brPos = backRightMotor.getCurrentPosition() + rightMove;
        flPos = frontLeftMotor.getCurrentPosition() + leftMove;
        frPos = frontRightMotor.getCurrentPosition() + rightMove;

        setMotorTargets();

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    //Strafe in ticks
    public void strafe(int move, float speed) {

        blPos = backLeftMotor.getCurrentPosition() - move;
        brPos = backRightMotor.getCurrentPosition() - move;
        flPos = frontLeftMotor.getCurrentPosition() + move;
        frPos = frontRightMotor.getCurrentPosition() + move;

        setMotorTargets();

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    //Ticks to Inches
    public void strafeInches(double inches, float speed) {
        strafe(strafeTicks(inches), speed);
    }

    public void driveInches(double leftInches, double rightInches, float speed) {
        drive(driveTicks(leftInches), driveTicks(rightInches), speed);
    }

    //Gyro turn
    public void turnToAbs(double target, double power) {
        updateHeadingDeg();
        while (botHeading < target - 2 || botHeading > target + 2) {
            driveBot(0, 0, power);

            updateHeadingDeg();
            opMode.telemetry.addData("Heading: ", botHeading);
            opMode.telemetry.update();
        }
        setPowers(0);
    }

    public void turnToRel(double reltarg, double power) {
        updateHeadingDeg();
        double target = botHeading + reltarg;
        if (target > 180) {
            target -= 360;
        }
        if (target < -180) {
            target += 360;
        }
        turnToAbs(target, power);
    }

    public void rotateAndMove(double seconds, double rotateTarget, double forwardPower, double strafePower, double rotatePower) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double fieldForward;
        double fieldStrafe;
        updateHeadingDeg();
        while (timer.time() < seconds && (botHeading < rotateTarget - 2 || botHeading > rotateTarget + 2)) {
            updateHeadingRad();
            fieldForward = strafePower * Math.sin(-botHeading) + forwardPower * Math.cos(-botHeading);
            fieldStrafe = strafePower * Math.cos(-botHeading) - forwardPower * Math.sin(-botHeading);
            updateHeadingDeg();

            driveBot(fieldForward, fieldStrafe, rotatePower);
            opMode.telemetry.addData("Heading: ", botHeading);
            opMode.telemetry.update();

        }
        while (timer.time() < seconds) {

            updateHeadingRad();
            fieldForward = strafePower * Math.sin(-botHeading) + forwardPower * Math.cos(-botHeading);
            fieldStrafe = strafePower * Math.cos(-botHeading) - forwardPower * Math.sin(-botHeading);
            driveBot(fieldForward, fieldStrafe, 0);
            opMode.telemetry.addData("Heading: ", botHeading);
            opMode.telemetry.addData("Time: ", timer.time());
            opMode.telemetry.update();
        }
        setPowers(0);
    }

    //Telemetry to Gamepad
    public void updateTelemetry() {
        opMode.telemetry.addData("Limiter: ", limiter);
        opMode.telemetry.addData("Heading: ", botHeading);
        opMode.telemetry.addData("Field Centric?: ", field);
        opMode.telemetry.addData("Back Left Power: ", backLeftMotor.getPower());
        opMode.telemetry.addData("Back Right Power: ", backRightMotor.getPower());
        opMode.telemetry.addData("Front Left Power: ", frontLeftMotor.getPower());
        opMode.telemetry.addData("Front Right Power: ", frontRightMotor.getPower());
    }


    // 2 Player TeleOp
    public void updateByTwoGamepads() {
        double forward;
        double strafe;
        double rotate;
        double fieldForward;
        double fieldStrafe;

        if (opMode.gamepad1.a) {
            limiter = HIGH_POWER;
        } else if (opMode.gamepad1.b) {
            limiter = LOW_POWER;
        }
        if (opMode.gamepad1.x) {
            imu.resetYaw();
            field = true;
        } else if (opMode.gamepad1.y) {
            field = false;
        }

        forward = -opMode.gamepad1.left_stick_y * limiter;
        strafe = opMode.gamepad1.left_stick_x * limiter;
        rotate = opMode.gamepad1.right_stick_x * limiter;

        if (Math.abs(-opMode.gamepad1.left_stick_y) < DEAD_ZONE_P1) {
            forward = -opMode.gamepad2.left_stick_y * LOW_POWER;
            if (Math.abs(opMode.gamepad2.left_stick_y) < DEAD_ZONE_P2) {
                forward = 0;
            }
        }
        if (Math.abs(opMode.gamepad1.left_stick_x) < DEAD_ZONE_P1) {
            strafe = opMode.gamepad2.left_stick_x * LOW_POWER;
            if (Math.abs(opMode.gamepad2.left_stick_x) < DEAD_ZONE_P2) {
                strafe = 0;
            }
        }
        if (Math.abs(opMode.gamepad1.right_stick_x) < DEAD_ZONE_P1) {
            rotate = opMode.gamepad2.right_stick_x * LOW_POWER;
            if (Math.abs(opMode.gamepad2.right_stick_x) < DEAD_ZONE_P2) {
                rotate = 0;
            }
        }
        fieldForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        fieldStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);

        updateHeadingRad();
        if (!field) {
            driveBot(forward, strafe, rotate);
        } else {
            driveBot(fieldForward, fieldStrafe, rotate);
        }


    }

    // 1 Player TeleOp
    public void updateByGamepad() {
        double forward;
        double strafe;
        double rotate;

        forward = -opMode.gamepad1.left_stick_y * limiter;
        strafe = opMode.gamepad1.left_stick_x * limiter;
        rotate = opMode.gamepad1.right_stick_x * limiter;
        if (Math.abs(-opMode.gamepad1.left_stick_y) < 0.01) {
            forward = 0;
        }
        if (Math.abs(opMode.gamepad1.left_stick_x) < 0.01) {
            strafe = 0;
        }
        if (Math.abs(opMode.gamepad1.right_stick_x) < 0.01) {
            rotate = 0;
        }

        driveBot(forward, strafe, rotate);
    }


    //Copypasted code
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the opmode is still active


        // Determine new target position, and pass to motor controller
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flPos = (driveTicks(distance));
        frPos = (driveTicks(distance));
        brPos = (driveTicks(distance));
        blPos = (driveTicks(distance));


        // Set Target FIRST, then turn on RUN_TO_POSITION
        setMotorTargets();

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (opMode.isBusy(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(driveSpeed, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(true);
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while ((Math.abs(headingError) > HEADING_THRESHOLD) && opMode.opModeIsActive()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            turnSpeed = 0.5;

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(true);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (holdTimer.time() < holdTime) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        backLeftMotor.setPower(leftSpeed);
        backRightMotor.setPower(rightSpeed);

    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            opMode.telemetry.addData("Motion", "Drive Straight");
            opMode.telemetry.addData("Target Pos L:R", "%7d:%7d", flPos, frPos);
            opMode.telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition());
        } else {
            opMode.telemetry.addData("Motion", "Turning");
        }

        opMode.telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        opMode.telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        opMode.telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        opMode.telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {

        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }


}
