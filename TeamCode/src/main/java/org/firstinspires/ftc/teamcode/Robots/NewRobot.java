package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;

public class NewRobot {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Hardware Configuration Names
    public static final String FRONT_LEFT_MOTOR_NAME = "motor_front_left";
    public static final String FRONT_RIGHT_MOTOR_NAME = "motor_front_right";
    public static final String BACK_LEFT_MOTOR_NAME = "motor_back_left";
    public static final String BACK_RIGHT_MOTOR_NAME = "motor_back_right";

    // Subsystems
    public final StrafeDrive strafeDrive;
    public final LinearOpMode linearOpMode;

    // Sensors
    public final Gamepad gamepad;

    public NewRobot(LinearOpMode linearOpMode, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.gamepad = gamepad;
        this.strafeDrive = initStrafeDrive();
    }
    
    public StrafeDrive initStrafeDrive() {
        DcMotor frontLeftMotor = (DcMotor) hardwareMap.get(FRONT_LEFT_MOTOR_NAME);
        DcMotor frontRightMotor = (DcMotor) hardwareMap.get(FRONT_RIGHT_MOTOR_NAME);
        DcMotor backLeftMotor = (DcMotor) hardwareMap.get(BACK_LEFT_MOTOR_NAME);
        DcMotor backRightMotor = (DcMotor) hardwareMap.get(BACK_RIGHT_MOTOR_NAME);

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        return new StrafeDrive(linearOpMode,
                telemetry,
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gamepad
        );
    }
}
