package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.old.subsystem.Subsystem;

abstract public class RobotOpMode extends LinearOpMode {
    public void addSubsystems(Subsystem... subsystems) {
        for (Subsystem subsystem : subsystems) {
            subsystem.setup();
        }
    }

    // Waits until at least one device or OpMode is not busy
    public void blockOn(DcMotor... motors) {
        while (opModeIsActive() && isBusy(motors)) {
            idle();
        }
    }

    public boolean isBusy(DcMotor... motors) {
        for (DcMotor motor : motors) {
            if (!motor.isBusy()) {
                return false;
            }
        }
        return true;
    }
}
