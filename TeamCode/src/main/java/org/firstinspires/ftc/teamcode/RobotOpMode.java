package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

abstract public class RobotOpMode extends LinearOpMode {
    ArrayList<Subsystem> subsystems = new ArrayList<Subsystem>();
    
    public void addSubsystems(Subsystem... subsystems) {
        for (Subsystem s : subsystems) {
            s.setup();
            this.subsystems.add(s);
        }
    }

    abstract public void setup();

    abstract public void update();

    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            update();
            for (Subsystem subsystem : subsystems) {
                subsystem.update();
            }
            telemetry.update();
        }
    }

    // Waits until at least one device is not busy
    public void blockOn(DcMotor... motors) {
        while (isBusy(motors)) {
            idle();
        }
    }

    public boolean isBusy(DcMotor... motors) {
        if (!opModeIsActive()) {
            return false;
        }

        for (DcMotor motor : motors) {
            if (!motor.isBusy()) {
                return false;
            }
        }

        return true;
    }
}
