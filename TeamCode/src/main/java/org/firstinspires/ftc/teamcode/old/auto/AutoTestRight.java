package org.firstinspires.ftc.teamcode.old.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.old.RobotOpMode;
import org.firstinspires.ftc.teamcode.old.pipeline.PoleDetectionOld;
import org.firstinspires.ftc.teamcode.old.pipeline.SleeveDetectionNewOld;
import org.firstinspires.ftc.teamcode.old.pipeline.SleeveDetectionNewOld.ParkingPosition;
import org.firstinspires.ftc.teamcode.old.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.old.subsystem.StrafeDrive;
import org.firstinspires.ftc.teamcode.old.subsystem.Webcam;

@Disabled
@Autonomous(name = "Autonomous Right")
public class AutoTestRight extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this);
    private final SleeveDetectionNewOld sleeve = new SleeveDetectionNewOld();
    private final PoleDetectionOld pole = new PoleDetectionOld();
    private final Webcam cam = new Webcam(this, "Webcam 1", sleeve);

    @Override
    public void runOpMode() {
        addSubsystems(drive, slide, cam);

        // Put motors in encoder mode
        drive.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.release();

        drive.imu.resetYaw();
        //You need to have wait for start or else bad things happen
        waitForStart();
        drive.imu.resetYaw();
        ParkingPosition position = sleeve.getPosition();
        slide.grab();
        slide.setSlide(200);
        drive.driveInches(2, 2, 0.75f);
//        drive.drive(50,-50, 0.75f);
        drive.strafeInches(-20, 0.6f);
        drive.driveInches(55, 55, 0.75f);
//        drive.strafeInches(-12, 0.5f);
        slide.setSlideAndWait(4000);
        drive.drive(375, -375, 0.35f);
        drive.driveInches(11, 11, 0.2f);
        drive.driveInches(-4, -4, 0.2f);
        slide.setSlideAndWait(3250);
        slide.release();
        slide.setSlideAndWait(4000);
        drive.driveInches(-5, -5, 0.2f);
        slide.setSlide(600);
//        drive.driveInches(-2250,-2250, 0.5f);
        //Second CONEEeee
        drive.drive(450, -450, 0.35f);
        drive.strafeInches(2, 0.6f);
        drive.driveInches(48, 48, 0.75f);
//        drive.strafeInches(4, 0.5f);
//        drive.driveInches(10,10,0.25f);
        drive.driveInches(-2, -2, 0.5f);
        slide.grab();
        slide.setSlideAndWait(1400);
        drive.driveInches(-5, -5, 0.25f);
        drive.drive(50, -50, 0.25f);
//        drive.strafeInches(-4,0.25f);
        slide.setSlide(460);
        drive.driveInches(-16, -16, 0.75f);
        drive.driveInches(-16, -16, 0.75f);
        drive.drive(-850, 850, 0.25f);
        slide.setSlideAndWait(4000);
        drive.driveInches(8, 8, 0.2f);
        drive.driveInches(-2, -2, 0.2f);
        slide.setSlideAndWait(3250);
        slide.release();
        slide.setSlideAndWait(4000);
        drive.driveInches(-3, -3, 0.2f);
        slide.setSlideAndWait(0);
        //Stuff Work

        //THIRD CONEEEEE
//        slide.setSlide(300,true);
//        drive.drive(-725,725,0.35f);
//        drive.driveInches(21,21,0.5f);
//        drive.driveInches(16,16,0.5f);
//        slide.grab();
//        drive.driveInches(2,2,0.35f);
//        slide.setSlide(1500,true);
//        drive.driveInches(-3,-3,0.25f);
//        slide.setSlide(420,true);
//        drive.strafeInches(1,0.35f);
//        drive.driveInches(-32,-32,0.75f);
//        drive.drive(725,-725,0.25f);
//        slide.setSlide(4000, true);
//        drive.driveInches(2,2, 0.2f);
//        slide.setSlide(3250, true);
//        slide.ungrab();
//        drive.driveInches(-2,-2,0.2f);
//        slide.setSlide(0, true);

        //GAS GAS GAS
//        drive.driveInches(2,2,1f);
//        drive.strafeInches(18,0.75f);
////        drive.turnRight(0, 0.2f);
//        drive.driveInches(50, 50,0.75f);
//        drive.strafeInches(-12, 0.5f);
////        drive.turnRight(0, 0.2f);
////        cam.camera.setPipeline(pole);
////        while(!pole.getPole()) {
////            drive.strafeInches(-2, 0.5f);
////        }
//        slide.setSlide(4000, true);
//        drive.driveInches(4,4, 0.2f);
//        slide.setSlide(3250, true);
//        slide.ungrab();
//        drive.driveInches(-5,-5,0.2f);
//        slide.setSlide(600, true);
//
////        drive.driveInches(-2250,-2250, 0.5f);
//        //Second CONEEeee
//        drive.drive(-775,775,0.35f);
//        drive.driveInches(21,21,1f);
//        drive.strafeInches(4, 0.5f);
//        drive.driveInches(16,16,0.5f);
//        slide.grab();
//        drive.driveInches(5,5,0.2f);
//        slide.setSlide(1500,true);
//        drive.driveInches(-5,-5,0.25f);
//        slide.setSlide(460,true);
//        drive.strafeInches(2,0.35f);
//        drive.driveInches(-16,-16,0.75f);
//        drive.strafeInches(-2,1f);
//        drive.driveInches(-16,-16,0.75f);
//        drive.drive(725,-725,0.25f);
//        slide.setSlide(4000, true);
//        drive.driveInches(1,1, 0.2f);
//        slide.setSlide(3250, true);
//        slide.ungrab();
//        drive.driveInches(-3,-3,0.2f);
//        //Stuff Work
//
//        //THIRD CONEEEEE
////        slide.setSlide(300,true);
////        drive.drive(-725,725,0.35f);
////        drive.driveInches(21,21,0.5f);
////        drive.driveInches(16,16,0.5f);
////        slide.grab();
////        drive.driveInches(2,2,0.35f);
////        slide.setSlide(1500,true);
////        drive.driveInches(-3,-3,0.25f);
////        slide.setSlide(420,true);
////        drive.strafeInches(1,0.35f);
////        drive.driveInches(-32,-32,0.75f);
////        drive.drive(725,-725,0.25f);
////        slide.setSlide(4000, true);
////        drive.driveInches(2,2, 0.2f);
////        slide.setSlide(3250, true);
////        slide.ungrab();
////        drive.driveInches(-2,-2,0.2f);
////        slide.setSlide(0, true);

        //Stuff No Work Yet
        switch (position) {
            case CENTER:
                drive.strafeInches(14, 0.2f);
                break;
            case RIGHT:
                drive.strafeInches(38, 0.2f);
                break;
            case LEFT:
                drive.strafeInches(-12, 0.2f);
                break;
        }

//        cycle(700);
    }

    public void cycle(int height) {
        drive.drive(-850, 850, 0.25f);
        drive.driveInches(33, 33, 0.45f);
        slide.setSlideAndWait(height);
        drive.driveInches(4, 4, 0.25f);
        slide.grab();
        drive.driveInches(-1, -1, 0.15f);
        slide.setSlideAndWait(height + 600);
//        drive.driveInches(-2, -2,0.25f);
//        slide.setSlide(900,true);
        drive.driveInches(-13, -10, 0.25f);
        slide.setSlideAndWait(300);
        drive.driveInches(-25, -25, 0.45f);
        drive.drive(875, -875, 0.25f);
        slide.setSlideAndWait(4150);
        drive.driveInches(2, 2, 0.15f);
        slide.release();
        drive.driveInches(-2, -2, 0.15f);
        slide.setSlideAndWait(0);
    }
}
