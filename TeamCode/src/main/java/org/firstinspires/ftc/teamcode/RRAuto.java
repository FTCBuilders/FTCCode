package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


import java.util.Arrays;
import java.util.List;


@Autonomous(name="RR_Auto_Preloaded", group="Auto")
public class RRAuto extends LinearOpMode {
    private MecanumDrive drive;
    private OuttakeMotor outtake;
    private transferMotor transfer;
    private intakeMotor intake;
    private AprilTag aprilTag;


    private ElapsedTime timer = new ElapsedTime();

    // Path points (units in inches)
    private Pose2d startPose = new Pose2d(6, 12, 0);         // starting near player
    private Pose2d shootingPose = new Pose2d(6, 20, 0);      // in front of shooter
    private Pose2d safeZonePose = new Pose2d(36, 0, 0);      // safe zone parking

    private final double DIST_TOLERANCE = 0.5;
    private final double HEADING_TOLERANCE = Math.toRadians(2);


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap,  new Pose2d(0,0,0));
        outtake = new OuttakeMotor(hardwareMap);
        transfer = new transferMotor(hardwareMap);
        intake = new intakeMotor(hardwareMap);

        //if it can detect april tag

        try {
            aprilTag = new AprilTag(hardwareMap);
        } catch (Exception e) {
            aprilTag = null;
            telemetry.addLine("AprilTag sensor not found — skipping detection");
            telemetry.update();
        }


        telemetry.addLine("Initialized — Ready to run");
        telemetry.update();

        waitForStart();

        boolean reachedShootingPose = false;

        while (opModeIsActive() && !reachedShootingPose) {
            // Move towards shooting position while optionally doing other actions
            reachedShootingPose = moveToPoseNonBlocking(shootingPose);

            // Example: spin up shooter continuously while moving
            outtake.start(1.0);
            transfer.setPower(1.0);

            telemetry.addData("Moving to shooting pose", !reachedShootingPose);
            telemetry.update();

            sleep(20); // small loop delay
        }

        if (isStopRequested()) return;

        // 1️⃣ Move to shooting position
        moveToPoseNonBlocking(shootingPose);


        // 2️⃣ Shoot preloaded balls (only when in front of shooter)
        shootBalls(3);

        // 3️⃣ Scan AprilTag and display pattern on telemetry
        boolean aprilDetected = false;
        if (aprilTag != null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                aprilDetected = true;
                // Optionally handle detection info here
                for (AprilTagDetection tag : detections) {
                    telemetry.addData("AprilTag ID", tag.id);
                }
                telemetry.update();
            }
        }

        //stop shooting
        moveToPoseNonBlocking(safeZonePose);

        // 4️⃣ Park in safe zone

        telemetry.addLine("Auto complete!");
        telemetry.update();

    }

    private boolean moveToPoseNonBlocking (Pose2d target) {

            Pose2d current = drive.localizer.getPose();

            double dx = target.position.x - current.position.x;
            double dy = target.position.y - current.position.y;
            double dHeading = angleDifference(target.heading.toDouble(), current.heading.toDouble());

            double distance = Math.hypot(dx, dy);

            if (distance < DIST_TOLERANCE && Math.abs(dHeading) < HEADING_TOLERANCE) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
                return true;
            }

            double kPos = 0.1;
            double kHeading = 0.1;

        double vx = clamp(dx * kPos, -1.0, 1.0);
        double vy = clamp(dy * kPos, -1.0, 1.0);
        double omega = clamp(dHeading * kHeading, -1.0, 1.0);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(vx, vy), omega));

        sleep(20);

        return false;

        }

   // clamper helper
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // helper for angle wrap-around
    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }


    private void shootBalls(int count) throws InterruptedException {
        for (int i = 0; i < count; i++) {
            // Start shooting
            outtake.start(1.0);
            transfer.setPower(1.0);

            sleep(700);  // time to shoot one ball

            // Stop motors after each shot
            outtake.stop();
            transfer.stop();

            sleep(200);  // short pause between shots
        }
    }

    private void scanAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            telemetry.addLine("No AprilTag detected");
        } else {
            for (AprilTagDetection tag : detections) {
                telemetry.addLine("AprilTag ID: " + tag.id);
                telemetry.addLine("Motif pattern: PURPLE, GREEN, PURPLE"); // Example pattern
            }
        }
        telemetry.update();

    }

}