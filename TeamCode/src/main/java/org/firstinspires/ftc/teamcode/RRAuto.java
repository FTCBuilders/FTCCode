package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

@Autonomous(name="RR_Auto", group="Auto")
public class RRAuto extends LinearOpMode {

    private MecanumDrive drive;
    private OuttakeMotor outtake;
    private transferMotor transfer;
    private intakeMotor intake;
    private AprilTag webcam1Tag; // explicitly for Webcam 1

    private static final double SHOOT_DISTANCE = 12.0; // inches to stop from tag
    private static final int BALL_COUNT = 3;
    private static final double RANGE_TOLERANCE = 1.0; // inches
    private static final double STRAFE_TOLERANCE = 1.0; // inches (camera Y)
    private static final double HEADING_TOLERANCE_DEG = 2.0; // degrees

    // Simple proportional gains for camera-based approach
    private static final double K_FORWARD = 0.05;
    private static final double K_STRAFE  = 0.06;
    private static final double K_TURN    = 0.04;
    private static final double MAX_CMD   = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        outtake = new OuttakeMotor(hardwareMap);
        transfer = new transferMotor(hardwareMap);
        intake = new intakeMotor(hardwareMap);
        webcam1Tag = new AprilTag(hardwareMap); // uses Webcam 1 inside the AprilTag class

        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Step 1: Continuously approach tag until within shooting range
        boolean inShootingPosition = false;
        while (opModeIsActive() && !inShootingPosition) {
            AprilTagDetection targetTag = getClosestDetection();

            if (targetTag != null && targetTag.ftcPose != null) {
                double[] powers = computeDriveToTag(targetTag, SHOOT_DISTANCE);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(powers[0], powers[1]), powers[2]));

                telemetry.addData("Range", targetTag.ftcPose.range);
                telemetry.addData("Bearing", targetTag.ftcPose.bearing);
                telemetry.addData("Yaw", targetTag.ftcPose.yaw);

                if (Math.abs(targetTag.ftcPose.range - SHOOT_DISTANCE) <= RANGE_TOLERANCE
                        && Math.abs(targetTag.ftcPose.y) <= STRAFE_TOLERANCE
                        && Math.abs(targetTag.ftcPose.bearing) <= HEADING_TOLERANCE_DEG) {
                    inShootingPosition = true;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                }
            } else {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                telemetry.addLine("Searching for tag...");
            }
            telemetry.update();
            sleep(20);
        }

        // Step 2: Shoot preloaded balls
        for (int i = 0; i < BALL_COUNT; i++) {
            outtake.setPower(1.0);
            transfer.setPower(1.0);
            sleep(700); // shooting duration
            outtake.stop();
            transfer.stop();
            sleep(200); // short pause
        }

        // Step 3: Park dynamically using the same tag or other detected tags
        boolean parked = false;
        while (opModeIsActive() && !parked) {
            AprilTagDetection parkTag = getClosestDetection();

            if (parkTag != null && parkTag.ftcPose != null) {
                double[] powers = computeDriveToTag(parkTag, SHOOT_DISTANCE + 18.0);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(powers[0], powers[1]), powers[2]));

                telemetry.addData("Parking Range", parkTag.ftcPose.range);
                telemetry.addData("Parking Bearing", parkTag.ftcPose.bearing);

                if (Math.abs(parkTag.ftcPose.range - (SHOOT_DISTANCE + 18.0)) <= RANGE_TOLERANCE
                        && Math.abs(parkTag.ftcPose.y) <= STRAFE_TOLERANCE
                        && Math.abs(parkTag.ftcPose.bearing) <= HEADING_TOLERANCE_DEG) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                    parked = true;
                }
            } else {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                parked = true;
            }
            telemetry.update();
            sleep(20);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
        telemetry.addLine("Autonomous complete!");
        telemetry.update();
    }

    // Get the closest detection from Webcam 1
    private AprilTagDetection getClosestDetection() {
        if (webcam1Tag == null) return null;
        List<AprilTagDetection> detections = webcam1Tag.getDetections();
        if (detections == null || detections.isEmpty()) return null;
        AprilTagDetection best = null;
        for (AprilTagDetection d : detections) {
            if (d.ftcPose == null) continue;
            if (best == null || d.ftcPose.range < best.ftcPose.range) best = d;
        }
        return best;
    }

    // Compute drive powers [vx, vy, omega] to approach a tag to a desired range
    private double[] computeDriveToTag(AprilTagDetection tag, double desiredRangeInches) {
        double rangeErr   = tag.ftcPose.range - desiredRangeInches; // + means too far
        double strafeErr  = tag.ftcPose.y;                           // + means tag to the left
        double headingErr = Math.toRadians(tag.ftcPose.bearing);     // degrees -> radians

        double vx    = clamp(rangeErr * K_FORWARD, -MAX_CMD, MAX_CMD);
        double vy    = clamp(strafeErr * K_STRAFE, -MAX_CMD, MAX_CMD);
        double omega = clamp(headingErr * K_TURN, -MAX_CMD, MAX_CMD);

        return new double[] { vx, vy, omega };
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
