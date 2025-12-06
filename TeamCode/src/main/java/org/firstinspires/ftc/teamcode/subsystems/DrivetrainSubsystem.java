package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class DrivetrainSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private Telemetry telemetry;

    public DrivetrainSubsystem(HardwareMap hardwareMap, Pose2d initialPose) {
        drive = new MecanumDrive(hardwareMap, initialPose);
    }
    
    // Method to set telemetry for debugging
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        // Debug telemetry to see what inputs we're getting
        if (telemetry != null) {
            telemetry.addData("leftY param (from rightStickX)", "%.2f", leftY);
            telemetry.addData("leftX param (from leftStickX)", "%.2f", leftX);
            telemetry.addData("rightX param (from leftStickY)", "%.2f", rightX);
            telemetry.addLine("---");
        }
        
        // FIELD-RELATIVE DRIVE (robot moves relative to field, not robot orientation)
        // driveFieldRelative(leftY, leftX, rightX);
        
        // For testing/debugging, you can switch to robot-relative:
        driveRobotRelative(leftY, leftX, rightX);
    }
    
    private void driveFieldRelative(double leftY, double leftX, double rightX) {
        double heading = drive.localizer.getPose().heading.toDouble();

        // Standard 2D vector rotation formula to convert joystick input to field coordinates
        // We rotate the joystick input by the negative of the robot's heading
        double fieldForward = leftY * Math.cos(-heading) - leftX * Math.sin(-heading);
        double fieldStrafe = leftY * Math.sin(-heading) + leftX * Math.cos(-heading);

        // Road Runner's PoseVelocity2d constructor: Vector2d(forward, strafe), turn
        // X = forward/backward, Y = left/right strafe
        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(fieldForward, fieldStrafe), -rightX)
        );
    }
    
    private void driveRobotRelative(double leftY, double leftX, double rightX) {
        // Simple robot-relative drive (easier to understand and debug)
        // leftY = forward/backward, leftX = strafe left/right, rightX = turn
        
        // FTC gamepads have inverted Y axis, so negate leftY for forward movement
        double forward = -leftY;  // Negate Y axis (FTC convention)
        double strafe = leftX;    // X axis is normal
        double turn = -rightX;    // Negate turn for correct direction
        
        if (telemetry != null) {
            telemetry.addData("Robot Forward (Vector2d.x)", "%.2f", forward);
            telemetry.addData("Robot Strafe (Vector2d.y)", "%.2f", strafe);
            telemetry.addData("Robot Turn", "%.2f", turn);
            telemetry.update();
        }
        
        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(forward, strafe), turn)
        );
    }

    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}