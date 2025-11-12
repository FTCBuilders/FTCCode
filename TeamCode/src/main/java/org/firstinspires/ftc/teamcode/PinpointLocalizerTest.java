package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

@TeleOp(name = "Pinpoint Localizer Test", group = "Test")
public class PinpointLocalizerTest extends OpMode {
    private PinpointLocalizer localizer;

    @Override
    public void init() {
        // Initialize the Pinpoint localizer with starting pose at origin
        localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, 0));
        
        telemetry.addLine("Pinpoint Localizer initialized");
        telemetry.addLine("Make sure your robot config has a device named 'pinpoint'");
        telemetry.addLine("Press A to reset position to (0,0,0°)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update the localizer
        PoseVelocity2d velocity = localizer.update();
        Pose2d pose = localizer.getPose();
        
        // Reset position if A button is pressed
        if (gamepad1.a) {
            localizer.setPose(new Pose2d(0, 0, 0));
            localizer.resetPosAndIMU(); // Also recalibrate the IMU
        }
        
        // Display current position and velocity
        telemetry.addData("X (inches)", "%.2f", pose.position.x);
        telemetry.addData("Y (inches)", "%.2f", pose.position.y);
        telemetry.addData("Heading (degrees)", "%.1f", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addLine();
        telemetry.addData("X Velocity (in/s)", "%.2f", velocity.linearVel.x);
        telemetry.addData("Y Velocity (in/s)", "%.2f", velocity.linearVel.y);
        telemetry.addData("Angular Velocity (deg/s)", "%.1f", Math.toDegrees(velocity.angVel));
        telemetry.addLine();
        telemetry.addLine("Push robot around to see tracking");
        telemetry.addLine("Press A to reset position and recalibrate IMU");
        
        telemetry.update();
    }
}

