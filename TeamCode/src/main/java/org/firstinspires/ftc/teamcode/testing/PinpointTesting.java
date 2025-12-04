package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.medinarobotics.decode.DecodeActions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name="PinpointTesting", group = "Main")
public class PinpointTesting extends LinearOpMode {

    protected MecanumDrive drive;
    protected Team team;
    protected StartingLocation startingLocation;
    protected ShootingLocation shootingLocation;
    protected DecodeActions decodeActions;

    @Override
    public void runOpMode() throws InterruptedException {

        team = Team.BLUE;
        startingLocation = StartingLocation.GOAL;
        shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;
        decodeActions = new DecodeActions();

        Pose2d initialPosition = decodeActions.getInitialPosition(team, startingLocation);

        // Init drive
        drive = new MecanumDrive(hardwareMap, initialPosition);

        // Wait for start
        waitForStart();
        if (isStopRequested()) return;

        // Build trajectory
        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(initialPosition);
        Action initialDrive = decodeActions.getInitialAction(trajectoryActionBuilder, team, startingLocation, shootingLocation);

        // Run trajectory manually in a loop to update Pinpoint
        boolean running = true;
        while (opModeIsActive() && running) {
            drive.updatePoseEstimate();
            TelemetryPacket dummyPacket = new TelemetryPacket();
            running = initialDrive.run(dummyPacket);

            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("X (in)", pose.position.x);
            telemetry.addData("Y (in)", pose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            sleep(10);
        }
    }
}