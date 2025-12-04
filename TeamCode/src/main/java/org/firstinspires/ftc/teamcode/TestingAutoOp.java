package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.medinarobotics.decode.DecodeActions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

@Autonomous(name="TestingAutoOp", group = "Main")
public class TestingAutoOp extends BaseAutoOp {
    @Override
    protected void runAuto() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            aim();
            sleep(1000);
        }
    }

    @Override
    protected void configure() {
        team = Team.BLUE;
        startingLocation = StartingLocation.GOAL;
        shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;
    }
}