package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.medinarobotics.decode.DecodeActions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="OfficialAutonomousSmart", group = "Main")
public class OfficialAutonomousSmart extends BaseAutoOp {

    @Override
    protected void runAuto() throws InterruptedException {

        double targetTPS = 1550;

        Team team = Team.BLUE;
        StartingLocation startingLocation = StartingLocation.GOAL;
        ShootingLocation shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;

        DecodeActions decodeActions = new DecodeActions();

        waitForStart();
        if (isStopRequested()) return;

        startMotors(targetTPS);

        // Keep flywheel updated while driving
        Thread flywheelThread = new Thread(() -> {
            while (opModeIsActive()) {
                outtakeMotor.update();
                try { Thread.sleep(20); } catch (Exception ignored) {}
            }
        });
        flywheelThread.start();

        Pose2d initialPosition = decodeActions.getInitialPosition(team, startingLocation);
        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(initialPosition);

        Action initialDrive = decodeActions.getInitialAction(trajectoryActionBuilder, team,
                startingLocation, shootingLocation);

        Actions.runBlocking(initialDrive);

        autoShoot(targetTPS);

        Pose2d positionAfterShooting = decodeActions.getPositionAfterShooting(team);
        TrajectoryActionBuilder trajectoryActionBuilderAfterShooting = drive.actionBuilder(positionAfterShooting);

        for (int i=0;i<3;i++) {
            Action getBallRow = decodeActions.getBallCollectionAction(trajectoryActionBuilderAfterShooting, team, i);
            Actions.runBlocking(getBallRow);
            autoShoot(targetTPS);
        }

        stopMotors();
    }
}
