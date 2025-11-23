package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.medinarobotics.decode.DecodeActions;

@Autonomous(name = "OfficialAutoOpBig", group = "Main")
public class OfficialAutoOpBig extends BaseAutoOp {

    @Override
    protected void runAuto() throws InterruptedException {
        // Define your trajectory
        Team team = Team.BLUE;
        StartingLocation startingLocation = StartingLocation.GOAL;
        ShootingLocation shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;

        DecodeActions decodeActions = new DecodeActions();
        Pose2d pose2d = decodeActions.getInitialPosition(team, startingLocation);


        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(pose2d);
        Action initialDrive = decodeActions.getInitialAction(trajectoryActionBuilder, team,
                startingLocation, shootingLocation);

        waitForStart();
        if (isStopRequested()) return;

        // Execute trajectory
        Actions.runBlocking(initialDrive);

        // Shoot
        autoShoot(1550);
    }
}
