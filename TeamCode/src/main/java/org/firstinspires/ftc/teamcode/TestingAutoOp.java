package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestingAutoOp", group = "Main")
public class TestingAutoOp extends BaseAutoOp {

    @Override
    protected void configure() {
        team = Team.BLUE;
        startingLocation = StartingLocation.GOAL;
        shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;
    }

    @Override
    protected void runAuto() throws InterruptedException {
        // Define your trajectory
        Action initialDrive = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(-24)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Execute trajectory
        Actions.runBlocking(initialDrive);

        // Shoot
        autoShoot(1550);
    }
}
