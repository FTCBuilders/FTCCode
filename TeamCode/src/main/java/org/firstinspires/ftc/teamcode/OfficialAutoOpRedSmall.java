package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OfficialAutoOpRedSmall", group = "Main")
public class OfficialAutoOpRedSmall extends BaseAutoOp {

    @Override
    protected void configure() {
        team = Team.RED;
        startingLocation = StartingLocation.GOAL;
        shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;
    }

    @Override
    protected void runAuto() throws InterruptedException {
        // Define your trajectory
        Action initialDrive = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(72)
                .strafeTo(new Vector2d(66, 24))
                .turn(Math.toRadians(-30))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Execute trajectory
        Actions.runBlocking(initialDrive);

        // Shoot
        autoShoot(1550);
    }
}
