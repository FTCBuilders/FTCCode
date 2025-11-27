package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OfficialAutoOpRedBig", group = "Main")
public class OfficialAutoOpRedBig extends BaseAutoOp {

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
                .lineToX(-40)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Execute trajectory
        Actions.runBlocking(initialDrive);

        // Shoot
        autoShoot(1550);
        drive.setMotorPowers(-0.5, 0.5, -0.5, 0.5);
        sleep(1000);
        drive.setMotorPowers(0, 0, 0, 0);
    }
}
