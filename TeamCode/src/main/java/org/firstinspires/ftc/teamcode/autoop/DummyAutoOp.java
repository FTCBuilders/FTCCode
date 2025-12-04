package org.firstinspires.ftc.teamcode.autoop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "DummyAutoOp", group = "Main")
public class DummyAutoOp extends BaseAutoOp {

    @Override
    protected void configure() {
        team = Team.BLUE;
        startingLocation = StartingLocation.SMALL_TRIANGLE;
        shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;
    }

    @Override
    protected void runAuto() throws InterruptedException {
        Action initialDrive = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(24)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(initialDrive);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
