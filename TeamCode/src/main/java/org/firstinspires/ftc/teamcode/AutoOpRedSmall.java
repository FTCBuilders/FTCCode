package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpRedSmall", group = "Main")
public class AutoOpRedSmall extends BaseAutoOp {

    @Override
    protected void runAuto() throws InterruptedException {
        // Define your trajectory
        Action initialDrive = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(72)
                .strafeTo(new Vector2d(72, 24))
                .turn(Math.toRadians(-30))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Execute trajectory
        Actions.runBlocking(initialDrive);

        // Shoot
        autoShoot();
    }
}
