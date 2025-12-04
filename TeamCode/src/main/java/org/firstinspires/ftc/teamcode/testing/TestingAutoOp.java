package org.firstinspires.ftc.teamcode.testing;

import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autoop.BaseAutoOp;

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