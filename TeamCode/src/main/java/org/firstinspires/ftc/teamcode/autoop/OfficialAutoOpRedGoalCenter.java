package org.firstinspires.ftc.teamcode.autoop;

import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="OfficialAutoOpRedGoalCenter", group = "Main")
public class OfficialAutoOpRedGoalCenter extends BaseAutoOp {

    @Override
    protected void configure() {
        team = Team.RED;
        startingLocation = StartingLocation.GOAL;
        shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;
    }

}
