package org.firstinspires.ftc.teamcode.autoop;

import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="OfficialAutoOpBlueObelisk", group = "Main")
public class OfficialAutoOpBlueObelisk extends BaseAutoOp {

    @Override
    protected void configure() {
        team = Team.BLUE;
        startingLocation = StartingLocation.GOAL;
        shootingLocation = ShootingLocation.NEAR_OBELISK;
        getBallRows = false;
    }

}
