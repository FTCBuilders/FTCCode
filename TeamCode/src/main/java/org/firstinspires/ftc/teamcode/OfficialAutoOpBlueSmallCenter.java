package org.firstinspires.ftc.teamcode;

import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="OfficialAutoOpBlueSmallCenter", group = "Main")
public class OfficialAutoOpBlueSmallCenter extends BaseAutoOp {

    @Override
    protected void configure() {
        team = Team.BLUE;
        startingLocation = StartingLocation.SMALL_TRIANGLE;
        shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;
    }

}
