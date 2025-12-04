package org.firstinspires.ftc.teamcode.autoop;

import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="OfficialAutoOpRedSmallCenter", group = "Main")
public class OfficialAutoOpRedSmallCenter extends BaseAutoOp {

    @Override
    protected void configure() {
        team = Team.RED;
        startingLocation = StartingLocation.SMALL_TRIANGLE;
        shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;
    }

}
