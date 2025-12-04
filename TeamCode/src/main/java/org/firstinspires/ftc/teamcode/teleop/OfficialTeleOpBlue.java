package org.firstinspires.ftc.teamcode.teleop;

import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OfficialTeleOpBlue", group = "Linear OpMode")
public class OfficialTeleOpBlue extends BaseTeleOp{

    @Override
    protected void configure() {
        team = Team.BLUE;
    }
}
