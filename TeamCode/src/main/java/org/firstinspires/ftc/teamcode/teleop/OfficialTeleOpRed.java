package org.firstinspires.ftc.teamcode.teleop;

import com.medinarobotics.decode.Team;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OfficialTeleOpRed", group = "Linear OpMode")
public class OfficialTeleOpRed extends BaseTeleOp{

    @Override
    protected void configure() {
        team = Team.RED;
    }
}
