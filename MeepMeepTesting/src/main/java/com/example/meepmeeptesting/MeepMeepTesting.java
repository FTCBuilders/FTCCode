package com.example.meepmeeptesting;

import com.medinarobotics.decode.DecodeActions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Team team = Team.BLUE;
        StartingLocation startingLocation = StartingLocation.GOAL;
        ShootingLocation shootingLocation = ShootingLocation.NEAR_OBELISK;

        MeepMeep meepMeep = new MeepMeep(800);
        DecodeActions decodeActions = new DecodeActions();

        ColorScheme colorScheme = team == Team.BLUE ? new ColorSchemeBlueLight() : new ColorSchemeRedLight();
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(DecodeActions.ROBOT_WIDTH, DecodeActions.ROBOT_HEIGHT)
                .setColorScheme(colorScheme)
                .build();

        Pose2d pose2d = decodeActions.getInitialPosition(team, startingLocation);
        TrajectoryActionBuilder trajectoryActionBuilder = myBot.getDrive().actionBuilder(pose2d);

        Action action = decodeActions.getInitialAction(trajectoryActionBuilder, team,
                startingLocation, shootingLocation);

        myBot.runAction(action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}