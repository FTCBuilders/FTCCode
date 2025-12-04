package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

import java.util.ArrayList;
import java.util.List;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Team team = Team.BLUE;
        StartingLocation startingLocation = StartingLocation.GOAL;
        ShootingLocation shootingLocation = ShootingLocation.NEAR_FIELD_CENTER;

        MeepMeep meepMeep = new MeepMeep(800);
        DecodeActions decodeActions = new DecodeActions();

        ColorScheme colorScheme = team == Team.BLUE ? new ColorSchemeBlueLight() : new ColorSchemeRedLight();
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(DecodeActions.ROBOT_WIDTH, DecodeActions.ROBOT_HEIGHT)
                .setColorScheme(colorScheme)
                .build();

        Pose2d initialPosition = decodeActions.getInitialPosition(team, startingLocation);
        TrajectoryActionBuilder trajectoryActionBuilder = myBot.getDrive().actionBuilder(initialPosition);
        Action initialDrive = decodeActions.getInitialAction(trajectoryActionBuilder, team,
                startingLocation, shootingLocation);

        Pose2d positionAfterShooting = decodeActions.getPositionAfterShooting(team);
        TrajectoryActionBuilder trajectoryActionBuilderAfterShooting = myBot.getDrive().actionBuilder(positionAfterShooting);

        List<Action> actionList = new ArrayList<>();
        actionList.add(initialDrive);
        actionList.add(new SleepAction(1));
        for (int i=0;i<3;i++) {
            Action getBallRowAction = decodeActions.getBallCollectionAction(trajectoryActionBuilderAfterShooting, team, i);
            actionList.add(getBallRowAction);
            actionList.add(new SleepAction(1));
        }

        Action actionSequence = new SequentialAction(actionList);
        myBot.runAction(actionSequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}