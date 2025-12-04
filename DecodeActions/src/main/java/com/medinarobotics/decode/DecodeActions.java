package com.medinarobotics.decode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class DecodeActions {

    public static final double ROBOT_WIDTH = 18;
    public static final double ROBOT_HEIGHT = 18;

    public Pose2d getInitialPosition(Team team, StartingLocation startingLocation) {
        double x = 0;
        double y = 0;
        double heading = 0;

        if (team == Team.BLUE && startingLocation == StartingLocation.SMALL_TRIANGLE) {
            x = 72 - ROBOT_WIDTH / 2;
            y = -24;
            heading = Math.toRadians(180);
        } else if (team == Team.RED && startingLocation == StartingLocation.SMALL_TRIANGLE) {
            x = 72 - ROBOT_WIDTH / 2;
            y = 24;
            heading = Math.toRadians(180);
        } else if (team == Team.BLUE && startingLocation == StartingLocation.GOAL) {
            x = -58 + ROBOT_HEIGHT / 2;
            y = -58 + ROBOT_HEIGHT / 2;
            heading = Math.toRadians(-130);
        } else if (team == Team.RED && startingLocation == StartingLocation.GOAL) {
            x = -58 + ROBOT_HEIGHT / 2;
            y = 58 - ROBOT_HEIGHT / 2;
            heading = Math.toRadians(130);
        }
        return new Pose2d(x, y, heading);
    }

    public Pose2d getPositionAfterShooting(Team team) {
        int fieldSide = team.equals(Team.BLUE) ? 1 : -1;
        return new Pose2d(-6, -18 * fieldSide, Math.toRadians(220 * fieldSide));
    }

    public Action getInitialAction(TrajectoryActionBuilder trajectoryActionBuilder,
                                   Team team,
                                   StartingLocation startingLocation,
                                   ShootingLocation shootingLocation) {

        int fieldSide = team.equals(Team.BLUE) ? 1 : -1;

        if (startingLocation == StartingLocation.GOAL && shootingLocation == ShootingLocation.NEAR_FIELD_CENTER) {
            return trajectoryActionBuilder
                    .strafeTo(new Vector2d(-6, -18 * fieldSide))
                    .turn(Math.toRadians(-10 * fieldSide))
                    .build();
        } else if (startingLocation == StartingLocation.SMALL_TRIANGLE && shootingLocation == ShootingLocation.NEAR_FIELD_CENTER) {
            return trajectoryActionBuilder
                    .strafeTo(new Vector2d(-6, -18 * fieldSide))
                    .turn(Math.toRadians(40 * fieldSide))
                    .build();
        } else if (shootingLocation == ShootingLocation.NEAR_OBELISK) {
            return trajectoryActionBuilder
                    .strafeTo(new Vector2d(-58.5, -39 * fieldSide))
                    .turn(Math.toRadians(40 * fieldSide))
                    .strafeTo(new Vector2d(-63, -13 * fieldSide))
                    .build();
        }
        return trajectoryActionBuilder
                .turn(Math.toRadians(0))
                .build();
    }

    public Action getBallCollectionAction(TrajectoryActionBuilder trajectoryActionBuilder, Team team, int ballRow) {
        int fieldSide = team.equals(Team.BLUE) ? 1 : -1;

        return trajectoryActionBuilder
                .turn(Math.toRadians(50 * fieldSide))
                .strafeTo(new Vector2d(-12 + ballRow * 24, -25 * fieldSide))
                .strafeTo(new Vector2d(-12 + ballRow * 24, (ballRow == 0 ? -56 : -62) * fieldSide))
                .strafeTo(new Vector2d(-6, -18 * fieldSide))
                .turn(Math.toRadians(-50 * fieldSide))
                .build();
    }
}