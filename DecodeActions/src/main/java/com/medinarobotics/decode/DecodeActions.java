package com.medinarobotics.decode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

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
            heading = Math.toRadians(-135);
        } else if (team == Team.RED && startingLocation == StartingLocation.GOAL) {
            x = -58 + ROBOT_HEIGHT / 2;
            y = 58 - ROBOT_HEIGHT / 2;
            heading = Math.toRadians(135);
        }
        return new Pose2d(x, y, heading);
    }

    public Action getInitialAction(TrajectoryActionBuilder trajectoryActionBuilder,
                                   Team team,
                                   StartingLocation startingLocation,
                                   ShootingLocation shootingLocation) {
        if (startingLocation == StartingLocation.GOAL && shootingLocation == ShootingLocation.NEAR_FIELD_CENTER) {
            return trajectoryActionBuilder
                    .lineToX(-18)
                    .build();
        } else if (startingLocation == StartingLocation.SMALL_TRIANGLE && shootingLocation == ShootingLocation.NEAR_FIELD_CENTER) {
            double heading;
            if (team == Team.BLUE) {
                heading = Math.toRadians(40);
            } else {
                heading = Math.toRadians(-40);
            }

            return trajectoryActionBuilder
                    .lineToX(-24)
                    .turn(heading)
                    .build();
        } else if (shootingLocation == ShootingLocation.NEAR_OBELISK) {
            double y;
            double heading;
            if (team == Team.BLUE) {
                y = -13;
                heading = Math.toRadians(45);
            } else {
                y = 13;
                heading = Math.toRadians(-45);
            }

            return trajectoryActionBuilder
                    .strafeTo(new Vector2d(-55, 2 * y))
                    .turn(heading)
                    .strafeTo(new Vector2d(-63, y))
                    .build();
        }
        return trajectoryActionBuilder
                .turn(Math.toRadians(0))
                .build();
    }

}