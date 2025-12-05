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
    public int fieldSide;
    public Pose2d startingPose;
    public Pose2d shootingPose;

    public DecodeActions(Team team,
                         StartingLocation startingLocation,
                         ShootingLocation shootingLocation) {
        fieldSide = team.equals(Team.BLUE) ? 1 : -1;

        startingPose = startingLocation == StartingLocation.SMALL_TRIANGLE ?
                new Pose2d(new Vector2d(72 - ROBOT_WIDTH / 2, -24 * fieldSide), Math.toRadians(180)) :
                new Pose2d(new Vector2d(-58 + ROBOT_HEIGHT / 2, -58 * fieldSide + ROBOT_HEIGHT * fieldSide / 2), Math.toRadians(-135 * fieldSide));

        shootingPose = shootingLocation == ShootingLocation.NEAR_FIELD_CENTER ?
                new Pose2d(new Vector2d(-6, -18 * fieldSide), Math.toRadians(220 * fieldSide)) :
                shootingLocation == ShootingLocation.NEAR_OBELISK ?
                        new Pose2d(new Vector2d(-63, -13 * fieldSide),  Math.toRadians(-90 * fieldSide)) :
                        new Pose2d(new Vector2d(54, -18 * fieldSide), Math.toRadians(200 * fieldSide));
    }

    public Pose2d getInitialPosition() {
        return new Pose2d(startingPose.position, startingPose.heading);
    }

    public Pose2d getPositionAfterShooting() {
        return new Pose2d(shootingPose.position, shootingPose.heading);
    }

    public Action getInitialAction(TrajectoryActionBuilder trajectoryActionBuilder, ShootingLocation shootingLocation) {

        if (shootingLocation == ShootingLocation.NEAR_OBELISK) {
            return trajectoryActionBuilder
                    .strafeTo(new Vector2d(-58.5, -39 * fieldSide))
                    .turnTo(shootingPose.heading)
                    .strafeTo(shootingPose.position)
                    .build();
        } else {
            return trajectoryActionBuilder
                    .strafeTo(shootingPose.position)
                    .turnTo(shootingPose.heading)
                    .build();
        }
    }

    public Action getBallCollectionAction(TrajectoryActionBuilder trajectoryActionBuilder, int ballRow) {
        return trajectoryActionBuilder
                .turnTo(Math.toRadians(-90 * fieldSide))
                .strafeTo(new Vector2d(-12 + ballRow * 24, -25 * fieldSide))
                .strafeTo(new Vector2d(-12 + ballRow * 24, (ballRow == 0 ? -56 : -62) * fieldSide))
                .strafeTo(shootingPose.position)
                .turnTo(shootingPose.heading)
                .build();
    }
}