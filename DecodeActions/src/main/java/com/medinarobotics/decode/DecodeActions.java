package com.medinarobotics.decode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DecodeActions {

    public static final double ROBOT_WIDTH = 18;
    public static final double ROBOT_HEIGHT = 18;
    public int fieldSide;
    public Pose2d startingPose;
    public Pose2d shootingPose;
    public Pose2d endingPose;

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

        endingPose = new Pose2d(new Vector2d(36, -24 * fieldSide), Math.toRadians(-90 * fieldSide));
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

//        double maxWheelVel = 50;
//        double minProfileAccel = -30;
//        double maxProfileAccel = 50;

        VelConstraint slowVel = (vel, pose, deriv) -> 15;
        AccelConstraint slowAccel = (accel, pose, deriv) -> new MinMax(-9, 15);

        return trajectoryActionBuilder
                .turnTo(Math.toRadians(-90 * fieldSide))
                .strafeTo(new Vector2d(-9 + ballRow * 24, -25 * fieldSide))
                .strafeTo(new Vector2d(-9 + ballRow * 24, (ballRow == 0 ? -58 : -62) * fieldSide), slowVel, slowAccel)
                .strafeTo(shootingPose.position)
                .turnTo(shootingPose.heading)
                .build();
    }

    public Action getEndAction(TrajectoryActionBuilder trajectoryActionBuilder) {
        return trajectoryActionBuilder
                .strafeTo(endingPose.position)
                .build();
    }
}