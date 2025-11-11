package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Blue1 Auto", group = "Main")
public class PRAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // intake + outtake motors
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        DcMotor outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");

        waitForStart();
        if (isStopRequested()) return;

        // Shoot
        outtakeMotor.setPower(1);
        intakeMotor.setPower(1);

        double distanceInches = 2.25 * 39.3701; // 2.3 meters forward

        // Move forward to shooting spot
        Action forward1 = drive.actionBuilder(startPose)
                .lineToX(startPose.position.x + distanceInches)
                .build();
        Actions.runBlocking(forward1);

        // Turn 45° toward target
        Pose2d poseAfterForward = drive.localizer.getPose();
        Action turn1 = drive.actionBuilder(poseAfterForward)
                .turn(Math.toRadians(45))
                .build();
        Actions.runBlocking(turn1);

        transferMotor.setPower(1);
        sleep(3000);


        // Stop transfer (save energy)
        transferMotor.setPower(0);
        intakeMotor.setPower(1);
        outtakeMotor.setPower(1);

        // Turn another 45° for collecting
        Pose2d poseAfterShoot = drive.localizer.getPose();
        Action turn2 = drive.actionBuilder(poseAfterShoot)
                .turn(Math.toRadians(49))
                .build();
        Actions.runBlocking(turn2);

        // Move back 0.5 m
        double moveForward = .125 * 39.3701;
        Pose2d poseAfterTurn2 = drive.localizer.getPose();
        Action forward = drive.actionBuilder(poseAfterTurn2)
                .lineToX(poseAfterTurn2.position.x + moveForward)
                .build();
        Actions.runBlocking(forward);
        sleep(300);

        // Move forward 0.5 m to collect
        Pose2d poseAfterforward = drive.localizer.getPose();
        Action forwardCollect = drive.actionBuilder(poseAfterForward)
                .lineToX(poseAfterForward.position.x + moveForward)
                .build();
        Actions.runBlocking(forwardCollect);

        // Stop intake + outtake after collecting
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);

        // Move back 0.12 meters to shooting position
        double backToShootDist = 0.120 * 39.3701; // 0.12 m in inches
        Pose2d poseAfterCollect = drive.localizer.getPose();
        Action moveBackToShoot = drive.actionBuilder(poseAfterCollect)
                .lineToX(poseAfterCollect.position.x - backToShootDist)
                .build();
        Actions.runBlocking(moveBackToShoot);

        // Turn to shooting angle (45°)
        Pose2d poseAtShoot = drive.localizer.getPose();
        Action turnToShoot = drive.actionBuilder(poseAtShoot)
                .turn(Math.toRadians(-45))
                .build();
        Actions.runBlocking(turnToShoot);

        // Shoot the balls
        intakeMotor.setPower(1);
        outtakeMotor.setPower(1);
        transferMotor.setPower(1);
        sleep(1500);

        // Stop all motors
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        transferMotor.setPower(0);


        // Final pose telemetry
        Pose2d finalPose = drive.localizer.getPose();
        telemetry.addData("X (in)", finalPose.position.x);
        telemetry.addData("Y (in)", finalPose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.update();

        sleep(1000);
    }
}
