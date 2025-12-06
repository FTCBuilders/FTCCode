package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Blue1Auto", group = "Main")
public class Blue1Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Tunable variables in METERS
        double flywheelVelocity = 1350;      // ticks/sec
        double moveDistanceHP = 2;         // meters toward HP station
        int cycles = 2;                       // number of cycles
        double turnToShootDeg = 53;           // shooting angle
        double turnToHPDeg = -152;            // turn toward human player station

        // Initialize subsystems
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor intake = new intakeMotor(hardwareMap);
        transferMotor transfer = new transferMotor(hardwareMap);
        OuttakeMotor outtake = new OuttakeMotor(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // Spin up flywheel
        outtake.start(flywheelVelocity);
        intake.setPower(-1); // optional idle intake
        sleep(1000);         // give flywheel time to reach velocity

        // 1️⃣ Turn to shooting angle
        Pose2d currentPose = drive.localizer.getPose();
        Actions.runBlocking(drive.actionBuilder(currentPose)
                .turn(Math.toRadians(turnToShootDeg))
                .build());

        // Shoot 3 preload balls
        for (int i = 0; i < 3; i++) {
            transfer.setPower(1);
            sleep(2000);
            transfer.stop();
        }

        // ***********************
        //        CYCLES
        // ***********************
        for (int rep = 0; rep < cycles; rep++) {

            // 2️⃣ Turn toward HP station
            currentPose = drive.localizer.getPose();
            Actions.runBlocking(drive.actionBuilder(currentPose)
                    .turn(Math.toRadians(turnToHPDeg))
                    .build());

            // 3️⃣ Move forward to collect balls (toward HP)
            currentPose = drive.localizer.getPose();
            intake.setPower(-1); // intake while moving
            Actions.runBlocking(drive.actionBuilder(currentPose)
                    .lineToX(currentPose.position.x + moveDistanceHP) // forward
                    .build());
            sleep(300);

            // 4️⃣ Move back to shooting zone
            currentPose = drive.localizer.getPose();
            Actions.runBlocking(drive.actionBuilder(currentPose)
                    .lineToX(currentPose.position.x - moveDistanceHP) // back
                    .build());

            // 5️⃣ Turn back to shooting angle
            currentPose = drive.localizer.getPose();
            double currentHeadingDeg = Math.toDegrees(currentPose.heading.toDouble());
            double deltaTurn = turnToShootDeg - currentHeadingDeg;
            Actions.runBlocking(drive.actionBuilder(currentPose)
                    .turn(Math.toRadians(deltaTurn))
                    .build());

            // 6️⃣ Shoot collected balls
            for (int i = 0; i < 3; i++) {
                transfer.setPower(1);
                sleep(2000);
                transfer.stop();
            }
        }

        // Stop all motors
        outtake.stop();
        intake.stop();
        transfer.stop();

        // Telemetry
        Pose2d finalPose = drive.localizer.getPose();
        telemetry.addData("Final X", finalPose.position.x);
        telemetry.addData("Final Y", finalPose.position.y);
        telemetry.addData("Final Heading (deg)", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.update();
    }
}
