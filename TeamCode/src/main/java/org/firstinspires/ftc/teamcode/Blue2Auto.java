package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Blue2Auto", group = "Main")
public class Blue2Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --------------------------
        // 🔧 Tunable variables
        // --------------------------
        double backwardDistanceInches = 1.5 * 39.3701; // 1 meter in inches
        double flywheelVelocity = 1200; // flywheel speed for shooting

        // --------------------------
        // 🔧 Initialize systems
        // --------------------------
        Pose2d startPose = new Pose2d(0, 0, 0); // robot facing shooter
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        intakeMotor intake = new intakeMotor(hardwareMap);
        transferMotor transfer = new transferMotor(hardwareMap);
        OuttakeMotor outtake = new OuttakeMotor(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // --------------------------
        // 🔵 Spin up flywheel & intake
        // --------------------------
        outtake.start(flywheelVelocity);
        intake.setPower(-1); // run intake backwards if needed

        // --------------------------
        // 🔵 Move backwards to shooting spot
        // --------------------------
        Action moveBack = drive.actionBuilder(startPose)
                .lineToX(-backwardDistanceInches)
                .build();
        Actions.runBlocking(moveBack);

        // --------------------------
        // 🔵 Shoot 3 balls (flywheel stays full power)
        // --------------------------
        for (int i = 0; i < 3; i++) {
            transfer.setPower(1);
            sleep(3000);  // give time for flywheel to recover
            transfer.stop();
        }

        // --------------------------
        // 🔵 Stop flywheel & intake
        // --------------------------
        outtake.stop();
        intake.stop();
    }
}


