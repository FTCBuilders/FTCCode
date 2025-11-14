package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Blue1 Auto", group = "Main")
public class Blue1Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --------------------------
        // 🔧 Tunable variables
        // --------------------------
        double forwardToShootInches = 88.6; // drive forward distance
        double shootAngleDeg = 45;          // shooting angle

        // --------------------------
        // 🔧 Initialize systems
        // --------------------------
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        intakeMotor intake = new intakeMotor(hardwareMap);
        transferMotor transfer = new transferMotor(hardwareMap);
        OuttakeMotor outtake = new OuttakeMotor(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // --------------------------
        // 🔵 Spin up flywheel & intake
        // --------------------------
        outtake.start(1200);  // adjust velocity as needed
        intake.setPower(-1); // run intake backwards if needed

        // --------------------------
        // 🔵 Move to shooting spot
        // --------------------------
        Action goToShoot = drive.actionBuilder(startPose)
                .lineToX(forwardToShootInches)
                .build();
        Actions.runBlocking(goToShoot);

        // --------------------------
        // 🔵 Turn to shooting angle
        // --------------------------
        Pose2d currentPose = drive.localizer.getPose();
        Action aim = drive.actionBuilder(currentPose)
                .turn(Math.toRadians(shootAngleDeg))
                .build();
        Actions.runBlocking(aim);

        // --------------------------
        // 🔵 Shoot 3 balls (flywheel stays full power)
        // --------------------------
        for (int i = 0; i < 3; i++) {
            transfer.setPower(1);
            sleep(3000);  // wait for ball to shoot and flywheel to recover
            transfer.stop();
        }

        // --------------------------
        // 🔵 Stop flywheel & intake to save power
        // --------------------------
        outtake.stop();
        intake.stop();
    }
}
