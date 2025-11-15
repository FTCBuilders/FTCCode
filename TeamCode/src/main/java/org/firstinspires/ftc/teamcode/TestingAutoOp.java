package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

@Autonomous(name = "TestingAutoOp", group = "Main")
public class TestingAutoOp extends LinearOpMode {

    private intakeMotor intakeMotor;
    private transferMotor transferMotor;
    private PIDOuttakeMotor outtakeMotor;

    public void autoShoot() {
        // Start flywheel early
        double targetTPS = 1550;

        // Stabilization + dynamic target adjustment
        long stabilizeStart = System.currentTimeMillis();
        boolean stabilized = false;
        int fireCount = 0;

        outtakeMotor.start(targetTPS);
        intakeMotor.setPower(-1);

        while (opModeIsActive() && fireCount < 3) {

            outtakeMotor.update();
            double velocity = outtakeMotor.getVelocity();

            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Actual TPS", velocity);
            telemetry.update();

            // Adjust target dynamically if outside tolerance
            if (velocity < 1500) {
                targetTPS += 50;
                outtakeMotor.start(targetTPS);
                stabilizeStart = System.currentTimeMillis();  // reset timer
            } else if (velocity > 1600) {
                targetTPS -= 50;
                outtakeMotor.start(targetTPS);
                stabilizeStart = System.currentTimeMillis();  // reset timer
            } else {
                // Only consider stabilized if within ±50 for 350ms
                if (System.currentTimeMillis() - stabilizeStart > 350) {
                    stabilized = true;
                }
            }

            if (stabilized) {
                transferMotor.setPower(-1);
                sleep(500);
                transferMotor.setPower(0);
                fireCount++;
                stabilized = false;
                sleep(1500);
            }

            sleep(250);
        }

        // Stop flywheel
        outtakeMotor.start(0);
        intakeMotor.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intakeMotor = new intakeMotor(hardwareMap);
        transferMotor = new transferMotor(hardwareMap);
        outtakeMotor = new PIDOuttakeMotor(hardwareMap);

        Action initialDrive = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(72)
                .strafeTo(new Vector2d(72, -24))
                .turn(Math.toRadians(45))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Drive to shooting position
        Actions.runBlocking(initialDrive);

        autoShoot();
    }
}
