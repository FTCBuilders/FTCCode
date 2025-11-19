package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;

// Base class: NO @Autonomous annotation, will NOT appear in driver station
public abstract class BaseAutoOp extends LinearOpMode {

    protected intakeMotor intakeMotor;
    protected transferMotor transferMotor;
    protected PIDOuttakeMotor outtakeMotor;
    protected MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor = new intakeMotor(hardwareMap);
        transferMotor = new transferMotor(hardwareMap);
        outtakeMotor = new PIDOuttakeMotor(hardwareMap);

        // Call the child class's autonomous routine
        runAuto();
    }

    // Child classes must implement this
    protected abstract void runAuto() throws InterruptedException;

    protected void autoShoot(double targetTPS) {
        long stabilizeStart = System.currentTimeMillis();
        boolean stabilized = false;
        int artifactsInRobot = 3;

        outtakeMotor.start(targetTPS);
        intakeMotor.setPower(-1);

        while (opModeIsActive() && artifactsInRobot > 0) {
            outtakeMotor.update();
            double velocity = outtakeMotor.getVelocity();

            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Actual TPS", velocity);
            telemetry.update();

            if (velocity < targetTPS - 50) {
                targetTPS += 50;
                outtakeMotor.start(targetTPS);
                stabilizeStart = System.currentTimeMillis();
            } else if (velocity > targetTPS + 50) {
                targetTPS -= 50;
                outtakeMotor.start(targetTPS);
                stabilizeStart = System.currentTimeMillis();
            } else {
                if (System.currentTimeMillis() - stabilizeStart > 350) {
                    stabilized = true;
                }
            }

            if (stabilized) {
                transferMotor.setPower(-1);
                if (artifactsInRobot == 1) {
                    drive.setMotorPowers(1, 1, 1, 1);
                    sleep(200);
                    drive.setMotorPowers(-1, -1, -1, -1);
                    sleep(300);
                    drive.setMotorPowers(0, 0, 0, 0);
                    sleep(4500);
                } else sleep(500);
                transferMotor.setPower(0);
                artifactsInRobot--;
                stabilized = false;
                sleep(1500);
            }

            sleep(250);
        }

        outtakeMotor.stop();
        intakeMotor.stop();
    }
}
