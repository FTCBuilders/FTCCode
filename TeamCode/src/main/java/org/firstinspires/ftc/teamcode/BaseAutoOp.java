package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

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

    protected void autoShoot() {
        double targetTPS = 1550;
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

            if (velocity < 1500) {
                targetTPS += 50;
                outtakeMotor.start(targetTPS);
                stabilizeStart = System.currentTimeMillis();
            } else if (velocity > 1600) {
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
                sleep(500);
                transferMotor.setPower(0);
                fireCount++;
                stabilized = false;
                sleep(1500);
            }

            sleep(250);
        }

        outtakeMotor.start(0);
        intakeMotor.stop();
    }
}
