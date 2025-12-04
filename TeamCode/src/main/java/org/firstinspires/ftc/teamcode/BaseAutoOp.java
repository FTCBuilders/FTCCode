package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.medinarobotics.decode.DecodeActions;
import com.medinarobotics.decode.ShootingLocation;
import com.medinarobotics.decode.StartingLocation;
import com.medinarobotics.decode.Team;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.IMU;

// Base class: NO @Autonomous annotation, will NOT appear in driver station
public abstract class BaseAutoOp extends LinearOpMode {

    protected intakeMotor intakeMotor;
    protected transferMotor transferMotor;
    protected PIDOuttakeMotor outtakeMotor;
    protected MecanumDrive drive;
    private Limelight3A limelight;
    private IMU imu;

    protected Team team;
    protected StartingLocation startingLocation;
    protected ShootingLocation shootingLocation;

    protected DecodeActions decodeActions;

    @Override
    public void runOpMode() throws InterruptedException {
        configure();

        decodeActions = new DecodeActions();

        // Initialize hardware
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor = new intakeMotor(hardwareMap);
        transferMotor = new transferMotor(hardwareMap);
        outtakeMotor = new PIDOuttakeMotor(hardwareMap, "outtakeMotor");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(team == Team.BLUE ? 1 : 5);
        telemetry.addData("pipeline", team == Team.BLUE ? 1 : 5);
        limelight.start();

        imu = drive.lazyImu.get();

        Pose2d initialLocation = decodeActions.getInitialPosition(team, startingLocation);

        // Call the child class's autonomous routine
        runAuto();
    }

    // Child classes must implement this
    protected abstract void runAuto() throws InterruptedException;

    // Child classes must implement this
    protected abstract void configure();

    protected void startMotors(double targetTPS) {
        outtakeMotor.start(targetTPS);
        intakeMotor.setPower(-1);
    }

    protected double getDistanceFromTag(double Ta) {
        double a = 31347.4; // 30665.95
        double b = 2.007394;
        double distance = Math.pow(a/Ta, 1.0 / b);
        return distance;
    }

    protected void aim() {
        int retries = 0;

        while (true) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult llResult = limelight.getLatestResult();
            boolean isAprilTagVisible = llResult != null && llResult.isValid();
            telemetry.addData("AprilTag visible", isAprilTagVisible);

            retries = isAprilTagVisible ? 0 : retries + 1;
            if (retries > 10) {
                break;
            }
            if (!isAprilTagVisible) {
                continue;
            }

            double rotate = llResult.getTx() * -1 * 0.2;
            double rotateRadians = Math.toRadians(rotate);
            rotateRadians = Math.max(Math.toRadians(-20), Math.min(Math.toRadians(20), rotateRadians));
            if (Math.abs(rotateRadians) < 1e-3) {   // ~0.001 rad (~0.06°)
                break;
            }

            // TODO Get current position from pinpoint
            Pose2d position = decodeActions.getPositionAfterShooting(team);
            Action action = drive.actionBuilder(position).turn(rotateRadians).build();
            Actions.runBlocking(action);

            // double distance = getDistanceFromTag(isAprilTagVisible ? llResult.getTa() : 0);

            telemetry.update();
            sleep(250);
        }
    }

    protected void autoShoot(double desiredTPS) {
        long stabilizeStart = System.currentTimeMillis();
        boolean stabilized = false;
        int artifactsInRobot = 3;
        double targetTPS = desiredTPS;

        aim();

        while (opModeIsActive() && artifactsInRobot > 0) {
            outtakeMotor.update();
            double velocity = outtakeMotor.getVelocity();

            telemetry.addData("Desired TPS", desiredTPS);
            telemetry.addData("Actual TPS", velocity);
            telemetry.addData("Target TPS", targetTPS);
            telemetry.update();

            if (velocity < desiredTPS - 50) {
                targetTPS += 50;
                outtakeMotor.start(targetTPS);
                stabilizeStart = System.currentTimeMillis();
            } else if (velocity > desiredTPS + 50) {
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
                artifactsInRobot--;
                stabilized = false;
                sleep(200);
            }

            sleep(250);
        }
    }

    protected void stopMotors() {
        outtakeMotor.stop();
        intakeMotor.stop();
        transferMotor.stop();
    }
}
