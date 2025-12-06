package org.firstinspires.ftc.teamcode.autoop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
import org.firstinspires.ftc.teamcode.motors.IntakeMotor;
import org.firstinspires.ftc.teamcode.motors.PIDOuttakeMotor;
import org.firstinspires.ftc.teamcode.motors.TransferMotor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.IMU;

// Base class: NO @Autonomous annotation, will NOT appear in driver station
public abstract class BaseAutoOp extends LinearOpMode {

    protected IntakeMotor intakeMotor;
    protected TransferMotor transferMotor;
    protected PIDOuttakeMotor outtakeMotor;
    protected MecanumDrive drive;
    private Limelight3A limelight;
    private IMU imu;

    protected Team team;
    protected StartingLocation startingLocation;
    protected ShootingLocation shootingLocation;

    protected boolean isShootingFromFar;
    protected boolean getBallRows = true;

    protected DecodeActions decodeActions;

    @Override
    public void runOpMode() throws InterruptedException {
        configure();

        isShootingFromFar = shootingLocation == ShootingLocation.NEAR_SMALL_TRIANGLE;

        decodeActions = new DecodeActions(team, startingLocation, shootingLocation);

        // Initialize hardware
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor = new IntakeMotor(hardwareMap);
        transferMotor = new TransferMotor(hardwareMap);
        outtakeMotor = new PIDOuttakeMotor(hardwareMap, "outtakeMotor");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(team == Team.BLUE ? 1 : 5);
        telemetry.addData("pipeline", team == Team.BLUE ? 1 : 5);
        limelight.start();

        imu = drive.lazyImu.get();

        Pose2d initialLocation = decodeActions.getInitialPosition();

        // Call the child class's autonomous routine
        runAuto();
    }

    protected void runAuto() throws InterruptedException {

        double targetTPS = isShootingFromFar ? 1700 : 1500;

        waitForStart();
        if (isStopRequested()) return;

        startMotors(targetTPS);

        // Keep flywheel updated while driving
        Thread flywheelThread = new Thread(() -> {
            while (opModeIsActive()) {
                outtakeMotor.update();
                try { Thread.sleep(20); } catch (Exception ignored) {}
            }
        });
        flywheelThread.start();

        Pose2d initialPosition = decodeActions.getInitialPosition();
        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(initialPosition);

        Action initialDrive = decodeActions.getInitialAction(trajectoryActionBuilder, shootingLocation);

        Actions.runBlocking(initialDrive);

        autoShoot(targetTPS);

        if (getBallRows) {

            // TODO: Get current position from pinpoint
            Pose2d positionAfterShooting = decodeActions.getPositionAfterShooting();

            /*for (int i = 0; i < 3; i++) {
                TrajectoryActionBuilder trajectoryActionBuilderAfterShooting = drive.actionBuilder(positionAfterShooting);

                Action getBallRow = decodeActions.getBallCollectionAction(trajectoryActionBuilderAfterShooting, isShootingFromFar ? 2 - i : i);
                Actions.runBlocking(getBallRow);
                autoShoot(targetTPS);
            }*/
            TrajectoryActionBuilder trajectoryActionBuilderAfterShooting = drive.actionBuilder(positionAfterShooting);

            Action getBallRow = decodeActions.getBallCollectionAction(trajectoryActionBuilderAfterShooting, isShootingFromFar ? 2 : 0);
            Actions.runBlocking(getBallRow);
            autoShoot(targetTPS);
        }

        if (shootingLocation == ShootingLocation.NEAR_FIELD_CENTER) {
            Pose2d positionAfterShooting = decodeActions.getPositionAfterShooting();
            TrajectoryActionBuilder trajectoryActionBuilderEnd = drive.actionBuilder(positionAfterShooting);
            Action endAction = decodeActions.getEndAction(trajectoryActionBuilderEnd);
            Actions.runBlocking(endAction);
        }

        stopMotors();
    }

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
            Pose2d position = decodeActions.getPositionAfterShooting();
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
                sleep(700);
                transferMotor.setPower(0);
                artifactsInRobot--;
                stabilized = false;
                sleep(300);
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
