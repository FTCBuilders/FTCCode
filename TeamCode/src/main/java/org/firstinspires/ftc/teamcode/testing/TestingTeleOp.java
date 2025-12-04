package org.firstinspires.ftc.teamcode.testing;

import com.medinarobotics.decode.Team;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.motors.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.motors.IntakeMotor;
import org.firstinspires.ftc.teamcode.motors.PIDOuttakeMotors;
import org.firstinspires.ftc.teamcode.motors.TransferMotor;

@TeleOp(name = "TestingTeleOp", group = "Linear OpMode")
public class TestingTeleOp extends LinearOpMode {

    Team team = Team.RED;

    private CustomMecanumDrive mecanumDrive;
    private IntakeMotor intakeMotor;
    private TransferMotor transferMotor;
    private PIDOuttakeMotors outtakeMotors;
    private Limelight3A limelight;
    private IMU imu;


    // ----- TOGGLE STATES -----
    private boolean intakeOn = false;
    private boolean outtakeOn = false;

    // ----- BUTTON STATE TRACKING -----
    private boolean lastIntakeButton = false;
    private boolean lastOuttakeButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // ----- DISTANCE FROM GOAL -----
    private double distance;

    Gamepad driveController;
    Gamepad ballController;

    private void setupControllers() {
        // Run with a single controller if the other one has not been seen yet
        if (gamepad1.getGamepadId() > -1 && gamepad2.getGamepadId() == -1) {
            // gamepad2 seems disconnected, so let's use gamepad1 for everything
            driveController = gamepad1;
            ballController = gamepad1;
        } else if (gamepad1.getGamepadId() == -1 && gamepad2.getGamepadId() > -1) {
            // gamepad1 seems disconnected, so let's use gamepad2 for everything
            driveController = gamepad2;
            ballController = gamepad2;
        } else {
            // Both gamepads are in the same state, so assign them to their default roles
            driveController = gamepad1;
            ballController = gamepad2;
        }
    }

    public double getDistanceFromTag(double Ta) {
        double a = 31347.4; // 30665.95
        double b = 2.007394;
        double distance = Math.pow(a/Ta, 1.0 / b);
        return distance;
    }

    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
        intakeMotor = new IntakeMotor(hardwareMap);
        transferMotor = new TransferMotor(hardwareMap);
        outtakeMotors = new PIDOuttakeMotors(hardwareMap, "outtakeMotor", "outtakeMotor2");
        int targetTicksPerSecond = 1500;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(team == Team.BLUE ? 1 : 5);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        telemetry.addLine("Initialized — Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            setupControllers();

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult llResult = limelight.getLatestResult();
            boolean isAprilTagVisible = llResult != null && llResult.isValid();
            distance = getDistanceFromTag(isAprilTagVisible ? llResult.getTa() : 0);

            // ----- DRIVE CONTROL -----
            double forward = 0;
            double strafe = 0;
            double rotate = 0;
            if (!driveController.left_bumper || !isAprilTagVisible) {
                // traveling
                forward = -driveController.left_stick_y;
                strafe = driveController.left_stick_x;
                rotate = driveController.right_stick_x;
            } else {
                // aiming
                rotate = llResult.getTx() * 0.05;
                rotate = Math.max(-1, Math.min(1, rotate));
            }

            mecanumDrive.setDrivePower(forward, strafe, rotate);

            // Check if left trigger is pressed (reverse mode)
            boolean reverseMode = ballController.left_trigger > 0.1;
            double direction = reverseMode ? 1.0 : -1.0;

            // ----- INTAKE TOGGLE (A button) -----
            boolean currentIntakeButton = ballController.a;
            if (currentIntakeButton && !lastIntakeButton) {
                intakeOn = !intakeOn;
            }
            lastIntakeButton = currentIntakeButton;

            if (intakeOn) intakeMotor.setPower(direction);
            else intakeMotor.stop();

            // ----- TRANSFER (right trigger) -----
            boolean transferPressed = ballController.right_trigger > 0.1;
            if (transferPressed) transferMotor.setPower(direction);
            else transferMotor.stop();

            // ----- DPAD UP/DOWN (adjust flywheel speed once per press) -----
            boolean currentDpadUp = ballController.dpad_up;
            boolean currentDpadDown = ballController.dpad_down;

            if (currentDpadUp && !lastDpadUp) {
                targetTicksPerSecond += 50;
            }
            if (currentDpadDown && !lastDpadDown) {
                targetTicksPerSecond -= 50;
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // ----- OUTTAKE TOGGLE (right bumper) -----
            boolean currentOuttakeButton = ballController.right_bumper;
            if (currentOuttakeButton && !lastOuttakeButton) {
                outtakeOn = !outtakeOn;
            }
            lastOuttakeButton = currentOuttakeButton;

            if (outtakeOn) {
                outtakeMotors.start(targetTicksPerSecond);
            } else {
                outtakeMotors.stop();
            }

            outtakeMotors.update();

            // ----- TELEMETRY -----
            telemetry.addData("Target X", !isAprilTagVisible ? "N/A" : llResult.getTx());
            telemetry.addData("Target Y", !isAprilTagVisible ? "N/A" : llResult.getTy());
            telemetry.addData("Target Area", !isAprilTagVisible ? "N/A" : llResult.getTa());
            telemetry.addData("Botpose", !isAprilTagVisible ? "N/A" : llResult.getBotpose_MT2().toString());
            telemetry.addData("Target Distance", !isAprilTagVisible ? "N/A" : distance);
            telemetry.addLine("---------------");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Transfer", transferPressed ? "ON" : "OFF");
            telemetry.addData("Flywheel", outtakeOn ? "ON" : "OFF");
            telemetry.addData("Flywheel Target Ticks Per Second", targetTicksPerSecond);
            telemetry.addData("Flywheel Actual", outtakeMotors.getAverageVelocity());
            telemetry.addLine("---------------");
            telemetry.addData("Flywheel Error", outtakeMotors.getTargetVelocity() - outtakeMotors.getAverageVelocity());
            telemetry.addData("Reverse mode", reverseMode ? "ON" : "OFF");
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("Drive Controller", driveController == gamepad1 ? "User 1 (BLUE)" : "User 2 (RED)");
            telemetry.addData("Ball Controller", ballController == gamepad1 ? "User 1 (BLUE)" : "User 2 (RED)");
            telemetry.addData("Gamepad1 ID", gamepad1.getGamepadId());
            telemetry.addData("Gamepad2 ID", gamepad2.getGamepadId());
            telemetry.update();
        }
    }
}