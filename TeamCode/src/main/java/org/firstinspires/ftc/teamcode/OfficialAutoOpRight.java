package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name="OfficialAutoOpRight", group="Auto")
public class OfficialAutoOpRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        intakeMotor intakeMotor = new intakeMotor(hardwareMap);
        transferMotor transferMotor = new transferMotor(hardwareMap);
        OuttakeMotor outtakeMotor = new OuttakeMotor(hardwareMap);

        Action forward = drive.actionBuilder(startPose)
                .lineToX(24)
                .build();
        Action turnRight = drive.actionBuilder(startPose)
                .turnTo(45)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(forward);
        Actions.runBlocking(turnRight);

        outtakeMotor.start(1.0);
        wait(2500);
        intakeMotor.setPower(1.0);
        transferMotor.setPower(1.0);
    }
}
