package org.firstinspires.ftc.teamcode.motors;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDOuttakeMotors {

    private final PIDOuttakeMotor motor1;
    private final PIDOuttakeMotor motor2;

    public PIDOuttakeMotors(HardwareMap hardwareMap, String deviceName1, String deviceName2) {
        motor1 = new PIDOuttakeMotor(hardwareMap, deviceName1);
        motor2 = new PIDOuttakeMotor(hardwareMap, deviceName2);
    }

    /** Start flywheel at desired velocity */
    public void start(double ticksPerSecond) {
        motor1.start(ticksPerSecond);
        motor2.start(ticksPerSecond);
    }

    /** Stop the flywheel */
    public void stop() {
        motor1.stop();
        motor2.stop();
    }

    /** Call this in your loop() every tick to enforce velocity */
    public void update() {
        motor1.update();
        motor2.update();

    }

    /** Returns current velocity in ticks/sec */
    public double getAverageVelocity() {
        return (motor1.getVelocity() + motor2.getVelocity())/2;
    }

    /** Returns target velocity */
    public double getTargetVelocity() {
        return motor1.getTargetVelocity();
    }
}
