package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTag {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private HardwareMap hardwareMap;

    public static final boolean USE_WEBCAM = true;  // true = webcam, false = phone camera
    public static final int DESIRED_TAG_ID = -1;     // -1 = any tag

    // PID-ish gains
    public static final double DESIRED_DISTANCE = 12.0; // inches
    public static final double SPEED_GAIN  = 0.02;
    public static final double STRAFE_GAIN = 0.015;
    public static final double TURN_GAIN   = 0.01;

    public static final double MAX_AUTO_SPEED  = 0.5;
    public static final double MAX_AUTO_STRAFE = 0.5;
    public static final double MAX_AUTO_TURN   = 0.3;

    public AprilTag(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initAprilTag();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                    .enableLiveView(true)
                    .setCameraResolution(new Size(1280, 800))
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                    .enableLiveView(true)
                    .setCameraResolution(new Size(1280, 800))
                    .build();
        }
    }

    public void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            Thread.sleep(20);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // Get list of current detections
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    // Find desired tag (or any if DESIRED_TAG_ID = -1)
    public AprilTagDetection getDesiredTag() {
        for (AprilTagDetection detection : getDetections()) {
            if (detection != null && (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
                return detection;
            }
        }
        return null;
    }

    // Simple helper to compute auto drive powers from a detected tag
    public double[] computeDriveToTag(AprilTagDetection tag) {
        if (tag == null) return new double[]{0,0,0};

        double rangeError   = tag.ftcPose.range - DESIRED_DISTANCE;
        double headingError = tag.ftcPose.bearing;
        double yawError     = tag.ftcPose.yaw;

        double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        return new double[]{drive, strafe, turn};
    }
}
