package org.firstinspires.ftc.teamcode.IntoTheDeep24_25.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.PowerPlay23_24.TemplateJanx;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AprilTag Detection Only", group = "Autonomous")
public class AprilTags extends LinearOpMode {

    private VisionPortal visionPortal;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1; // Track any detected AprilTag
    private static final double TARGET_DISTANCE = 36; // Distance at which to initiate left turn
    private static final long TURN_DURATION_MS = 2000; // Duration to turn left (in milliseconds)
    private boolean hasTurned = false;

    @Override
    public void runOpMode() {
        robotInit();
        initAprilTag();

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (!hasTurned) {
                updateAprilTagDetection();
                moveRobot();
            } else {
                performLeftTurn(TURN_DURATION_MS);
            }
            telemetry.update();
        }

        stopRobot();
    }

    private void robotInit() {
        TemplateJanx janx = new TemplateJanx(hardwareMap);
        janx.wheelInit("frontRight", "backRight", "backLeft", "frontLeft");
        frontLeft = janx.fl;
        frontRight = janx.fr;
        backRight = janx.br;
        backLeft = janx.bl;
    }

    private void moveRobot() {
        if (desiredTag != null) {
            double distance = desiredTag.ftcPose.range;
            telemetry.addData("Tag Range", distance);

            if (distance > TARGET_DISTANCE) {  // Move forward until the target distance
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
                backRight.setPower(-1);
                backLeft.setPower(-1);
            } else {  // Start left turn once the distance is reached
                hasTurned = true;
            }
        } else {
            stopRobot();
        }
    }

    private void performLeftTurn(long durationMs) {
        // Set motors to turn left
        frontLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backRight.setPower(0.5);
        backLeft.setPower(-0.5);

        sleep(durationMs);  // Turn for the specified duration

        // Stop after turning
        stopRobot();
    }

    private void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    // Update the AprilTag detection and find the desired tag
    private void updateAprilTagDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections != null && !currentDetections.isEmpty()) {
            desiredTag = currentDetections.get(0); // Get the first detected tag
        } else {
            desiredTag = null;  // Reset if no tags are detected
        }
    }

    // Initialize the AprilTag detection system
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    // Method to set manual exposure for reducing motion blur
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }


        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}

