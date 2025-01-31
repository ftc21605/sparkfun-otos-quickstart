package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.OTOSKt.OTOSPoseToRRPose;
import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

@Config // Enable FTC Dashboard configuration
public class SparkFunOTOSDrive extends MecanumDrive {

    public static class Params {
        // Offset components accessible via the dashboard
        public double offsetX = -3.75;
        public double offsetY = 0.55;
        public double offsetHeading = -2.3555 - (Math.PI / 4);

        // Linear and angular scalars
        public double linearScalar = 100 / 98.3171;
        public double angularScalar = 0.9982;
    }

    public static Params PARAMS = new Params(); // Static instance for dashboard access

    public SparkFunOTOSCorrected otos;
    private Pose2d lastOtosPose = pose;

    // Track current offset values to detect changes
    private double currentOffsetX = PARAMS.offsetX;
    private double currentOffsetY = PARAMS.offsetY;
    private double currentOffsetHeading = PARAMS.offsetHeading;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    public SparkFunOTOSDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("OTOS_PARAMS", PARAMS);
        otos = hardwareMap.get(SparkFunOTOSCorrected.class, "otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // Apply initial offset from PARAMS
        updateSensorOffset();

        System.out.println("OTOS calibration beginning!");
        System.out.println(otos.setLinearScalar(PARAMS.linearScalar));
        System.out.println(otos.setAngularScalar(PARAMS.angularScalar));

        otos.setPosition(RRPoseToOTOSPose(pose));
        System.out.println(otos.calibrateImu(255, true));
        System.out.println("OTOS calibration complete!");
    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        // Check if offset parameters have changed
        if (currentOffsetX != PARAMS.offsetX || currentOffsetY != PARAMS.offsetY || currentOffsetHeading != PARAMS.offsetHeading) {
            updateSensorOffset();
        }

        if (lastOtosPose != pose) {
            otos.setPosition(RRPoseToOTOSPose(pose));
        }

        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);

        pose = OTOSPoseToRRPose(otosPose);
        lastOtosPose = pose;

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y), otosVel.h);
    }

    // Update the sensor's offset when parameters change
    private void updateSensorOffset() {
        SparkFunOTOS.Pose2D newOffset = new SparkFunOTOS.Pose2D(
                PARAMS.offsetX,
                PARAMS.offsetY,
                PARAMS.offsetHeading
        );
        otos.setOffset(newOffset);
        currentOffsetX = PARAMS.offsetX;
        currentOffsetY = PARAMS.offsetY;
        currentOffsetHeading = PARAMS.offsetHeading;
    }
}