package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Distance;
import org.firstinspires.ftc.teamcode.hardware.DistanceBack;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.Rotator;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "auto Aidan", group = "Wallace")
//@Disabled
public class autoaidan extends LinearOpMode {

    /* Declare OpMode members. */
    //    IntegratingGyroscope gyro;
    //NavxMicroNavigationSensor navxMicro;
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: REV Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // 4x and 5x gear boxes.
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) * .85;
    static final double DRIVE_SPEED = -0.3;
    static final double TURN_SPEED = -0.2;
    static final double MAX_POS = 0.15;     // Maximum rotational position
    static final double MIN_POS = 0.5;     // Minimum rotational position
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    // final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    // final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)
    private final ElapsedTime runtime = new ElapsedTime();
    boolean skip_opencv = false;
    //int DESIRED_TAG_ID = 5;    // Choose the tag you want to approach or set to -1 for ANY tag.
    DriveTrain drive_train = new DriveTrain(this);
    Arm arm = new Arm(this);
    Rotator rotator = new Rotator(this);
    Grabber grabber = new Grabber(this);
    Slide slide = new Slide(this);
    boolean run_with_distance_sensor = false;
    private final Distance distance = new Distance(this);
    private final DistanceBack distance_back = new DistanceBack(this);
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override
    public void runOpMode() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = .5;        // Desired forward power/speed (-1 to +1) +ve is forward
        double turn = .5;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise

        Pose2d posStart = new Pose2d(0, 0, 0);
        SparkFunOTOSDrive driveTrain = new SparkFunOTOSDrive(hardwareMap, posStart);
        drive_train.init();
        distance.init();
        distance_back.init();
        arm.init();
        slide.init();
        grabber.init();
        rotator.init();
        initAprilTag();
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        //        sleep(500);
        arm.move(0.1);
        //	sleep(500);
        //	rotator.initpos();
        grabber.grab();

        // navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        // gyro = (IntegratingGyroscope)navxMicro;
        boolean doall = false;
        // here is what happens after we hit start
        while (!isStarted() && !isStopRequested()) {
        }
        if (doall) {
            rotator.setposition(0.45);
            //	    moveRobot_forward(DRIVE_SPEED+0.1,0,5);
            //arm.Float();
            arm.MoveTo(300, 0.7);

            //	left_turn(92);
            //arm.Brake();
            // while (!gamepad1.a) {
            //     sleep(1);
            // }
            slide.MoveTo(1200, 0.7);
            drive_train.moveRobot(DRIVE_SPEED, 0);
            telemetry.addData(">", "running straight");
            //            while (distance.getDistanceMM() > 120) {
            telemetry.update();
            while (distance.getDistanceMM() > 150) {
                sleep(1);
            }
            drive_train.Stop();
            telemetry.addData("slide position: ", "%10d", slide.getCurrentPosition());
            //	slidebusy.run();
            telemetry.addData("arm position: ", "%10d", arm.getCurrentPosition());
            telemetry.update();
            //	    	grabber.release();
            // while (!gamepad1.a) {
            //     sleep(1);
            // }
            // while (!gamepad1.a) {
            //      sleep(1);
            //  }

            telemetry.addData("slide power after a: ", "%5.2f", slide.getPower());
            telemetry.update();
            sleep(1000);
            slide.MoveTo(400);
            while (slide.isBusy()) {
                sleep(1);
            }
            sleep(1000);
            grabber.release();
            drive_train.moveRobot_backward(DRIVE_SPEED, 0, 5);
            slide.MoveTo(60, 0.7);
            //	slide.move(-0.07);
            while (slide.isBusy()) {
                sleep(5);
            }
            arm.MoveTo(100);
            rotator.setposition(0.45);
            arm.Stop();
        // while (!gamepad1.a) {
        //      sleep(1);
        //  }
        Pose2d currentPose = driveTrain.pose;
        Action movement = driveTrain.actionBuilder(currentPose)
                .turn(Math.toRadians(-97.0))
                .build();

        // Execute the action
        Actions.runBlocking(movement);
        // while (!gamepad1.a) {
        //      sleep(1);
        //  }
	}
        targetFound = false;
        desiredTag = null;
        int detection_id = 0;
        int isleep = 0;
        drive_train.coast();
        telemetry.addData(">", "Press A to proceed");
        telemetry.update();
        while (!run_with_distance_sensor) {
            targetFound = false;
            while (!targetFound) {
                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            telemetry.addData("found tag ", "%d", detection.id);
                            telemetry.update();

                            break;  // don't look any further.
                        }
                    }
                }
                if (currentDetections == null || currentDetections.isEmpty()) {
                    telemetry.addData("distance, press A", "%5.2f", distance_back.getDistanceMM());
                    telemetry.update();

                    drive_train.Stop();

                    if (!gamepad1.a) {
                        sleep(1);
                    }

                }
                sleep(1); // target not found. sleep and try again
            }
            // Determine heading and range error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;

            // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double dist = distance_back.getDistanceMM();
            telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            telemetry.addData("Range/heading error", "Range %5.2f, Heading %5.2f", rangeError, headingError);
            telemetry.addData("desiredTag.ftcPose.range", "%5.2f", desiredTag.ftcPose.range);
            telemetry.addData("distance back", "%5.2f", dist);
            telemetry.update();
            if (dist < 450) { // 410 april tags out of vision
                run_with_distance_sensor = true;
                telemetry.addData("distance < 470, press A", "%5.2f", dist);
                telemetry.update();
                break;
            }
            // if (desiredTag.ftcPose.range < 0) {
            //     run_with_distance_sensor = true;
            // 	telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            // 	telemetry.addData("Range/heading error", "Range %5.2f, Heading %5.2f", rangeError, headingError);
            // 	telemetry.addData("Range/heading error", "Range %5.2f, Heading %5.2f", rangeError, headingError);
            // 	telemetry.addData("all done at", "Range %5.2f", desiredTag.ftcPose.range);
            // 	telemetry.update();
            // 	break;
            // }
            drive_train.moveRobot(drive, turn);
            sleep(10);
        }
        while (run_with_distance_sensor) {

            telemetry.addData("distance: ", "%5.2f", distance_back.getDistanceMM());
            telemetry.update();
            if (distance_back.getDistanceMM() > 30) {
                drive = MAX_AUTO_SPEED / 2.;
            } else {
                drive = 0;
                run_with_distance_sensor = false;
                drive_train.Stop();
                break;
            }
        }
        drive_train.Stop();
        telemetry.addData(">", "All done, press A");
        telemetry.update();
        if (!gamepad1.a) {
            sleep(1);
        }

        rotator.setposition(0.45);
	sleep(300);
	double currdist = distance_back.getDistanceMM();
	drive = -MAX_AUTO_SPEED/2.;
	drive_train.moveRobot(drive, turn);
	while(distance_back.getDistanceMM() < currdist+230)
	    {
		sleep(1);
	    }
	drive_train.off();
	arm.MoveTo(10);
	while(arm.isBusy())
	    {
		sleep(1);
	    }
	arm.move(0.01);
	grabber.release();
	//HERE is my new fancy *funcinctonal* code

        posStart = new Pose2d(0, 0, 0);
        Actions.runBlocking(driveTrain.actionBuilder(posStart)
                        .turn(Math.toRadians(90))
                        .build());




	slide.MoveTo(1100,1.);
	while(slide.isBusy())
	    {
		sleep(10);
	    }
	sleep(500);
	grabber.grab();
	slide.MoveTo(60,0.7);
	arm.MoveTo(arm.getArmDropPosition(),0.7);
        rotator.setposition(0.45); // rotate sample horizontal
	drive_train.left_turn_angle(150.);
	while(!gamepad1.a)
	    {
		sleep(1);
	    }
	slide.MoveTo(slide.maxSlidePosition(arm.getArmDropPosition()));
	currdist = distance.getDistanceMM();
	drive = -MAX_AUTO_SPEED/2.;
	drive_train.moveRobot(drive, turn);
	while(distance.getDistanceMM() > 170)
	    {
		sleep(1);
	    }
	drive_train.off();
	while(slide.isBusy())
	    {
		sleep(1);
	    }
		    grabber.release();
		    sleep(500);
	while(!gamepad1.a)
	    {
		sleep(1);
	    }
		    
    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /*
      Manually set the camera gain and exposure.
      This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }


}
