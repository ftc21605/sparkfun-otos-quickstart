
package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Distance;
import org.firstinspires.ftc.teamcode.hardware.DistanceBack;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.Rotator;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@Autonomous(name = "auto specimen no park", group = "Wallace")
@Disabled
public class autospecimennopark extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    boolean skip_opencv = false;
    /* Declare OpMode members. */
    //    IntegratingGyroscope gyro;
    //NavxMicroNavigationSensor navxMicro;
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: REV Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // 4x and 5x gear boxes.
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI)*.85;
    static final double DRIVE_SPEED = -0.3;
    static final double TURN_SPEED = -0.2;


    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    static final double MAX_POS = 0.15;     // Maximum rotational position
    static final double MIN_POS = 0.5;     // Minimum rotational position

    //int DESIRED_TAG_ID = 5;    // Choose the tag you want to approach or set to -1 for ANY tag.
    DriveTrain drive_train = new DriveTrain(this);
    Arm arm = new Arm(this);
    Rotator rotator = new Rotator(this);
    Grabber grabber = new Grabber(this);
    Slide slide = new Slide(this);
    private Distance distance = new Distance(this);
    private DistanceBack distance_back = new DistanceBack(this);

    @Override
    public void runOpMode() {

         Pose2d posStart = new Pose2d(0, 0, 0);
        SparkFunOTOSDrive driveTrain = new SparkFunOTOSDrive(hardwareMap, posStart);
       drive_train.init();
        distance.init();
        distance_back.init();
       arm.init();
        slide.init();
       grabber.init();
       rotator.init();
       //	sleep(500);
	//	rotator.initpos();
	grabber.grab();

        // navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        // gyro = (IntegratingGyroscope)navxMicro;

        // here is what happens after we hit start
	arm.move(0.1);
        while (!isStarted() && !isStopRequested()) {
        }
	rotator.setposition(0.45);
	//	    moveRobot_forward(DRIVE_SPEED+0.1,0,5);
	//arm.Float();
	    arm.MoveTo(300,0.7);

	//	left_turn(92);
	//arm.Brake();
            // while (!gamepad1.a) {
            //     sleep(1);
            // }
	    slide.MoveTo(1400,0.7);
	    drive_train.moveRobot(DRIVE_SPEED,0);
	    telemetry.addData(">", "running straight");
	    //            while (distance.getDistanceMM() > 120) {
	telemetry.update();
            while (distance.getDistanceMM() > 180) {
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

	telemetry.addData("slide power after a: ", "%5.2f", slide.getPower());
        telemetry.update();
	sleep(1000);
	    long startpos = slide.getCurrentPosition();
	slide.MoveTo(750);
	while(slide.isBusy())
	    {
		sleep(1);
	    }
	sleep(500);
	grabber.release();
	drive_train.moveRobot_backward(DRIVE_SPEED,0,5);
	slide.MoveTo(60);
	//	slide.move(-0.07);
	arm.MoveTo(700);
rotator.setposition(0.45);
             while (!gamepad1.a) {
                  sleep(1);
              }

    }




}
