package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.Rotator;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "Faire TeleOp", group = "AWallace")
//@Disabled
public class FaireTeleOP extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    //    Grabber grabber = new Grabber(this);
    DriveTrain drive = new DriveTrain(this);
    Arm arm = new Arm(this);
    Rotator rotator = new Rotator(this);
    Slide slide = new Slide(this);
    Grabber grabber = new Grabber(this);
    boolean apushed = false;
    boolean bpushed = false;
    double savearmpower = 0.;
    double savepowerslide = 0.;
    boolean p1Xpushed = false;
    boolean p1Ypushed = false;
    boolean p1bpushed = false;
    boolean p1apushed = false;
    boolean override_arm_safety = false;
    boolean override_slide_safety = false;
    boolean auto_arm_slide = false;
    boolean auto_arm_slide_up = false;
    boolean auto_arm_slide_down = false;

    boolean armdown = false;
    boolean slidedown = false;
    boolean slidedownonly = false;
    boolean leftbumper = false;
    boolean rightbumper = false;
    boolean armup = false;
    boolean no_move_arm = false;

    @Override
    public void runOpMode() {
        drive.init();
        arm.init();
        grabber.init();
        rotator.init();
        //sleep(10);
        //	rotator.initpos();
        slide.init();
        telemetry.addData(">", "Press Start to run");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            boolean slowbot = false;
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = 0;
            double left_stick_val = gamepad1.left_stick_x;
            if (Math.abs(left_stick_val) > 0.2) {
                lateral = gamepad1.left_stick_x * 0.8;
            }
            double yaw = gamepad1.right_stick_x * 0.6;
            double armpower = 0;  // Note: pushing stick forward gives negative value
            int armposition = arm.getCurrentPosition();
            int slideposition = slide.getCurrentPosition();
            double powerslide = 0;//-gamepad1.right_stick_y;  // Note: pushing stick forward gives negative value
            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                if (gamepad1.right_trigger > 0) {
                    if (slideposition < 1000) {
                        powerslide = gamepad1.right_trigger;
                    }
                } else {
                    powerslide = -gamepad1.left_trigger;
                }
            }
            // if (Math.abs(gamepad1.left_stick_x) > 0.8) {
            //     if (Math.abs(gamepad1.left_stick_y) < 0.3) {
            //         lateral = gamepad1.left_stick_x*0.8;
            //     }
            // }
            if (gamepad2.a) {
                axial = 0;
                lateral = 0;
                yaw = 0;
            }
            drive.driveRobot(axial * 0.7, lateral, yaw * 0.9);
            if (gamepad1.y && !gamepad2.a) {
                grabber.grab();
            }
            if (gamepad1.x && !gamepad2.a) {
                grabber.release();
            }
            if (gamepad1.a && !gamepad2.a) {
                if (!apushed) {
                    //                    rotator.rotate_left();
                    rotator.setposition(0.45);
                    apushed = true;
                }
            } else {
                apushed = false;
            }
            if (gamepad1.b && !gamepad2.a) {
                if (!bpushed) {
                    rotator.rotate_right();
                    bpushed = true;
                }
            } else {
                bpushed = false;
            }

            if (gamepad1.left_bumper && !gamepad2.a) {
                if (!leftbumper) {
                    slide.Float();
                    slide.move(-0.7);
                    leftbumper = true;
                    slidedown = true;
                    auto_arm_slide = true;
                    auto_arm_slide_down = true;
                    auto_arm_slide_up = false;
                }

            } else {
                leftbumper = false;
            }

            if (gamepad1.right_bumper && !gamepad2.a) {
                if (!rightbumper) {
                    arm.Brake();
                    arm.MoveTo(arm.getArmDropPosition() - 400, 1.);
                    rotator.setposition(0.45); // rotate sample horizontal
                    rightbumper = true;
                    armup = true;
                    auto_arm_slide_down = false;
                    auto_arm_slide = true;
                    auto_arm_slide_up = true;
                }

            } else {
                rightbumper = false;
            }


            if (armup && armposition > 500) {
                slide.MoveTo(1800, 1.);
                armup = false;
                telemetry.addData(">", "should move slide Press dpad_up to continue");

                telemetry.update();

            }


            if (slowbot) {
                drive.driveRobotSlow(axial, lateral, yaw);
            }

            if (Math.abs(armpower) > 0.05) {
                savearmpower = armpower;
            }

            if (armposition >= arm.getArmMaxPosition()) {
                armpower = Math.min(armpower, 0);
            }
            if (armposition <= 60 && !override_arm_safety) {
                armpower = Math.max(armpower, 0);
            }

            if (Math.abs(powerslide) > 0.05) {
                savepowerslide = powerslide;
            }
            if (slideposition >= slide.maxSlidePosition(armposition)) {
                powerslide = Math.min(powerslide, 0);
            }
            if (slideposition <= 60 && !override_slide_safety) {
                powerslide = Math.max(powerslide, 0);
            }

            if (armposition > arm.getArmSlowPosition() && slideposition > 2000) {
                armpower = Math.min(armpower, 0.2);
            }
            if (armdown && armposition < 100) {
                arm.Stop();
                armdown = false;
                auto_arm_slide_down = false;
            }
            if (slidedown && slideposition < 60) {
                slide.Stop();
                slidedown = false;
                arm.Float();
                arm.move(-0.7);
                armdown = true;
            }
            if (auto_arm_slide && !slide.isBusy() && !arm.isBusy() && !auto_arm_slide_up && !auto_arm_slide_down) {
                auto_arm_slide = false;
            }
            if (!auto_arm_slide && !no_move_arm) {
                slide.move(powerslide);
                arm.move(armpower);
            }
            // if (gamepad2.right_trigger > 0)
            // 	{
            //
            // 	}
            // else
            // 	{
            // 	    slide.Stop();
            // 	}
            // if (gamepad2.left_trigger > 0)
            // 	{
            // 	    slide.move(gamepad2.right_trigger);
            // 	}
            // else
            // 	{
            // 	    slide.Stop();
            // 	}
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("rotator pos:", "%5.2f", rotator.currpos());
            telemetry.addData("armpos:", "%10d", armposition);
            telemetry.addData("slidepos:", "%10d", slideposition);
            telemetry.addData(">", "Press a to turn left.");
            telemetry.addData(">", "Press b to turn right.");
            telemetry.addData(">", "Press X to grab.");
            telemetry.addData(">", "Press y to release.");
            telemetry.addData(">", "Press left trigger to position arm");
            telemetry.addData(">", "Press right trigger to lock slide");
            telemetry.addData(">", "Use DPad for fine positioning of robot");
            telemetry.addData(">", "Use Bumpers to fine rotation of robot");
            telemetry.addData(">", "Press gamepad1 x to reset Arm encoder");
            telemetry.addData(">", "Press gamepad1 y to reset Slide encoder");


            telemetry.update();

            // Set the servo to the new position and pause;
            //            rotator_servo.setPosition(position);
            //            sleep(CYCLE_MS);
            //idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
