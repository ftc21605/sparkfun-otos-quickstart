package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "AAAA Test: Arm Slide Move Position", group = "AAAA")
//@Disabled
public class ArmSlideMoveToPositionTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    Arm arm = new Arm(this);
    Slide slide = new Slide(this);
    double savepower = 0.;
    boolean apushed = false;
    boolean bpushed = false;

    @Override
    public void runOpMode() {

        arm.init();
	slide.init();
        telemetry.addData(">", "Press Start to run.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            int armposition = arm.getCurrentPosition();
            int slideposition = slide.getCurrentPosition();
	    if (gamepad1.a)
		{
		    if (!apushed)
			{
			    arm.Brake();
			    //			    arm.MoveTo(arm.getArmDropPosition(),0.2);
			    arm.MoveTo(1000,0.3);
		    apushed = true;
			}
		    
		}
	    else
		{
		    apushed = false;
		}
	    if (gamepad1.b)
		{
		    if (!bpushed)
			{
		    slide.MoveTo(slide.maxSlidePosition(arm.getArmDropPosition()));
		    bpushed = true;
			}
		    
		}
	    else
		{
		    bpushed = false;
		}
            telemetry.addData("Arm Position", "%10d", armposition);
            telemetry.addData("Slide Position", "%10d", slideposition);

	    if (arm.isBusy())
		{
		    telemetry.addData("current armpower", "%5.2f", arm.getPower());
		}
	    else
		{
            telemetry.addData(">", "arm not busy");
		}
	    if (slide.isBusy())
		{
		    telemetry.addData("current slide power", "%5.2f", slide.getPower());
		}
	    else
		{
            telemetry.addData(">", "slide not busy");
		}
            telemetry.addData(">", "Press A to move arm");
            telemetry.addData(">", "Press B to move slide");
		    
            telemetry.update();
        }
        // Set the servo to the new position and pause;
        //            rotator_servo.setPosition(position);
        //            sleep(CYCLE_MS);
        //idle();


    }
}
