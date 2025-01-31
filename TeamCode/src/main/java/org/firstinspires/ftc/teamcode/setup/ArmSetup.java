package org.firstinspires.ftc.teamcode.setup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name = "Setup: Arm Setup", group = "Setup")
//@Disabled
public class ArmSetup extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    Arm arm = new Arm(this);
    double savepower = 0.;
    boolean apushed = false;
    boolean bpushed = false;

    @Override
    public void runOpMode() {

        arm.init();
        telemetry.addData(">", "Press Start to run.");
        telemetry.update();
        waitForStart();

        double slidepower = 0;

        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            double armpower = 0;
            int armposition = arm.getCurrentPosition();
	    if (gamepad1.a)
		{
		    if (!apushed)
			{
		    arm.MoveTo(1020);
		    apushed = true;
			}
		    
		}
	    else
		{
		    apushed = false;
		}
            telemetry.addData("Arm Position", "%10d", armposition);

	    if (arm.isBusy())
		{
            telemetry.addData("current armpower", "%5.2f", armpower);
		}
	    else
		{
            telemetry.addData(">", "not busy");
		}
            telemetry.addData(">", "Press A to move Arm to starting position");
		    
            telemetry.update();
        }
        // Set the servo to the new position and pause;
        //            rotator_servo.setPosition(position);
        //            sleep(CYCLE_MS);
        //idle();


    }
}
