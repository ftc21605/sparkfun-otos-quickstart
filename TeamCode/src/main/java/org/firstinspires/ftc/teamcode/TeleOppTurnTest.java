package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Turn Test")
public class TeleOppTurnTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d posStart = new Pose2d(0, 0, 0);
        SparkFunOTOSDrive driveTrain = new SparkFunOTOSDrive(hardwareMap, posStart);

        waitForStart();

        boolean lastAPressed = false;

        while (opModeIsActive()) {


            // Check for A button press

            boolean aPressed = gamepad1.a;
            if (aPressed && !lastAPressed) {
                // Get current pose and create new turn action
                Pose2d currentPose = driveTrain.pose;
                Action movement = driveTrain.actionBuilder(currentPose)
                        .lineToX(40)
                        .waitSeconds(2)
                        .turn(Math.toRadians(180))
                        .build();

                // Execute the action
                Actions.runBlocking(movement);
            }
            lastAPressed = aPressed;

            // Update telemetry
            Pose2d currentPose = driveTrain.pose;
            telemetry.addData("X Position", currentPose.position.x);
            telemetry.addData("Y Position", currentPose.position.y);
            telemetry.addData("Heading", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.update();
        }
    }
}