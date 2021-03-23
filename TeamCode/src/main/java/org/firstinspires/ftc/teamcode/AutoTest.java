package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestAutonomous", group="A")
public class AutoTest extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();

        //encoderStrafe(STRAFE_SPEED,10,10,5,0);

        //encoderDrive(DRIVE_SPEED,-28,-28,5,0);
        while (opModeIsActive()) {
            int randomItems = ub_vision();
            telemetry.addData("Random Items ", randomItems);
            telemetry.update();
        }
    }
}
