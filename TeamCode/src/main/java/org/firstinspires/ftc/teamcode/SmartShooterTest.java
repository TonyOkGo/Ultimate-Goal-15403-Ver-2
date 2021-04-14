package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SmartShooterTest", group="A")
public class SmartShooterTest extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();
        //initCamera();
        telemetry.addData("Initialization complete", "yup");
        telemetry.update();
        waitForStart();
        smartShoot(30);

    }
}
