package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestAutonomousNoVision", group="A")
public class Auto_Non_Vision extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        encoderStrafe(STRAFE_SPEED, -5,-5,10,0);
        //encoderDrive(DRIVE_SPEED, -25,-25,10,0);
        //funny scan vision
        //encoderStrafe(STRAFE_SPEED, -15,-24,10,0);
        //encoderDrive(DRIVE_SPEED, -30,-30,10,0);
        //encoderStrafe(STRAFE_SPEED, 22,22,10,0);
        //encoderDrive(DRIVE_SPEED, -7,-7,10,0);
        //encoderDrive(DRIVE_SPEED, -2.2,2.2,10,0);
        runtime.reset();
        utilmotor3.setPower(-1);
        sleep(900);
        while(runtime.seconds() < 5){
            utilmotor1.setPower(-1); utilmotor2.setPower(-1);
            utilmotor3.setPower(-1); crservo1.setPower(1);
            crservo2.setPower(-1);
        }
        utilmotor1.setPower(0); utilmotor2.setPower(0);
        utilmotor3.setPower(0); crservo1.setPower(0);
        crservo2.setPower(0);
        telemetry.addData("fsda","fsdaf");
        telemetry.update();
        //encoderStrafe(STRAFE_SPEED,10,10,10,0);
        //encoderDrive(DRIVE_SPEED,-28,-28,10,0);

    }
}
