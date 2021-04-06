package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousVision", group="A")
public class Auto_Vision extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        int amountofrings = 0;
        initAuto();
        waitForStart();
        //Line up to the rings in order to scan
        encoderStrafe(STRAFE_SPEED, -5,-5,10,0);
        encoderDrive(DRIVE_SPEED, -25,-25,10,0);
        //funny scan vision
        if(amountofrings == 0){

        }
        if(amountofrings ==1){

        }
        if(amountofrings == 2){
            
        }
        encoderStrafe(STRAFE_SPEED, -15,-24,10,0);
        encoderDrive(DRIVE_SPEED, -30,-30,10,0);
        encoderStrafe(STRAFE_SPEED, 22,22,10,0);
        encoderDrive(DRIVE_SPEED, -7,-7,10,0);
        encoderDrive(DRIVE_SPEED, -2.2,2.2,10,0);
        shoot(5);
        telemetry.addData("fsda","fsdaf");
        telemetry.update();

    }
}
