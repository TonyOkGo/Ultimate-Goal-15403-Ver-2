/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name="MecanumDrive", group="hello")
//@Disabled
public class MecanumDrive extends LinearOpMode {

    /* Declare OpMode members. */
    MecanumTeleopHardwareMap robot = new MecanumTeleopHardwareMap() ;


    static final double FORWARD_SPEED = 0.6;
    static double turnPower = 0.5;
    static double strafePower = 0.5;
    static double fwdBackPower = 0.5;
    static double leftFrontPower;
    static double leftBackPower;
    static double rightFrontPower;
    static double rightBackPower;


    @Override
    public void runOpMode() {


        robot.init(hardwareMap);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();




        while (opModeIsActive()) {

            fwdBackPower = gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = -gamepad1.right_stick_x;
            leftFrontPower = fwdBackPower - turnPower - strafePower;
            rightFrontPower = fwdBackPower + turnPower + strafePower;
            leftBackPower = fwdBackPower - turnPower + strafePower;
            rightBackPower = fwdBackPower + turnPower - strafePower;
            robot.leftfrontDrive.setPower(-leftFrontPower);
            robot.rightfrontDrive.setPower(-rightFrontPower);
            robot.leftbackDrive.setPower(-leftBackPower);
            robot.rightbackDrive.setPower(-rightBackPower);
            telemetry.addData("Speed", robot.rightbackDrive.getPower());
            telemetry.update();
        }

    }
}
