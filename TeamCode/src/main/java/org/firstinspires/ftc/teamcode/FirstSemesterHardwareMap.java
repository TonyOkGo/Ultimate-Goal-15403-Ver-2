package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class FirstSemesterHardwareMap extends  HardwareMapUtil
{
    HardwareMap hwmap       = null;

    /* Public OpMode members. */
    public DcMotor  shooterMotor        = null;
    public DcMotor  wobbleGrabMotor     = null;
    public Servo    wobbleGrabServo     = null;
    //public DcMotor linearActuator = null;
    public DcMotor  leftfrontDrive   = null;
    public DcMotor  rightfrontDrive  = null;
    public DcMotor  leftbackDrive   = null;
    public DcMotor  rightbackDrive  = null;
    public DcMotor intakeMotor = null;
    public DcMotor pastaMotor = null;
    public CRServo pastaServo = null;
    public CRServo pastaServo2 = null;


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        shooterMotor = HardwareInitMotor("shootM", true);
        wobbleGrabMotor = HardwareInitMotor("wobbleG", true);
        //wobbleGrabServo = HardwareInitCRServo("wobbleS", false);
        leftfrontDrive = HardwareInitMotor("lfD", true);
        rightbackDrive = HardwareInitMotor("rbD", false);
        leftbackDrive = HardwareInitMotor("lbD", true);
        rightfrontDrive = HardwareInitMotor("rfD", false);
        intakeMotor = HardwareInitMotor("Intake", false);
        pastaMotor = HardwareInitMotor("pastaM", false);
        wobbleGrabServo = hwMap.get(Servo.class, "wobbleS");
        wobbleGrabServo.setPosition(0);
        pastaServo = hwMap.get(CRServo.class, "pastaS");
        pastaServo2 = hwMap.get(CRServo.class, "pastaS2");
        pastaServo.setDirection(CRServo.Direction.FORWARD);
        pastaServo2.setDirection(CRServo.Direction.FORWARD);
        pastaServo.setPower(0);
        pastaServo2.setPower(0);

    }
}

