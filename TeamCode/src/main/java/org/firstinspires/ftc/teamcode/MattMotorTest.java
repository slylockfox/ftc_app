package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by matt on 11/12/16.
 */

@Autonomous(name="Matt Motor Test", group="Pushbot")
public class MattMotorTest extends LinearOpMode {

    //private DcMotor motorLeft, motorRight;
    MattHardwarePushbot     robot   = new MattHardwarePushbot();


    @Override
    public void runOpMode() throws InterruptedException {
        //motorLeft = hardwareMap.dcMotor.get("left_drive");
        //motorRight = hardwareMap.dcMotor.get("right_drive");
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            robot.rightMotor.setPower(.5);
        }

    }
}
