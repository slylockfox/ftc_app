package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by matt on 11/12/16.
 */

@Autonomous(name="Motor Test", group="Caroline")
public class CarolineMotorTest extends LinearOpMode {

    //private DcMotor motorLeft, motorRight;
    HardwareCarolinePushbot robot   = new HardwareCarolinePushbot();


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
