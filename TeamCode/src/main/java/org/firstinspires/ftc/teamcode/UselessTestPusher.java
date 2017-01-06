package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.HardwareUselessbot.PUSHER_EXTEND;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.PUSHER_RETRACT;

/**
 * Created by matt on 11/12/16.
 */

@Autonomous(name="Test Pusher", group="Uselessbot")
public class UselessTestPusher extends LinearOpMode {

    //private DcMotor motorLeft, motorRight;
    HardwareUselessbot robot   = new HardwareUselessbot();


    @Override
    public void runOpMode() throws InterruptedException {
        //motorLeft = hardwareMap.dcMotor.get("left_drive");
        //motorRight = hardwareMap.dcMotor.get("right_drive");
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            robot.buttonPusher.setPosition(PUSHER_EXTEND);
            sleep(500);
            robot.buttonPusher.setPosition(PUSHER_RETRACT);
            sleep(500);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }
}
