package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="auto", group="auto")
public class auto extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx cuca = null;
    private DcMotorEx juan = null;
    private DcMotorEx pancho = null;
    private DcMotorEx caballo = null;
    private DcMotorEx lapatrona = null;
    private DcMotorEx paquito = null;
    private DcMotorEx kiki = null;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        cuca = hardwareMap.get(DcMotorEx.class,"cuca");
        juan = hardwareMap.get(DcMotorEx.class,"juan");
        pancho = hardwareMap.get(DcMotorEx.class,"pancho");
        caballo = hardwareMap.get(DcMotorEx.class,"caballo");


        lapatrona = hardwareMap.get(DcMotorEx.class,"lapatrona");
        paquito = hardwareMap.get(DcMotorEx.class,"paquito");
        kiki = hardwareMap.get(DcMotorEx.class,"kiki");


        cuca.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        juan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pancho.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        caballo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kiki.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kiki.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        kiki.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lapatrona.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lapatrona.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lapatrona.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        paquito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        paquito.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        paquito.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        cuca.setDirection(DcMotorEx.Direction.REVERSE);
        juan.setDirection(DcMotorEx.Direction.FORWARD);
        caballo.setDirection(DcMotorEx.Direction.REVERSE);
        pancho.setDirection(DcMotorEx.Direction.FORWARD);

        lapatrona.setDirection(DcMotorEx.Direction.FORWARD);
        paquito.setDirection(DcMotorEx.Direction.REVERSE);
        kiki.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // FIRST MOVE - 1000 ticks
        kiki.setVelocity(4500);

        // WAIT FOR 3 SECONDS WHILE PELVIS/LAUNCH ACCELERATE
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            // Just wait for 3 seconds
        }

        // START PICKUP MOTOR AT 2500 VELOCITY
        paquito.setVelocity(-2500);

        // WAIT FOR 1 SECOND WHILE PICKUP ACCELERATES
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            // Just wait for 1 second
        }

        lapatrona.setVelocity(2500);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5.0) {
            // Just wait for 1 second

        }


            // STOP ALL MOTORS
        kiki.setVelocity(0);
        lapatrona.setVelocity(0);
        paquito.setVelocity(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}


