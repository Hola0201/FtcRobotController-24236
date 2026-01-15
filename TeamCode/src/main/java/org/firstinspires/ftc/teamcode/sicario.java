package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="sicario", group="sicario")
public class sicario extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
   // private DcMotorEx cuca = null;
    //private DcMotorEx juan = null;
    //private DcMotorEx pancho = null;
    //private DcMotorEx caballo = null;
    //private DcMotorEx lapatrona = null;
    private DcMotorEx paquito = null;
    private DcMotorEx kiki = null;


    private DcMotorEx hondo = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //cuca = hardwareMap.get(DcMotorEx.class,"cuca");
        //juan = hardwareMap.get(DcMotorEx.class,"juan");
        //pancho = hardwareMap.get(DcMotorEx.class,"pancho");
        //caballo = hardwareMap.get(DcMotorEx.class,"caballo");


        //lapatrona = hardwareMap.get(DcMotorEx.class,"lapatrona");
        paquito = hardwareMap.get(DcMotorEx.class,"paquito");
        kiki = hardwareMap.get(DcMotorEx.class,"kiki");
        hondo = hardwareMap.get(DcMotorEx.class,"hondo");


        //cuca.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //juan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //pancho.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //caballo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kiki.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kiki.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        kiki.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //lapatrona.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lapatrona.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //lapatrona.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        paquito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        paquito.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        paquito.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hondo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hondo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hondo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //cuca.setDirection(DcMotorEx.Direction.REVERSE);
        //juan.setDirection(DcMotorEx.Direction.FORWARD);
        //caballo.setDirection(DcMotorEx.Direction.REVERSE);
        //pancho.setDirection(DcMotorEx.Direction.FORWARD);

        //lapatrona.setDirection(DcMotorEx.Direction.FORWARD);
        paquito.setDirection(DcMotorEx.Direction.REVERSE);
        kiki.setDirection(DcMotorSimple.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        double chasisMax = 2500;
        double shotMax = 4500;

        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        double lf = -gamepad1.left_stick_x;

        //double lapatronaP = gamepad2.left_trigger*shotMax;
        double paquitoP = gamepad2.left_stick_y*chasisMax;

        double kikiP = gamepad2.right_trigger-gamepad2.left_trigger;



        if (gamepad2.y) {
          //  lapatrona.setVelocity(-shotMax);  // Mover en reversa
        } else {
            //lapatrona.setVelocity(lapatronaP);  // Control normal con left trigger
        }

        if (gamepad2.a) {
            int hondoTarget = hondo.getCurrentPosition()+100;
            hondo.setTargetPosition(hondoTarget);
            hondo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            hondo.setPower(0.5);


        }
        double cucaP = Range.clip(drive+lf+turn,-1,1);
        double juanP = Range.clip(drive-lf-turn,-1,1);
        double panchoP = Range.clip(drive+lf-turn,-1,1);
        double caballoP = Range.clip(drive-lf+turn,-1,1);

        //cuca.setVelocity(cucaP*chasisMax);
        //juan.setVelocity(juanP*chasisMax);
        //pancho.setVelocity(panchoP*chasisMax);
        //caballo.setVelocity(caballoP*chasisMax);
        //lapatrona.setVelocity(lapatronaP);
        kiki.setPower(kikiP);
        paquito.setPower(paquitoP);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)");

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}