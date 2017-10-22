package org.firstinspires.ftc.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum Drive Test", group = "Prototype")

public class MecanumTeleop extends OpMode {
    private MecanumRobot robot = null;
    private Controller g1, g2;
    Servo claw= hardwareMap.get(Servo.class, "claw_servo");
    DcMotor extender= hardwareMap.get(DcMotor.class, "linearslide_motor");
    DcMotor arm = hardwareMap.get(DcMotor.class, "arm_drive");
    private boolean debug_mode = false;
    private double armPower, extenderPower;
    private double clawPosition = 1.0;


    @Override
    public void init() {
        robot = new MecanumRobot(hardwareMap, telemetry);

        g1 = new Controller(gamepad1);
        g2 = new Controller(gamepad2);
    }

    @Override
    public void init_loop() {
        g1.update();
        if (g1.AOnce()) {
            debug_mode = ! debug_mode;
        }
        telemetry.addData("Debug? (a)", debug_mode ? "on" : "off");
        telemetry.addData("Ready?", "YES.");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.onStart();
    }

    @Override
    public void stop() {
        robot.onStop();
    }

    private void g1Loop(Controller g) {
        DriveHelper.drive(g, robot);
    }


    @Override
    public void loop() {
        g1.update();
        g2.update();
        robot.loop();
        g1Loop(g1);
        armPower = -gamepad2.left_stick_y*0.25;
        extenderPower = -gamepad2.right_stick_y*0.5;
        if(gamepad2.a){
            if(clawPosition>0.6){
                clawPosition-=0.1;
            }
        }
        if(gamepad2.b){
            if(clawPosition<1.0){
                clawPosition+=0.1;
            }
        }
        arm.setPower(armPower);
        //robot.moveClaw(clawPosition);
        extender.setPower(extenderPower);
        if (debug_mode) {
            robot.updateSensorTelemetry();
            telemetry.update();
        }
    }
}
