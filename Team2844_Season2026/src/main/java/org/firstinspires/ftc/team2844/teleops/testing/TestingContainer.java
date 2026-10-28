package org.firstinspires.ftc.team2844.teleops.testing;

import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.core.command.InstantCommand;
import com.vcs.valleylib.core.command.decorators.ParallelCommandGroup;
import com.vcs.valleylib.core.fsm.StateMachine;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.input.Trigger;

import org.firstinspires.ftc.team2844.autos.AutoPaths;
import org.firstinspires.ftc.team2844.commands.drive.GoToCmd;
import org.firstinspires.ftc.team2844.commands.turret.DefaultTurretCmd;
import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.PIDF;
import org.firstinspires.ftc.team2844.helpers.Subsystems;
import org.firstinspires.ftc.team2844.subsystems.ShooterSubsystem;

public class TestingContainer {
    public enum Cycle {DRIVE, PIDF, LINREG, TURRET}
    boolean nextState = false;
    boolean previousState = false;
    double velocity, hoodPos;
    int hoodMultiplier, velocityMultiplier, turretMultiplier, pidfMultiplier;
    double[] hoodArray, velocityArray, turretArray, pidfArray;

    int currentPIDF;
    ShooterSubsystem currentShooter;
    Trigger leftStickRight, leftStickLeft, leftStickUp, leftStickDown,
            rightStickLeft, rightStickRight, rightStickUp, rightStickDown;
    StateMachine<Cycle> testingMachine;
    Subsystems subsystems;
    DefaultTurretCmd defaultTurretCmd;
    Pose resetPose;

    /* ---------- Controller and Buttons ---------- */
    CommandGamepad driver;
    CommandGamepad operator;
    /*
    * Used Buttons on the Driver Controller
    * Bumpers: Switch to next and previous state
    * Left and Right Triggers: Throughput and Intake or Outtake
    * Right Stick: Multipliers for Velocity, Hood and Turret
    * Left Stick: PIDF modifiers and increase/decrease
    * A,B,X,Y: Turret Manual Pos
    * DPad: increase/decrease velocity, hood, turret; autoPark
    * Start: reset pose
    * Guide, Back, Mode: not used
    * */
    public TestingContainer(Subsystems subsystems, CommandGamepad gamepad1, CommandGamepad gamepad2){
        /* --- Set the Defaults --- */
        this.subsystems = subsystems;
        driver = gamepad1;
        operator = gamepad2;
        velocity = Constants.SHOOTER_IDLE;
        hoodPos = Constants.HOOD_DEFAULT;
        currentShooter = subsystems.nectar;
        currentPIDF = 0;


        resetPose = (Constants.alliance == Constants.BLUE_ALLIANCE)
                ?new Pose(144.0-(Constants.ROBOT_LENGTH/2.0), Constants.ROBOT_WIDTH/2.0,180.0)
                :new Pose(Constants.ROBOT_LENGTH/2.0, (144-Constants.ROBOT_WIDTH/2.0), 0.0);

        /* --- Arrays and Multipliers --- */
        hoodMultiplier = 0;
        velocityMultiplier = 0;
        turretMultiplier = 0;
        pidfMultiplier = 0;
        hoodArray = new double[]{0.01, 0.05, 0.1, 0.5};
        velocityArray = new double[]{10, 50, 100, 500};
        turretArray = new double[]{1.0, 5.0, 10.0, 50.0, 180.0};
        pidfArray = new double[]{0.005, 0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0, 50.0, 100.0, 1000.0};

        defaultTurretCmd = new DefaultTurretCmd(subsystems.turret, subsystems.vision);

        /* --- Left Stick Triggers --- */
        leftStickLeft = new Trigger(() -> operator.leftX()<= -0.75);
        leftStickRight = new Trigger(() -> operator.leftX() >= 0.75);
        leftStickDown = new Trigger(() -> operator.leftY()>= 0.75);
        leftStickUp = new Trigger(() -> operator.leftY() <= -0.75);


        /* --- Right Stick Triggers --- */
        rightStickLeft = new Trigger(() -> operator.rightX()<= -0.75);
        rightStickRight = new Trigger(() -> operator.rightX()>= 0.75);
        rightStickUp = new Trigger(() -> operator.rightY()<= -0.75);
        rightStickDown = new Trigger(() -> operator.rightY()>= 0.75);


        /* --- State Machine --- */
        testingMachine = new StateMachine<>(Cycle.DRIVE)
                //!DRIVE state
                .state(Cycle.DRIVE, new InstantCommand(()-> {}))

                //!PIDF State
                .onEnter(Cycle.PIDF, ()-> {
                    subsystems.nectar.idleShoot();
                    subsystems.pollen.idleShoot();
                })
                .state(Cycle.PIDF, Commands.run(()-> {
                    subsystems.nectar.shoot(velocity, hoodPos);
                    subsystems.pollen.shoot(velocity, hoodPos);
                }))
                .onExit(Cycle.PIDF, ()-> {
                    subsystems.nectar.shoot(0.0, 0.0);
                    subsystems.pollen.shoot(0.0, 0.0);
                })

                //!LINREG State
                .onEnter(Cycle.LINREG, ()-> {
                    subsystems.nectar.idleShoot();
                    subsystems.pollen.idleShoot();
                })
                .state(Cycle.LINREG, Commands.run(()-> {
                    subsystems.nectar.shoot(velocity, hoodPos);
                    subsystems.pollen.shoot(velocity, hoodPos);
                }))
                .onExit(Cycle.LINREG, ()-> {
                    subsystems.nectar.shoot(0.0, 0.0);
                    subsystems.pollen.shoot(0.0, 0.0);
                })

                //!TURRET State
                .state(Cycle.TURRET, new InstantCommand(() ->defaultTurretCmd.setTrack(true)))

                //?Transition forward
                .transition(Cycle.DRIVE,  Cycle.PIDF, () -> nextState)
                .transition(Cycle.PIDF, Cycle.LINREG, () -> nextState)
                .transition(Cycle.LINREG, Cycle.TURRET, () -> nextState)

                .onTransition((from, to) -> nextState = false)

                //?Transition backward
                .transition(Cycle.PIDF,  Cycle.DRIVE, () -> previousState)
                .transition(Cycle.LINREG, Cycle.PIDF, () -> previousState)
                .transition(Cycle.TURRET, Cycle.LINREG, () -> previousState)

                .onTransition((from, to) -> previousState = false);

        configureBindings();
    }

    public void configureBindings(){
        configureDefaults();
        configureDRIVE();
        configurePIDF();
        configureLINREG();
        configureTURRET();
    }

    public void configureDefaults(){
        subsystems.drive.setDefaultCommand(subsystems.drive.run(
                () -> subsystems.drive.driveFieldCentric(
                        -driver.leftY(),
                        -driver.leftX(),
                        -driver.rightX()
                )
        ));
        driver.rightBumper().onTrue(new InstantCommand(() -> nextState = true));
        driver.leftBumper().onTrue(new InstantCommand(() -> previousState = true));

        //! Resets the pose to the corner near the Human Player
        driver.guide().onTrue(new InstantCommand(() -> subsystems.drive.setPose(resetPose)));

        subsystems.turret.setDefaultCommand(defaultTurretCmd);
        defaultTurretCmd.setTrack(false);
    }

    public void configureDRIVE(){
        when(Cycle.DRIVE, driver.dpadDown()).whileTrue(
                new GoToCmd(subsystems.drive,
                        () -> AutoPaths.routeToPark(AutoPaths.getField(), subsystems.drive.getPose()), 0.8)
                        .andThen(Commands.run(() -> {}, subsystems.drive))
        );
    }

    public void configureTURRET(){
        //!Uses a single default command that can be modified to do different things so that the statemachine can't steal the requirement

        when(Cycle.TURRET, rightStickRight).onTrue(new InstantCommand(()->turretMultiplier = addCapped(turretMultiplier, turretArray.length-1)));
        when(Cycle.TURRET, rightStickLeft).onTrue(new InstantCommand(()->turretMultiplier = subtractCapped(turretMultiplier)));

        //? Sets the turret to defined positions
        when(Cycle.TURRET, driver.x()).onTrue(new InstantCommand(() -> {
            defaultTurretCmd.setDegrees(180.0);
            defaultTurretCmd.setManual(true);
        }));
        when(Cycle.TURRET, driver.a()).onTrue(new InstantCommand(() -> {
            defaultTurretCmd.setDegrees(0.0);
            defaultTurretCmd.setManual(true);
        }));
        when(Cycle.TURRET, driver.b()).onTrue(new InstantCommand(() -> {
            defaultTurretCmd.setDegrees(-180.0);
            defaultTurretCmd.setManual(true);
        }));
        //? Set manual or relative
        when(Cycle.TURRET, driver.dpadUp()).onTrue(new InstantCommand(() -> defaultTurretCmd.setManual(!defaultTurretCmd.getManual())));
        when(Cycle.TURRET, operator.dpadDown()).onTrue(new InstantCommand(() -> defaultTurretCmd.setRelative(!defaultTurretCmd.getRelative())));

        //? Set the turret to track mode
        when(Cycle.TURRET, operator.y()).onTrue(new InstantCommand(() -> {
            defaultTurretCmd.setManual(false);
            defaultTurretCmd.setTrack(true);
        }));

        //? Add and subtract from turret turn
        when(Cycle.TURRET, driver.dpadRight()).onTrue(new InstantCommand(()-> {
            defaultTurretCmd.setDegrees(defaultTurretCmd.getDegrees()+turretArray[turretMultiplier]);
            defaultTurretCmd.setManual(true);
        }));
        when(Cycle.TURRET, driver.dpadLeft()).onTrue(new InstantCommand(()-> {
            defaultTurretCmd.setDegrees(defaultTurretCmd.getDegrees()-turretArray[turretMultiplier]);
            defaultTurretCmd.setManual(true);
        }));
    }

    public void configurePIDF(){
        configureVelocity(Cycle.PIDF);
        configureHood(Cycle.PIDF);
        configureShooting(Cycle.PIDF);
        configureMultipliers(Cycle.PIDF);

        when(Cycle.PIDF, leftStickUp).onTrue(new InstantCommand(()-> addToPIDF(currentPIDF, pidfArray[pidfMultiplier], currentShooter))); //increase
        when(Cycle.PIDF, leftStickDown).onTrue(new InstantCommand(()-> subtractFromPIDF(currentPIDF, pidfArray[pidfMultiplier], currentShooter))); //decrease
        when(Cycle.PIDF, leftStickRight).onTrue(new InstantCommand(()-> pidfMultiplier = addCapped(pidfMultiplier, pidfArray.length-1))); //Increase multiplier
        when(Cycle.PIDF, leftStickLeft).onTrue(new InstantCommand(()-> pidfMultiplier = subtractCapped(pidfMultiplier))); //Decrease multiplier

        when(Cycle.PIDF, driver.x()).onTrue(new InstantCommand(()-> currentPIDF = subtractCapped(currentPIDF))); //Next PIDF
        when(Cycle.PIDF, driver.b()).onTrue(new InstantCommand(()-> currentPIDF = addCapped(currentPIDF, 4))); //Previous PIDF
        when(Cycle.PIDF, driver.a()).onTrue(new InstantCommand(()-> {
            if(currentShooter == subsystems.nectar){
                currentShooter = subsystems.pollen;
            } else {
                currentShooter = subsystems.nectar;
            }
        })); //switch shooter subsystem
    }

    public void configureLINREG(){
        configureVelocity(Cycle.LINREG);
        configureHood(Cycle.LINREG);
        configureShooting(Cycle.LINREG);
        configureMultipliers(Cycle.LINREG);
    }

    public void configureMultipliers(Cycle state){
        /* --- All of the Multiplier Stuff --- */
        Command multiplyVelocity = new InstantCommand(()->
            velocityMultiplier = addCapped(velocityMultiplier, velocityArray.length-1));
        Command divideVelocity = new InstantCommand(()->
            velocityMultiplier = subtractCapped(velocityMultiplier));

        Command multiplyHood = new InstantCommand(()->
            hoodMultiplier = addCapped(hoodMultiplier, hoodArray.length-1));
        Command divideHood = new InstantCommand(()->
            hoodMultiplier = subtractCapped(hoodMultiplier));

        when(state, rightStickLeft).onTrue(divideVelocity);
        when(state, rightStickRight).onTrue(multiplyVelocity);
        when(state, rightStickUp).onTrue(multiplyHood);
        when(state, rightStickDown).onTrue(divideHood);
    }


    /* --- Configure the Velocity and Hood --- */
    public void configureVelocity(Cycle state){
        when(state, driver.dpadRight()).onTrue(new InstantCommand(()->velocity+=velocityArray[velocityMultiplier]));
        when(state, driver.dpadLeft()).onTrue(new InstantCommand(()->velocity-=velocityArray[velocityMultiplier]));
    }

    public void configureHood(Cycle state){
        when(state, driver.dpadUp()).onTrue(new InstantCommand(()-> hoodPos +=hoodArray[hoodMultiplier]));
        when(state, driver.dpadDown()).onTrue(new InstantCommand(()-> hoodPos -=hoodArray[hoodMultiplier]));
    }

    public void configureShooting(Cycle state){
        //?Intake with right trigger, outtake with left trigger
        when(state, driver.rightTriggerButton(0.05)).whileTrue(new ParallelCommandGroup(
                subsystems.intake.startEnd(subsystems.intake::intake, subsystems.intake::stopIntake),
                subsystems.transfer.startEnd(subsystems.transfer::transfer, subsystems.transfer::stop)
        ));
        when(state, driver.leftTriggerButton(0.05)).whileTrue(new ParallelCommandGroup(
                subsystems.intake.startEnd(()->subsystems.intake.intake(-1.0), subsystems.intake::stopIntake),
                subsystems.transfer.startEnd(()->subsystems.transfer.transfer(-1.0), subsystems.transfer::stop)
        ));

        //?Opens the gate when it is closed, and closes when they are open
        when(state, driver.y()).onTrue(new InstantCommand(()->{
            if(subsystems.transfer.areGatesOpen()){
                subsystems.transfer.closeGates();
            } else {
                subsystems.transfer.openGates();
            }
        }));
    }

    public StateMachine<Cycle> getStateMachine(){
        return testingMachine;
    }

    /* --- Private Helpers --- */
    private int addCapped(int adder, int max){
        int temp = adder;
        if(temp+1 <= max){
            temp += 1;
        }
        return temp;
    }

    private int subtractCapped(int subtracter){
        int temp = subtracter;
        if(!(temp-1 < 0)){
            temp -= 1;
        }
        return temp;
    }

    private void addToPIDF(int pidf, double addAmount, ShooterSubsystem shooter){
        PIDF newPIDF = shooter.getPIDF();
        switch(pidf){
            case (0):
                newPIDF.kP += addAmount;
                break;
            case (1):
                newPIDF.kI += addAmount;
                break;
            case (2):
                newPIDF.kD += addAmount;
                break;
            case (3):
                newPIDF.kS += addAmount;
                break;
            case(4):
                newPIDF.kV += addAmount;
                break;
        }
        shooter.changePIDF(newPIDF);
    }

    private void subtractFromPIDF(int pidf, double subtractAmount, ShooterSubsystem shooter){
        PIDF newPIDF = shooter.getPIDF();
        switch(pidf){
            case (0):
                newPIDF.kP -= subtractAmount;
                break;
            case (1):
                newPIDF.kI -= subtractAmount;
                break;
            case (2):
                newPIDF.kD -= subtractAmount;
                break;
            case (3):
                newPIDF.kS -= subtractAmount;
                break;
            case(4):
                newPIDF.kV -= subtractAmount;
                break;
        }
        shooter.changePIDF(newPIDF);
    }

    public String getSelectedPIDF(){
        switch(currentPIDF){
            case (0):
                return "P";
            case (1):
                return "I";
            case (2):
                return "D";
            case (3):
                return "kS";
            case(4):
                return "kV";
            default:
                return "Invalid";
        }
    }

    private Trigger when(Cycle c, Trigger button){
        return button.and(testingMachine.in(c));
    }
}
