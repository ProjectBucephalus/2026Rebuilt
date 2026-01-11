// package frc.robot.util.controlTransmutation;

// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import frc.robot.util.Conversions;

// public class PIDDriveTransmuter extends InputTransmuter
// {
//   private ObjectList objectList;
//   private final PIDController xController;
//   private final PIDController yController;
//   private Supplier<Translation2d> targetPosSup;
//   private Translation2d targetPos;

//   public PIDDriveTransmuter(double driveP, double driveI, double driveD)
//   {
//     xController = new PIDController(driveP, driveI, driveD);
//     yController = new PIDController(driveP, driveI, driveD);
//     objectList = new ObjectList();
//   }

//   @Override
//   public Translation2d process(Translation2d controlInput) 
//   {
//     targetPos = targetPosSup.get();

//     double throttleX = Conversions.clamp(xController.calculate(robotPos.getX(), targetPos.getX()));
//     double throttleY = Conversions.clamp(yController.calculate(robotPos.getY(), targetPos.getY()));

//     Translation2d throttleXY = new Translation2d(throttleX, throttleY);
//     return objectList.process(throttleXY);
//   }

//   public PIDDriveTransmuter withObjectList(ObjectList objectList)
//   {
//     this.objectList = objectList;
//     return this;
//   }

//     /**
//    * Sets the global robot position supplier for all field objects
//    * @param robotPosSupplier Translation2d Supplier for the robot position (not pose)
//    */
//   public PIDDriveTransmuter withTargetPoseSup(Supplier<Pose2d> targetPoseSup)
//   {
//     this.targetPosSup = () -> targetPoseSup.get().getTranslation();
//     return this;
//   }
// }