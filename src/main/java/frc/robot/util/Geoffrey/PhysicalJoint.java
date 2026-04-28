// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Geoffrey;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ejml.simple.SimpleMatrix;

/** Add your docs here. */
public interface PhysicalJoint {

  public static PhysicalJoint ground =
      new PhysicalJoint() {
        private final Transform3d kIdentityTransform = new Transform3d();
        private final SimpleMatrix kZeroVector = new SimpleMatrix(6, 1);

        @Override
        public void updateKinematics() {
          // Do nothing since this is the ground joint
        }

        @Override
        public Transform3d getForwardKinematic() {
          return kIdentityTransform;
        }

        @Override
        public SimpleMatrix getLocalVelocity() {
          return kZeroVector;
        }

        @Override
        public SimpleMatrix getLocalAcceleration() {
          return kZeroVector;
        }

        @Override
        public PhysicalJoint getParentJoint() {
          return null;
        }
      };

  public static PhysicalJoint getStructureJoint(Transform3d t) {
    return new PhysicalJoint() {
      public Transform3d transform = t;
      private final SimpleMatrix kZeroVector = new SimpleMatrix(6, 1);
      private PhysicalJoint base = null;

      @Override
      public void updateKinematics() {
        // Do nothing since this is the ground joint
      }

      @Override
      public Transform3d getForwardKinematic() {
        return transform;
      }

      @Override
      public SimpleMatrix getLocalVelocity() {
        return kZeroVector;
      }

      @Override
      public SimpleMatrix getLocalAcceleration() {
        return kZeroVector;
      }

      @Override
      public PhysicalJoint getParentJoint() {
        return base;
      }

      @Override
      public void setBase(PhysicalJoint b) {
        this.base = b;
      }
    };
  }

  public class kinematics {
    public Transform3d forwardKinematic;
    public SimpleMatrix localVelocity = new SimpleMatrix(6, 1);
    public SimpleMatrix localAcceleration = new SimpleMatrix(6, 1);
  }

  void updateKinematics();

  Transform3d getForwardKinematic();

  SimpleMatrix getLocalVelocity();

  SimpleMatrix getLocalAcceleration();

  PhysicalJoint getParentJoint();

  default void setBase(PhysicalJoint base) {}

  default Transform3d getGlobalPose() {
    PhysicalJoint parent = getParentJoint();
    if (parent != null) {
      return parent.getGlobalPose().plus(getForwardKinematic());
    } else {
      return getForwardKinematic();
    }
  }

  default Pose2d getGlobalPose2d() {
    return Pose3d.kZero.transformBy(getGlobalPose()).toPose2d();
  }

  default Transform3d getGlobalPose(double timestamp) {
    throw new UnsupportedOperationException("getGlobalPose(double timestamp) not implemented");
  }

  // local velocity instead of the parent rotation
  default SimpleMatrix getGlobalVelocity() {
    PhysicalJoint parent = getParentJoint();
    if (parent == null) return getLocalVelocity();

    // 1. Get Parent States (Global)
    SimpleMatrix vP6 = parent.getGlobalVelocity();
    double vpx = vP6.get(0), vpy = vP6.get(1), vpz = vP6.get(2);
    double wpx = vP6.get(3), wpy = vP6.get(4), wpz = vP6.get(5);

    // 2. Get Child Info
    Transform3d globalPose = getGlobalPose();
    Rotation3d rotC = globalPose.getRotation();

    // r is the vector from parent origin to child origin in global space
    Translation3d parentTrans = parent.getGlobalPose().getTranslation();
    double rx = globalPose.getX() - parentTrans.getX();
    double ry = globalPose.getY() - parentTrans.getY();
    double rz = globalPose.getZ() - parentTrans.getZ();

    // 3. Rotate Local Velocity into Global frame
    SimpleMatrix vL6 = getLocalVelocity();
    Translation3d vL_G = new Translation3d(vL6.get(0), vL6.get(1), vL6.get(2)).rotateBy(rotC);
    Translation3d wL_G = new Translation3d(vL6.get(3), vL6.get(4), vL6.get(5)).rotateBy(rotC);

    // 4. Calculate Global components using raw doubles
    // v_global = v_parent + (w_parent x r) + v_local_rotated
    // w x r cross product
    double wxr_x = wpy * rz - wpz * ry;
    double wxr_y = wpz * rx - wpx * rz;
    double wxr_z = wpx * ry - wpy * rx;

    SimpleMatrix result = new SimpleMatrix(6, 1);
    result.set(0, vpx + wxr_x + vL_G.getX());
    result.set(1, vpy + wxr_y + vL_G.getY());
    result.set(2, vpz + wxr_z + vL_G.getZ());
    // w_global = w_parent + w_local_rotated
    result.set(3, wpx + wL_G.getX());
    result.set(4, wpy + wL_G.getY());
    result.set(5, wpz + wL_G.getZ());
    return result;
  }

  default SimpleMatrix getGlobalAcceleration() {
    PhysicalJoint parent = getParentJoint();
    if (parent == null) return getLocalAcceleration();

    // 1. Parent Global Kinematics
    SimpleMatrix vP6 = parent.getGlobalVelocity();
    SimpleMatrix aP6 = parent.getGlobalAcceleration();
    double wpx = vP6.get(3), wpy = vP6.get(4), wpz = vP6.get(5);
    double apx = aP6.get(0), apy = aP6.get(1), apz = aP6.get(2);
    double alphaPx = aP6.get(3), alphaPy = aP6.get(4), alphaPz = aP6.get(5);

    // 2. Child Global Info
    Transform3d globalPose = getGlobalPose();
    Rotation3d rotC = globalPose.getRotation();
    Translation3d parentTrans = parent.getGlobalPose().getTranslation();
    double rx = globalPose.getX() - parentTrans.getX();
    double ry = globalPose.getY() - parentTrans.getY();
    double rz = globalPose.getZ() - parentTrans.getZ();

    // 3. Local Kinematics (Rotated to Global)
    SimpleMatrix vL6 = getLocalVelocity();
    SimpleMatrix aL6 = getLocalAcceleration();
    Translation3d vL_G = new Translation3d(vL6.get(0), vL6.get(1), vL6.get(2)).rotateBy(rotC);
    Translation3d wL_G = new Translation3d(vL6.get(3), vL6.get(4), vL6.get(5)).rotateBy(rotC);
    Translation3d aL_G = new Translation3d(aL6.get(0), aL6.get(1), aL6.get(2)).rotateBy(rotC);
    Translation3d alphaL_G = new Translation3d(aL6.get(3), aL6.get(4), aL6.get(5)).rotateBy(rotC);

    // 4. Compute Global Linear Accel using raw doubles
    // tangential = alphaP x r
    double tang_x = alphaPy * rz - alphaPz * ry;
    double tang_y = alphaPz * rx - alphaPx * rz;
    double tang_z = alphaPx * ry - alphaPy * rx;
    // wP x r (for centripetal)
    double wxr_x = wpy * rz - wpz * ry;
    double wxr_y = wpz * rx - wpx * rz;
    double wxr_z = wpx * ry - wpy * rx;
    // centripetal = wP x (wP x r)
    double cent_x = wpy * wxr_z - wpz * wxr_y;
    double cent_y = wpz * wxr_x - wpx * wxr_z;
    double cent_z = wpx * wxr_y - wpy * wxr_x;
    // coriolis = 2 * (wP x vL_G)
    double vlgx = vL_G.getX(), vlgy = vL_G.getY(), vlgz = vL_G.getZ();
    double cor_x = 2.0 * (wpy * vlgz - wpz * vlgy);
    double cor_y = 2.0 * (wpz * vlgx - wpx * vlgz);
    double cor_z = 2.0 * (wpx * vlgy - wpy * vlgx);

    SimpleMatrix result = new SimpleMatrix(6, 1);
    result.set(0, apx + tang_x + cent_x + aL_G.getX() + cor_x);
    result.set(1, apy + tang_y + cent_y + aL_G.getY() + cor_y);
    result.set(2, apz + tang_z + cent_z + aL_G.getZ() + cor_z);

    // 5. Compute Global Angular Accel
    // alphaP + alphaL_G + (wP x wL_G)
    double wlgx = wL_G.getX(), wlgy = wL_G.getY(), wlgz = wL_G.getZ();
    result.set(3, alphaPx + alphaL_G.getX() + (wpy * wlgz - wpz * wlgy));
    result.set(4, alphaPy + alphaL_G.getY() + (wpz * wlgx - wpx * wlgz));
    result.set(5, alphaPz + alphaL_G.getZ() + (wpx * wlgy - wpy * wlgx));

    return result;
  }
}
