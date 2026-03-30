// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public interface PhysicalJoint {

    public static PhysicalJoint ground = new PhysicalJoint() {
        @Override
        public void updateKinematics() {
            //Do nothing since this is the ground joint
        }

        @Override
        public Transform3d getForwardKinematic() {
            return new Transform3d();
        }

        @Override
        public SimpleMatrix getLocalVelocity() {
            return new SimpleMatrix(6, 1);
        }

        @Override
        public SimpleMatrix getLocalAcceleration() {
            return new SimpleMatrix(6, 1);
        }

        @Override
        public PhysicalJoint getParentJoint() {
            return null;
        }
    };

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

    default Transform3d getGlobalPose(){
        PhysicalJoint parent = getParentJoint();
        if (parent != null) {
            return parent.getGlobalPose().plus(getForwardKinematic());
        } else {
            return getForwardKinematic();
        }
    }

    default SimpleMatrix getGlobalVelocity() {
        PhysicalJoint parent = getParentJoint();
        if (parent == null) return getLocalVelocity();

        // 1. Get Parent States (Global)
        SimpleMatrix vP6 = parent.getGlobalVelocity();
        Translation3d vP = new Translation3d(vP6.get(0), vP6.get(1), vP6.get(2));
        Translation3d wP = new Translation3d(vP6.get(3), vP6.get(4), vP6.get(5));

        // 2. Get Child Info
        Transform3d globalPose = getGlobalPose();
        Rotation3d rotC = globalPose.getRotation();
        
        // r is the vector from parent origin to child origin in global space
        Translation3d r = globalPose.getTranslation().minus(parent.getGlobalPose().getTranslation());

        // 3. Rotate Local Velocity into Global frame
        SimpleMatrix vL6 = getLocalVelocity();
        Translation3d vL_G = new Translation3d(vL6.get(0), vL6.get(1), vL6.get(2)).rotateBy(rotC);
        Translation3d wL_G = new Translation3d(vL6.get(3), vL6.get(4), vL6.get(5)).rotateBy(rotC);

        // 4. Calculate Global components
        // v_global = v_parent + (w_parent x r) + v_local_rotated
        Translation3d vGlobal = vP.plus(cross(wP, r)).plus(vL_G);
        // w_global = w_parent + w_local_rotated
        Translation3d wGlobal = wP.plus(wL_G);

        return pack(vGlobal, wGlobal);
    }

    default SimpleMatrix getGlobalAcceleration() {
        PhysicalJoint parent = getParentJoint();
        if (parent == null) return getLocalAcceleration();

        // 1. Parent Global Kinematics
        SimpleMatrix vP6 = parent.getGlobalVelocity();
        SimpleMatrix aP6 = parent.getGlobalAcceleration();
        Translation3d wP = new Translation3d(vP6.get(3), vP6.get(4), vP6.get(5));
        Translation3d aP = new Translation3d(aP6.get(0), aP6.get(1), aP6.get(2));
        Translation3d alphaP = new Translation3d(aP6.get(3), aP6.get(4), aP6.get(5));

        // 2. Child Global Info
        Transform3d globalPose = getGlobalPose();
        Rotation3d rotC = globalPose.getRotation();
        Translation3d r = globalPose.getTranslation().minus(parent.getGlobalPose().getTranslation());

        // 3. Local Kinematics (Rotated to Global)
        SimpleMatrix vL6 = getLocalVelocity();
        SimpleMatrix aL6 = getLocalAcceleration();
        Translation3d vL_G = new Translation3d(vL6.get(0), vL6.get(1), vL6.get(2)).rotateBy(rotC);
        Translation3d wL_G = new Translation3d(vL6.get(3), vL6.get(4), vL6.get(5)).rotateBy(rotC);
        Translation3d aL_G = new Translation3d(aL6.get(0), aL6.get(1), aL6.get(2)).rotateBy(rotC);
        Translation3d alphaL_G = new Translation3d(aL6.get(3), aL6.get(4), aL6.get(5)).rotateBy(rotC);

        // 4. Compute Global Linear Accel
        // Formula: aP + (alphaP x r) + wP x (wP x r) + aL_G + 2(wP x vL_G)
        Translation3d tangential = cross(alphaP, r);
        Translation3d centripetal = cross(wP, cross(wP, r));
        Translation3d coriolis = cross(wP, vL_G).times(2.0);
        Translation3d accGlobal = aP.plus(tangential).plus(centripetal).plus(aL_G).plus(coriolis);

        // 5. Compute Global Angular Accel
        // Formula: alphaP + alphaL_G + (wP x wL_G)
        Translation3d alphaGlobal = alphaP.plus(alphaL_G).plus(cross(wP, wL_G));

        return pack(accGlobal, alphaGlobal);
    }

    /** Manual 3D Cross Product for Translation3d */
    private static Translation3d cross(Translation3d a, Translation3d b) {
        return new Translation3d(
            a.getY() * b.getZ() - a.getZ() * b.getY(),
            a.getZ() * b.getX() - a.getX() * b.getZ(),
            a.getX() * b.getY() - a.getY() * b.getX()
        );
    }

    private static SimpleMatrix pack(Translation3d lin, Translation3d ang) {
        SimpleMatrix m = new SimpleMatrix(6, 1);
        m.set(0, lin.getX()); m.set(1, lin.getY()); m.set(2, lin.getZ());
        m.set(3, ang.getX()); m.set(4, ang.getY()); m.set(5, ang.getZ());
        return m;
    }
} 