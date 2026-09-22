using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// An extension class that adds extra methods for ArticulationBody components
    /// making some operations easier and quicker to write. 
    /// </summary>
    public static class ArticulationBodyExtensions
    {
        /// <summary>
        /// Set the X drive target for this ArticulationBody. If the provided value matches
        /// the current value, this call is ignored. 
        /// </summary>
        public static void SetXDriveTarget(this ArticulationBody ab, float target)
        {
            if (ab.xDrive.target != target)
            {
                var drive = ab.xDrive;
                drive.target = target;
                ab.xDrive = drive;
            }
        }

        /// <summary>
        /// Return the current X drive target of this ArticulationBody.
        /// </summary>
        public static float GetXDriveTarget(this ArticulationBody ab)
        {
            var drive = ab.xDrive;
            return drive.target;
        }

        /// <summary>
        /// Set the X drive target velocity for this ArticulationBody. If the
        /// provided value matches the current value, this call is ignored. 
        /// </summary>
        public static void SetXDriveTargetVelocity(this ArticulationBody ab, float targetVelocity)
        {
            if (ab.xDrive.targetVelocity != targetVelocity)
            {
                var drive = ab.xDrive;
                drive.targetVelocity = targetVelocity;
                ab.xDrive = drive;
            }
        }
        /// <summary>
        /// Return the current X drive target velocity of this ArticulationBody.
        /// </summary>
        public static float GetXDriveTargetVelocity(this ArticulationBody ab)
        {
            var drive = ab.xDrive;
            return drive.targetVelocity;
        }

        /// <summary>
        /// Set the X drive stiffness for this ArticulationBody. If the provided value
        /// matches the current value, this call is ignored. 
        /// </summary>
        public static void SetXDriveStiffness(this ArticulationBody ab, float stiffness)
        {
            if (ab.xDrive.stiffness != stiffness)
            {
                var drive = ab.xDrive;
                drive.stiffness = stiffness;
                ab.xDrive = drive;
            }
        }

        /// <summary>
        /// Set the X drive damping for this ArticulationBody. If the provided value
        /// matches the current value, this call is ignored. 
        /// </summary>
        public static void SetXDriveDamping(this ArticulationBody ab, float damping)
        {
            if (ab.xDrive.damping != damping)
            {
                var drive = ab.xDrive;
                drive.damping = damping;
                ab.xDrive = drive;
            }
        }

        /// <summary>
        /// Set the X drive lower limit for this ArticulationBody. If the provided value
        /// matches the current value, this call is ignored. 
        /// </summary>
        public static void SetXDriveLowerLimit(this ArticulationBody ab, float lowerLimit)
        {
            if (ab.xDrive.lowerLimit != lowerLimit)
            {
                var drive = ab.xDrive;
                drive.lowerLimit = lowerLimit;
                ab.xDrive = drive;
            }
        }

        /// <summary>
        /// Set the X drive upper limit for this ArticulationBody. If the provided value
        /// matches the current value, this call is ignored. 
        /// </summary>
        public static void SetXDriveUpperLimit(this ArticulationBody ab, float upperLimit)
        {
            if (ab.xDrive.upperLimit != upperLimit)
            {
                var drive = ab.xDrive;
                drive.upperLimit = upperLimit;
                ab.xDrive = drive;
            }
        }

        /// <summary>
        /// Return the root of this ArticulationBody chain.
        /// </summary>
        public static ArticulationBody GetRoot(this ArticulationBody ab)
        {
            if (ab.isRoot)
                return ab;

            var next = ab.transform.parent.GetComponentInParent<ArticulationBody>();

            if (next == null)
                return ab;

            return next.GetRoot();
        }

        /// <summary>
        /// Get the parent ArticulationBody of this ArticulationBody component.
        /// </summary>
        public static ArticulationBody GetParent(this ArticulationBody ab)
        {
            if (ab.transform.parent == null)
                return null;

            return ab.transform.parent.GetComponentInParent<ArticulationBody>();
        }
    }
}
