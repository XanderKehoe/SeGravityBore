using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System;
using VRage.Collections;
using VRage.Game.Components;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        string drillGroupName = "Drills";
        string rcName = "Remote Control";

        float desiredMiningSpeed = 0.5f;

        public abstract class State
        {
            protected StateMachineManager stateMachineManager;
            protected Program program;
            public readonly string name;

            public State(StateMachineManager stateMachineManager, Program program, string name)
            {
                this.program = program;
                this.stateMachineManager = stateMachineManager;
                this.name = name;
            }

            public virtual void Enter() { } // intentionally left empty, to be overwritten.

            public virtual void Update() { } // intentionally left empty, to be overwritten.

            public virtual void Exit() { } // intentionally left empty, to be overwritten.
        }

        public class StateMachineManager
        {
            public State currentState { get; private set; }
            protected Program program;

            public StateOff offState;
            public StatePointDown pointDownState;
            public StateMiningWithThrust miningWithThrustState;
            public StateMiningNoThrust miningWithNoThrustState;
            public StateReturningToSurface returningToSurfaceState;
            public StateFinishing finishingState;
            public StateError errorState;

            public StateMachineManager(Program program)
            {
                // set grid program (for debugging outputs with Echo and other uses).
                this.program = program;

                // initialize states
                offState = new StateOff(this, program, "offState");
                pointDownState = new StatePointDown(this, program, "pointDownState");
                miningWithThrustState = new StateMiningWithThrust(this, program, "miningWithThrustState");
                miningWithNoThrustState = new StateMiningNoThrust(this, program, "miningWithNoThrustState");
                returningToSurfaceState = new StateReturningToSurface(this, program, "returningToSurfaceState");
                finishingState = new StateFinishing(this, program, "finishingState");
                errorState = new StateError(this, program, "errorState");
            }

            public void Update()
            {
                currentState.Update();
            }

            public void ChangeState(State newState)
            {
                currentState.Exit();
                currentState = newState;
                currentState.Enter();
            }

            public void OverrideCurrentState(State newState)
            {
                currentState = newState;
            }
        }

        public class StateOff : State
        {
            public StateOff(StateMachineManager stateMachineManager, Program program, string name) : base(stateMachineManager, program, name) { }

            public override void Enter()
            {
                program.Echo("Entering State::OFF");
                program.Runtime.UpdateFrequency = UpdateFrequency.Update100;

                foreach (IMyThrust thruster in program.allThrusters)
                {

                }
            }

            public override void Update()
            {
                program.Echo("Executing State::OFF");
            }

            public override void Exit() { }
        }

        public class StatePointDown : State
        {
            public StatePointDown(StateMachineManager stateMachineManager, Program program, string name) : base(stateMachineManager, program, name) { }

            public override void Enter()
            {
                program.Echo("Entering State::POINTING_DOWN");
                program.Runtime.UpdateFrequency = UpdateFrequency.Update10;

                program.gravityDir = program.rc.GetNaturalGravity().Normalized();
                program.startingOrientation = program.rc.WorldMatrix.Forward.Normalized();
            }

            public override void Update()
            {
                program.Echo("Updating State::POINTING_DOWN");

                bool aligned = program.AlignOrientationWithVector(program.gravityDir);

                if (aligned)
                {
                    stateMachineManager.ChangeState(stateMachineManager.miningWithThrustState);
                }
            }

            public override void Exit() { }
        }

        public class StateMiningWithThrust : State
        {
            public StateMiningWithThrust(StateMachineManager stateMachineManager, Program program, string name) : base(stateMachineManager, program, name) { }

            public override void Enter()
            {
                program.Echo("Entering State::MINING_WITH_THRUST");

                // turn on all drills
                foreach (IMyShipDrill drill in program.drills)
                    drill.ApplyAction("OnOff_On");

                // turn on all connectors, have them set to collect all items, and then throw them out.
                foreach (IMyShipConnector connector in program.allConnectors)
                {
                    connector.ApplyAction("OnOff_On");
                    connector.CollectAll = true;
                    connector.ThrowOut = true;
                }

                program.startingPosition = program.rc.GetPosition();
            }

            public override void Update()
            {
                program.Echo("Updating State::MINING_WITH_THRUST");

                program.AlignOrientationWithVector(program.gravityDir);

                if (program.rc.GetShipSpeed() > program.desiredMiningSpeed)
                {
                    foreach (IMyThrust thruster in program.backThrusters)
                    {
                        thruster.ApplyAction("OnOff_On");
                    }
                }
                else
                {
                    foreach (IMyThrust thruster in program.backThrusters)
                    {
                        thruster.ApplyAction("OnOff_Off");
                    }
                }

                double distanceToChangeState = Vector3D.Distance((Vector3D)program.rc.GetPosition(), program.startingPosition);
                if (distanceToChangeState > program.depthWithThrust)
                {
                    stateMachineManager.ChangeState(stateMachineManager.miningWithNoThrustState);
                }
                else
                {
                    program.Echo("Current depth: " + distanceToChangeState + "m");
                    program.Echo("Turning off thrusters at " + program.depthWithThrust + "m");
                }
            }

            public override void Exit() { }
        }

        public class StateMiningNoThrust : State
        {
            public StateMiningNoThrust(StateMachineManager stateMachineManager, Program program, string name) : base(stateMachineManager, program, name) { }

            public override void Enter()
            {
                program.Echo("Entering State::MINING_NO_THRUST");

                foreach (IMyThrust thruster in program.allThrusters)
                {
                    thruster.ApplyAction("OnOff_Off");
                }

                program.Runtime.UpdateFrequency = UpdateFrequency.Update100;
            }

            public override void Update()
            {
                program.Echo("Updating State::MINING_NO_THRUST");

                double distanceToChangeState = Vector3D.Distance((Vector3D)program.rc.GetPosition(), program.startingPosition);
                if (distanceToChangeState > program.depthWithNoThrust)
                {
                    stateMachineManager.ChangeState(stateMachineManager.returningToSurfaceState);
                }
                else
                {
                    program.Echo("Current depth: " + distanceToChangeState + "m");
                    program.Echo("Switching to 'Return to Surface State' at " + program.depthWithNoThrust + "m");
                }
            }

            public override void Exit()
            {
                foreach (IMyThrust thruster in program.allThrusters)
                {
                    thruster.ApplyAction("OnOff_On");
                }
            }
        }

        public class StateReturningToSurface : State
        {
            double startPosDistanceFromPlanetCenter;
            public StateReturningToSurface(StateMachineManager stateMachineManager, Program program, string name) : base(stateMachineManager, program, name) { }

            public override void Enter()
            {
                program.Echo("Entering State::RETURNING_TO_SURFACE");

                program.Runtime.UpdateFrequency = UpdateFrequency.Update10;

                // turn off all drills
                foreach (IMyShipDrill drill in program.drills)
                    drill.ApplyAction("OnOff_Off");

                startPosDistanceFromPlanetCenter = Vector3D.Distance(program.planetCenter, program.startingPosition);
            }

            public override void Update()
            {
                // Code to run when in StateA
                program.Echo("Updating State::RETURNING_TO_SURFACE");

                program.AlignOrientationWithVector(program.gravityDir);

                if (program.rc.GetShipSpeed() > program.desiredMiningSpeed)
                {
                    foreach (IMyThrust thruster in program.backThrusters)
                    {
                        thruster.ThrustOverridePercentage = 0f;
                    }
                }
                else
                {
                    foreach (IMyThrust thruster in program.backThrusters)
                    {
                        thruster.ThrustOverridePercentage = 1f;
                    }
                }

                double currentDistanceFromPlanetCenter = Vector3D.Distance((Vector3D)program.rc.GetPosition(), program.planetCenter);
                if (currentDistanceFromPlanetCenter > startPosDistanceFromPlanetCenter)
                {
                    stateMachineManager.ChangeState(stateMachineManager.finishingState);
                }
                else
                {
                    program.Echo("Distance to surface: " + (startPosDistanceFromPlanetCenter - currentDistanceFromPlanetCenter) + "m");
                }

            }

            public override void Exit()
            {
                foreach (IMyThrust thruster in program.backThrusters)
                {
                    thruster.ThrustOverridePercentage = 0f;
                }
            }
        }

        public class StateFinishing : State
        {
            
            public StateFinishing(StateMachineManager stateMachineManager, Program program, string name) : base(stateMachineManager, program, name) { }

            public override void Enter()
            {
                program.Echo("Entering State::FINISHING");
            }

            public override void Update()
            {
                program.Echo("Updating State::FINISHING");

                Vector3D reversedGravityDir = program.rc.GetNaturalGravity().Normalized();
                reversedGravityDir.X *= -1;
                reversedGravityDir.Y *= -1;
                reversedGravityDir.Z *= -1;

                bool aligned = program.AlignOrientationWithSimultaneousVectors(program.startingOrientation, reversedGravityDir);

                if (aligned)
                {
                    stateMachineManager.ChangeState(stateMachineManager.offState);
                }
            }

            public override void Exit() 
            {
                foreach (IMyGyro gyro in program.gyros)
                    gyro.GyroOverride = false;
            }
        }

        public class StateError : State
        {
            public StateError(StateMachineManager stateMachineManager, Program program, string name) : base(stateMachineManager, program, name) { }

            public override void Enter()
            {
                program.Echo("Entering State::StateError");
                program.ini.Clear();
            }

            public override void Update()
            {
                program.Echo("In error state, fix the following issues and then recompile the script...");
                program.Echo(program.initializationErrors);
            }

            public override void Exit() { }
        }

        StateMachineManager stateMachineManager;

        float depthWithThrust = -1f;
        float depthWithNoThrust = -1f;

        List<IMyShipDrill> drills = new List<IMyShipDrill>();
        List<IMyGyro> gyros = new List<IMyGyro>();

        List<IMyThrust> allThrusters = new List<IMyThrust>();
        List<IMyThrust> backThrusters = new List<IMyThrust>();
        List<IMyThrust> forwardThrusters = new List<IMyThrust>();
        List<IMyThrust> upThrusters = new List<IMyThrust>();

        List<IMyShipConnector> allConnectors = new List<IMyShipConnector>();

        IMyRemoteControl rc;

        bool errorsPresent = false;
        string initializationErrors = "";

        private Vector3D gravityDir; // unit vector of natural gravity
        private Vector3D startingPosition; // starting position of digging.
        private Vector3D startingOrientation; // starting orientation of digging
        private Vector3D planetCenter; // center of planet

        MyIni ini = new MyIni();

        // Credit to Whiplash141 for this function.
        double VectorAngleBetween(Vector3D a, Vector3D b) //returns radians 
        {
            if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
                return 0;
            else
                return Math.Acos(MathHelper.Clamp(a.Dot(b) / Math.Sqrt(a.LengthSquared() * b.LengthSquared()), -1, 1));
        }

        // Credit to Whiplash141 for this function.
        void GetRotationAngles(Vector3D targetVector, IMyTerminalBlock reference, out double yaw, out double pitch)
        {
            var localTargetVector = Vector3D.TransformNormal(targetVector, MatrixD.Transpose(reference.WorldMatrix));
            var flattenedTargetVector = new Vector3D(localTargetVector.X, 0, localTargetVector.Z);

            yaw = VectorAngleBetween(Vector3D.Forward, flattenedTargetVector) * Math.Sign(localTargetVector.X); //right is positive
            if (Math.Abs(yaw) < 1E-6 && localTargetVector.Z > 0) //check for straight back case
                yaw = Math.PI;

            if (Vector3D.IsZero(flattenedTargetVector)) //check for straight up case
                pitch = MathHelper.PiOver2 * Math.Sign(localTargetVector.Y);
            else
                pitch = VectorAngleBetween(localTargetVector, flattenedTargetVector) * Math.Sign(localTargetVector.Y); //up is positive
        }

        // Credit to Whiplash141 for this function
        public static Vector3D SafeNormalize(Vector3D a)
        {
            if (Vector3D.IsZero(a))
                return Vector3D.Zero;

            if (Vector3D.IsUnit(ref a))
                return a;

            return Vector3D.Normalize(a);
        }

        // Credit to Whiplash141 for this function
        // This function is slightly modified to prioritize desiredUpVector over desiredForwardVector
        public static void GetRotationAnglesSimultaneous(Vector3D desiredForwardVector, Vector3D desiredUpVector, MatrixD worldMatrix, out double pitch, out double yaw, out double roll)
        {
            desiredForwardVector = SafeNormalize(desiredForwardVector);

            MatrixD transposedWm;
            MatrixD.Transpose(ref worldMatrix, out transposedWm);
            Vector3D.Rotate(ref desiredForwardVector, ref transposedWm, out desiredForwardVector);
            Vector3D.Rotate(ref desiredUpVector, ref transposedWm, out desiredUpVector);

            Vector3D leftVector = Vector3D.Cross(desiredUpVector, desiredForwardVector);
            Vector3D axis;
            double angle;
            if (Vector3D.IsZero(desiredUpVector) || Vector3D.IsZero(leftVector))
            {
                axis = new Vector3D(desiredForwardVector.Y, -desiredForwardVector.X, 0);
                angle = Math.Acos(MathHelper.Clamp(-desiredForwardVector.Z, -1.0, 1.0));
            }
            else
            {
                leftVector = SafeNormalize(leftVector);
                desiredUpVector = SafeNormalize(desiredUpVector);

                Vector3D forwardVector = SafeNormalize(Vector3D.Cross(leftVector, desiredUpVector));

                // Create matrix
                MatrixD targetMatrix = MatrixD.Zero;
                targetMatrix.Forward = forwardVector;
                targetMatrix.Left = leftVector;
                targetMatrix.Up = desiredUpVector;

                axis = new Vector3D(targetMatrix.M23 - targetMatrix.M32,
                                    targetMatrix.M31 - targetMatrix.M13,
                                    targetMatrix.M12 - targetMatrix.M21);

                double trace = targetMatrix.M11 + targetMatrix.M22 + targetMatrix.M33;
                angle = Math.Acos(MathHelper.Clamp((trace - 1) * 0.5, -1, 1));
            }

            if (Vector3D.IsZero(axis))
            {
                angle = desiredForwardVector.Z < 0 ? 0 : Math.PI;
                yaw = angle;
                pitch = 0;
                roll = 0;
                return;
            }

            axis = SafeNormalize(axis);
            // Because gyros rotate about -X -Y -Z, we need to negate our angles
            yaw = -axis.Y * angle;
            pitch = -axis.X * angle;
            roll = -axis.Z * angle;
        }

        // Credit to Whiplash141 for this function.
        void ApplyGyroOverride(double pitch_speed, double yaw_speed, double roll_speed, List<IMyGyro> gyro_list, IMyTerminalBlock reference, bool simulatenous)
        {
            Vector3D rotationVec;
            if (!simulatenous)
                rotationVec = new Vector3D(-pitch_speed, yaw_speed, roll_speed); //because keen does some weird stuff with signs
            else
                rotationVec = new Vector3D(pitch_speed, yaw_speed, roll_speed);
            var shipMatrix = reference.WorldMatrix;
            var relativeRotationVec = Vector3D.TransformNormal(rotationVec, shipMatrix);
            foreach (var thisGyro in gyro_list)
            {
                var gyroMatrix = thisGyro.WorldMatrix;
                var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(gyroMatrix));
                thisGyro.Pitch = (float)transformedRotationVec.X;
                thisGyro.Yaw = (float)transformedRotationVec.Y;
                thisGyro.Roll = (float)transformedRotationVec.Z;
                thisGyro.GyroOverride = true;
            }
        }

        // Sets initialization errors as present, and adds a new message for why initialization is failing.
        public void AddInitializationError(string str)
        {
            errorsPresent = true;
            initializationErrors += str + "\n";
        }

        // Obtains the total max effective thrust from the input list of thrusters.
        public float GetTotalThrustFromThrusterGroup(List<IMyThrust> thrustGroup)
        {
            float totalThrust = 0;
            foreach (IMyThrust thruster in upThrusters)
                totalThrust += thruster.MaxEffectiveThrust;

            return totalThrust;
        }

        // Aligns the grid with the target vector, returns true if aligned, false otherwise.
        public bool AlignOrientationWithVector(Vector3D targetVector)
        {
            double lockTolerance = 0.01;

            double yaw, pitch;
            bool yawLock = false;
            bool pitchLock = false;

            GetRotationAngles(targetVector, rc, out yaw, out pitch);
            ApplyGyroOverride(pitch, yaw, 0, gyros, rc, false);

            if (Math.Abs(yaw) < lockTolerance)
                yawLock = true;

            if (Math.Abs(pitch) < lockTolerance)
                pitchLock = true;

            //Echo(string.Format("Debugging - \ntargetVector [{0}]\nrc forward vector [{1}]\npitch: [{2}]\nyaw: [{3}]\npitch/yawLock: [{4}][{5}]", targetVector, rc.WorldMatrix.Forward, pitch, yaw, pitchLock, yawLock));
            //Runtime.UpdateFrequency = UpdateFrequency.None;

            if (yawLock && pitchLock)
                return true;
            else
                return false;
        }

        public bool AlignOrientationWithSimultaneousVectors(Vector3D targetForwardVector, Vector3D targetUpwardVector)
        {
            double lockTolerance = 0.01;

            double yaw, pitch, roll;
            bool yawLock = false;
            bool pitchLock = false;
            bool rollLock = false;

            GetRotationAnglesSimultaneous(targetForwardVector, targetUpwardVector, rc.WorldMatrix, out pitch, out yaw, out roll);
            ApplyGyroOverride(pitch, yaw, roll, gyros, rc, true);

            if (Math.Abs(yaw) < lockTolerance)
                yawLock = true;

            if (Math.Abs(pitch) < lockTolerance)
                pitchLock = true;

            if (Math.Abs(roll) < lockTolerance)
                rollLock = true;

            if (yawLock && pitchLock && rollLock)
                return true;
            else
                return false;
        }

        public Program()
        {
            stateMachineManager = new StateMachineManager(this);

            // Find and count drills
            IMyBlockGroup blockGroup = GridTerminalSystem.GetBlockGroupWithName(drillGroupName);
            if (blockGroup != null)
            {
                blockGroup.GetBlocksOfType(drills);
                
                if (drills.Count <= 0)
                    AddInitializationError(string.Format("Invalid number of drills found: {0} drills.", drills.Count));
            }
            else
                AddInitializationError(string.Format("No drill group of the name {0} found.", drillGroupName));

            // Find remote control block
            rc = GridTerminalSystem.GetBlockWithName(rcName) as IMyRemoteControl;
            if (rc == null)
                AddInitializationError(string.Format("No RC with the name {0} found.", rcName));

            // Find all thrusters and determine there's enough in atleast each direction (besides downwards since there is gravity)
            allThrusters = new List<IMyThrust>();
            GridTerminalSystem.GetBlocksOfType(allThrusters);
            if (allThrusters != null)
            {
                bool upThrusterFound = false;
                bool leftThrusterFound = false;
                bool rightThrusterFound = false;
                bool forwardThrusterFound = false;
                bool backwardThrusterFound = false;

                string upwardThrusterTag = " [UPWARD]";
                string leftThrusterTag = " [LEFTWARD]";
                string rightThrusterTag = " [RIGHTWARD]";
                string forwardThrusterTag = " [FORWARD]";
                string backwardThrusterTag = " [BACKWARD]";

                foreach (IMyThrust thruster in allThrusters)
                {
                    if (Base6Directions.GetFlippedDirection(thruster.Orientation.Forward) == rc.Orientation.Forward)
                    {
                        forwardThrusters.Add(thruster);
                        forwardThrusterFound = true;
                        if (!thruster.CustomName.Contains(forwardThrusterTag))
                            thruster.CustomName += forwardThrusterTag;
                    }
                    else if (Base6Directions.GetFlippedDirection(thruster.Orientation.Forward) == rc.Orientation.Up)
                    {
                        upThrusters.Add(thruster);
                        upThrusterFound = true;
                        if (!thruster.CustomName.Contains(upwardThrusterTag))
                            thruster.CustomName += upwardThrusterTag;
                    }
                    else if (Base6Directions.GetFlippedDirection(thruster.Orientation.Forward) == rc.Orientation.Left)
                    {
                        leftThrusterFound = true;
                        if (!thruster.CustomName.Contains(leftThrusterTag))
                            thruster.CustomName += leftThrusterTag;
                    }
                    else if (thruster.Orientation.Forward == rc.Orientation.Left)
                    {
                        rightThrusterFound = true;
                        if (!thruster.CustomName.Contains(rightThrusterTag))
                            thruster.CustomName += rightThrusterTag;
                    }
                    else if (thruster.Orientation.Forward == rc.Orientation.Forward)
                    {
                        backThrusters.Add(thruster);
                        backwardThrusterFound = true;
                        if (!thruster.CustomName.Contains(backwardThrusterTag))
                            thruster.CustomName += backwardThrusterTag;
                    }
                }

                if (!upThrusterFound)
                    AddInitializationError("No Upward Thruster Detected");

                if (!leftThrusterFound)
                    AddInitializationError("No Leftward Thruster Detected");

                if (!rightThrusterFound)
                    AddInitializationError("No Rightward Thruster Detected");

                if (!forwardThrusterFound)
                    AddInitializationError("No Forward Thruster Detected");

                if (!backwardThrusterFound)
                    AddInitializationError("No Backward Thruster Detected");

                float downwardForce = (float) (rc.CalculateShipMass().PhysicalMass * rc.GetTotalGravity().Length());

                if (upThrusterFound)
                {
                    float totalThrust = GetTotalThrustFromThrusterGroup(upThrusters);

                    if (totalThrust <= downwardForce)
                        AddInitializationError("Not enough upward thrust to support ship mass");
                }
                if (backwardThrusterFound)
                {
                    float totalThrust = GetTotalThrustFromThrusterGroup(backThrusters);

                    if (totalThrust <= downwardForce)
                        AddInitializationError("Not enough backward thrust to support ship mass");
                }
            }
            else
                AddInitializationError("Unable to create thruster list.");

            allConnectors = new List<IMyShipConnector>();
            GridTerminalSystem.GetBlocksOfType(allConnectors);
            if (allConnectors == null)
                AddInitializationError("Unable to create connectors list.");

            gyros = new List<IMyGyro>();
            GridTerminalSystem.GetBlocksOfType(gyros);
            if (gyros == null)
                AddInitializationError("Unable to create gyros list.");

            if (errorsPresent)
                stateMachineManager.ChangeState(stateMachineManager.errorState);
            else
            {
                ini.TryParse(Storage);
                LoadSave();
                Echo("Gravity Based Miner - Setup Complete. (OVERRIDING depthWithThrust depthWithNoThrust values)");
                depthWithThrust = 50f;
                depthWithNoThrust = 100f;
            }
        }
       
        // Saves data so if world unloads or server is saving, then program data is stored.
        public void Save()
        {
            ini.Clear();
            ini.Set("save", "depthWithNoThrust", depthWithNoThrust);
            ini.Set("save", "depthWithThrust", depthWithThrust);
            ini.Set("save", "gravityDir", gravityDir.ToString());
            ini.Set("save", "startingPosition", startingPosition.ToString());
            ini.Set("save", "startingOrientation", startingOrientation.ToString());
            ini.Set("save", "planetCenter", planetCenter.ToString());
            ini.Set("save", "currentState", stateMachineManager.currentState.name);

            Storage = ini.ToString();
        }

        // Restores saved program data from server save or world load.
        public void LoadSave()
        {
            TryLoadKeyFloat("depthWithNoThrust", out depthWithNoThrust);
            TryLoadKeyFloat("depthWithThrust", out depthWithThrust);
            TryLoadKeyVector3D("gravityDir", out gravityDir);
            TryLoadKeyVector3D("startingPosition", out startingPosition);
            TryLoadKeyVector3D("startingOrientation", out startingOrientation);
            TryLoadKeyVector3D("planetCenter", out planetCenter);

            if (ini.ContainsKey("save", "currentState"))
            {
                string currentStateStr = ini.Get("save", "currentState").ToString();

                switch (currentStateStr)
                {
                    case "offState":
                        stateMachineManager.OverrideCurrentState(stateMachineManager.offState);
                        break;
                    case "pointDownState":
                        stateMachineManager.OverrideCurrentState(stateMachineManager.pointDownState);
                        break;
                    case "miningWithNoThrustState":
                        stateMachineManager.OverrideCurrentState(stateMachineManager.miningWithNoThrustState);
                        break;
                    case "miningWithThrustState":
                        stateMachineManager.OverrideCurrentState(stateMachineManager.miningWithThrustState);
                        break;
                    case "returningToSurfaceState":
                        stateMachineManager.OverrideCurrentState(stateMachineManager.returningToSurfaceState);
                        break;
                    case "finishingState":
                        stateMachineManager.OverrideCurrentState(stateMachineManager.finishingState);
                        break;

                    default:
                        Echo("Unrecognized currentStateStr value: " + currentStateStr);
                        break;
                }
            }
            else
            {
                stateMachineManager.OverrideCurrentState(stateMachineManager.offState);
                stateMachineManager.currentState.Enter();
            }
        }

        // Attempts to load in float value from a key from storage, if it exists.
        public bool TryLoadKeyFloat(string key, out float val)
        {
            val = float.MinValue;
            if (ini.ContainsKey("save", key))
            {
                bool success = float.TryParse(ini.Get("save", key).ToString(), out val);
                if (success)
                {
                    return true;
                }
                else
                {
                    val = float.MinValue;
                    return false;
                }
            }

            return false;
        }

        // Attempts to load in Vector3D value from a key from storage, if it exists.
        public bool TryLoadKeyVector3D(string key, out Vector3D val)
        {
            val = new Vector3D();
            if (ini.ContainsKey("save", key))
            {
                bool success = Vector3D.TryParse(ini.Get("save", key).ToString(), out val);
                if (success)
                {
                    return true;
                }
                else
                {
                    val = new Vector3D();
                    return false;
                }

            }

            return false;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (stateMachineManager.currentState != stateMachineManager.errorState)
            {
                argument = argument.ToLower();
                string[] argSplit = argument.Split(',');
            
                if (argSplit[0] == "start")
                {
                    if (depthWithNoThrust > 0 && depthWithThrust > 0)
                    {
                        if (depthWithNoThrust < depthWithThrust)
                        {
                            Echo(string.Format("Starting Depth [{0}m] must be less than Total Depth [{1}m]", depthWithThrust, depthWithNoThrust));
                        }
                        else
                        {
                            bool planetFound = rc.TryGetPlanetPosition(out planetCenter);
                            if (planetFound)
                                stateMachineManager.ChangeState(stateMachineManager.pointDownState);
                            else
                                Echo("No planet found...");
                        }
                    }
                    else if (depthWithThrust <= 0)
                        Echo("Starting depth not set, use 'SetStartDepth,X' to set.");
                    else if (depthWithNoThrust <= 0)
                        Echo("Total mining depth not set, use 'SetTotalDepth,X', to set.");
                    else
                        Echo(string.Format("Unknown Error, script debugging required... [{0}] [{1}]", depthWithNoThrust, depthWithThrust));
                }
                else if (argSplit[0] == "settotaldepth")
                {
                    int val;
                    if (int.TryParse(argSplit[1], out val))
                    {
                        if (val > 0)
                        {
                            depthWithNoThrust = val;
                            Echo("Successfully set total depth to: " + val);
                        }
                        else
                        {
                            Echo(string.Format("Error: failed to set Total Depth with a value of '{0}'. Please use a positive numerical value", argSplit[1]));
                        }
                    }
                    else
                    {
                        Echo(string.Format("Error: failed to set Total Depth with a value of '{0}'. Please use a positive numerical value", argSplit[1]));
                    }
                }
                else if (argSplit[0] == "setstartdepth")
                {
                    int val;
                    if (int.TryParse(argSplit[1], out val))
                    {
                        if (val > 0)
                        {
                            depthWithThrust = val;
                            Echo("Successfully set start depth to: " + val);
                        }
                        else
                        {
                            Echo(string.Format("Error: failed to set Starting Depth with a value of '{0}'. Please use a positive numerical value", argSplit[1]));
                        }
                    }
                    else
                    {
                        Echo(string.Format("Error: failed to set Starting Depth with a value of '{0}'. Please use a positive numerical value", argSplit[1]));
                    }
                }

            }

            stateMachineManager.Update();
        }
    }
}