/**
 * Copyright (c) 2020-2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Utilities;
using UnityEngine;
using Simulator.Sensors.UI;
using System.Collections.Generic;
using Simulator.Map;
using System;
using System.Collections;
using Simulator.Analysis;

namespace Simulator.Sensors
{
    [SensorType("Vehicle Odometry", new[] { typeof(VehicleOdometryData) })]
    public partial class VehicleOdometrySensor : SensorBase
    {
        [SensorParameter]
        [Range(1f, 100f)]
        public float Frequency = 10.0f;

        private uint SendSequence;
        private float NextSend;

        private BridgeInstance Bridge;
        private Publisher<VehicleOdometryData> Publish;

        private IVehicleDynamics Dynamics;
        private IAgentController Controller;
        private VehicleActions Actions;
        private VehicleLane Lane;

        private VehicleOdometryData msg;

        [AnalysisMeasurement(MeasurementType.Distance)]
        private float Distance = 0f;
        private Vector3 PrevPos = new Vector3(0f, 0f, 0f);

        [AnalysisMeasurement(MeasurementType.RightHandPos)]
        private Vector3 StartPosition;
        [AnalysisMeasurement(MeasurementType.RightHandPos)]
        private Vector3 EndPosition => transform.position;

        private MapTrafficLane SpeedViolationLane;
        private float SpeedViolationCount = 0f;
        private float SpeedViolationMin = 0f;
        private float SpeedViolationMax = 0f;

        public override SensorDistributionType DistributionType => SensorDistributionType.LowLoad;

        private void Awake()
        {
            Dynamics = GetComponentInParent<IVehicleDynamics>();
            Controller = GetComponentInParent<IAgentController>();
            Lane = GetComponentInParent<VehicleLane>();
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            if (bridge.Plugin.Factory is Bridge.Cyber.CyberBridgeFactory)
            {
                return;
            }

            Bridge = bridge;
            Publish = bridge.AddPublisher<VehicleOdometryData>(Topic);
        }

        public void Start()
        {
            NextSend = Time.time + 1.0f / Frequency;
            StartPosition = transform.position;
        }

        public void Update()
        {
            // distance analysis
            Distance += Vector3.Distance(transform.position, PrevPos) / 1000;
            PrevPos = transform.position;

            // traffic speed violation
            float speed = Dynamics.Velocity.magnitude;
            if (speed > Lane?.CurrentMapLane?.speedLimit)
            {
                SpeedViolationLane = Lane.CurrentMapLane;
                SpeedViolationCount += Time.fixedDeltaTime;
                UpdateMinMax(speed, ref SpeedViolationMin, ref SpeedViolationMax);
            }
            else
            {
                if (SpeedViolationCount > 0 && SpeedViolationLane != null)
                {
                    SpeedViolationEvent(Controller.GTID, SpeedViolationLane);
                }
                SpeedViolationCount = 0f;
                SpeedViolationMax = 0f;
                SpeedViolationLane = null;
            }

            msg = new VehicleOdometryData()
            {
                Time = SimulatorManager.Instance.CurrentTime,
                Speed = speed,
                SteeringAngleFront = Dynamics.WheelAngle,
                SteeringAngleBack = 0f,
            };

            if (Time.time < NextSend)
            {
                return;
            }
            NextSend = Time.time + 1.0f / Frequency;

            if (Bridge != null && Bridge.Status == Status.Connected)
            {
                Publish(msg);
            }
        }

        private void UpdateMinMax(float value, ref float min, ref float max)
        {
            if (value < min)
            {
                min = value;
            }

            if (value > max)
            {
                max = value;
            }
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Assert(visualizer != null);

            if (msg == null)
            {
                return;
            }

            var graphData = new Dictionary<string, object>()
            {
                { "Speed", msg.Speed },
                { "Steering Front", msg.SteeringAngleFront },
                { "Steering Back", msg.SteeringAngleBack },
                { "Start Position", StartPosition },
                { "Distance", Distance },
                { "CurrentSpeedLimit", SpeedViolationLane?.speedLimit },
                { "SpeedViolationDuration", TimeSpan.FromSeconds(SpeedViolationCount).ToString() },
                { "SpeedViolationMax", SpeedViolationMax },
            };
            visualizer.UpdateGraphValues(graphData);
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }

        private void SpeedViolationEvent(uint id, MapTrafficLane laneData)
        {
            Hashtable data = new Hashtable
            {
                { "Id", id },
                { "Type", "SpeedViolation" },
                { "Time", SimulatorManager.Instance.GetSessionElapsedTimeSpan().ToString() },
                { "Location", transform.position },
                { "SpeedLimit", laneData.speedLimit },
                { "MaxSpeed", SpeedViolationMax },
                { "Duration", TimeSpan.FromSeconds(SpeedViolationCount).ToString() },
                { "Status", AnalysisManager.AnalysisStatusType.Failed },
            };
            SimulatorManager.Instance.AnalysisManager.AddEvent(data);
        }
    }
}
