using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.PickAndPlace
{

    [RequireComponent(typeof(MeshRenderer))]
    [RequireComponent(typeof(BoxCollider))]
    public class TargetPlacement : MonoBehaviour
    {
        const string k_NameExpectedTarget = "Target";
        static readonly int k_ShaderColorId = Shader.PropertyToID("_Color");
        // The threshold that the Target's speed must be under to be considered "placed" in the target area
        const float k_MaximumSpeedForStopped = 0.01f;

        [SerializeField]
        [Tooltip("Target object expected by this placement area. Can be left blank if only one Target in scene")]
        GameObject m_Target;
        [SerializeField]
        [Range(0, 255)]
        [Tooltip("Alpha value for any color set during state changes.")]
        int m_ColorAlpha = 100;

        MeshRenderer m_TargetMeshRenderer;

        float m_ColorAlpha01 => m_ColorAlpha / 255f;
        MeshRenderer m_MeshRenderer;
        BoxCollider m_BoxCollider;
        PlacementState m_CurrentState;
        PlacementState m_LastColoredState;

        public PlacementState CurrentState
        {
            get => m_CurrentState;
            private set
            {
                m_CurrentState = value;
                UpdateStateColor();
            }
        }

        public enum PlacementState
        {
            Outside,
            InsideFloating,
            InsidePlaced
        }

        // Start is called before the first frame update
        void Start()
        {
            // Check for mis-configurations and disable if something has changed without this script being updated
            // These are warnings because this script does not contain critical functionality
            if (m_Target == null)
            {
                m_Target = GameObject.Find(k_NameExpectedTarget);
            }

            if (m_Target == null)
            {
                Debug.LogWarning($"{nameof(TargetPlacement)} expects to find a GameObject named " +
                    $"{k_NameExpectedTarget} to track, but did not. Can't track placement state.");
                enabled = false;
                return;
            }

            if (!TrySetComponentReferences())
            {
                enabled = false;
                return;
            }
            InitializeState();
        }

        bool TrySetComponentReferences()
        {
            m_TargetMeshRenderer = m_Target.GetComponent<MeshRenderer>();
            if (m_TargetMeshRenderer == null)
            {
                Debug.LogWarning($"{nameof(TargetPlacement)} expects a {nameof(MeshRenderer)} to be attached " +
                    $"to {k_NameExpectedTarget}. Cannot check bounds without it, so cannot track placement state.");
                return false;
            }

            // Assume these are here because they are RequiredComponent components
            m_MeshRenderer = GetComponent<MeshRenderer>();
            m_BoxCollider = GetComponent<BoxCollider>();
            return true;
        }

        void OnValidate()
        {
            // Useful for visualizing state in editor, but doesn't wholly guarantee accurate coloring in EditMode
            // Enter PlayMode to see color update correctly
            if (m_Target != null)
            {
                if (TrySetComponentReferences())
                {
                    InitializeState();
                }
            }
        }

        void InitializeState()
        {
            if (m_Target.GetComponent<BoxCollider>().bounds.Intersects(m_BoxCollider.bounds))
            {
                CurrentState = IsTargetStoppedInsideBounds() ?
                    PlacementState.InsidePlaced : PlacementState.InsideFloating;
            }
            else
            {
                CurrentState = PlacementState.Outside;
            }
        }

        void OnTriggerEnter(Collider other)
        {
            if (other.gameObject.name == m_Target.name)
            {
                CurrentState = PlacementState.InsideFloating;
            }
        }

        void OnTriggerExit(Collider other)
        {
            if (other.gameObject.name == m_Target.name)
            {
                CurrentState = PlacementState.Outside;
            }
        }

        bool IsTargetStoppedInsideBounds()
        {
            var targetIsStopped = m_Target.GetComponent<Rigidbody>().velocity.magnitude < k_MaximumSpeedForStopped;
            var targetIsInBounds = m_BoxCollider.bounds.Contains(m_TargetMeshRenderer.bounds.center);

            return targetIsStopped && targetIsInBounds;
        }

        // Update is called once per frame
        void Update()
        {
            if (CurrentState != PlacementState.Outside)
            {
                CurrentState = IsTargetStoppedInsideBounds() ?
                    PlacementState.InsidePlaced : PlacementState.InsideFloating;
            }
        }

        void UpdateStateColor()
        {
            if (m_CurrentState == m_LastColoredState)
            {
                return;
            }

            var mpb = new MaterialPropertyBlock();
            Color stateColor;
            switch (m_CurrentState)
            {
                case PlacementState.Outside:
                    stateColor = Color.red;
                    break;
                case PlacementState.InsideFloating:
                    stateColor = Color.yellow;
                    break;
                case PlacementState.InsidePlaced:
                    stateColor = Color.green;
                    break;
                default:
                    Debug.LogError($"No state handling implemented for {m_CurrentState}");
                    stateColor = Color.magenta;
                    break;
            }

            stateColor.a = m_ColorAlpha01;
            mpb.SetColor(k_ShaderColorId, stateColor);
            m_MeshRenderer.SetPropertyBlock(mpb);
            m_LastColoredState = m_CurrentState;
        }
    }
}
