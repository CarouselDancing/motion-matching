using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomAnimation
{

        [System.Serializable]
    public class StartPose
    {
        public Vector3 position;
        public Quaternion orientation;
        public bool forceWalkEndConstraints;
    }

	[System.Serializable]
	public class CKeyframe
	{
		public Vector3 rootTranslation;
		public Vector4[] rotations;
        public string action;
        public bool isIdle;
        public string annotation;
    }

    [System.Serializable]
    public class PointCloudKeyframe
    {
        public Vector3[] positions;
        public Vector3[] xAxisPositions;
        public string action;
        public bool isIdle;
        public string annotation;
    }

    [System.Serializable]
    public class GlobalPose{
        
		public Vector4[] rotations;      
        public Vector3[] positions;
    }

    
}
