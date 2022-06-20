using UnityEngine;

namespace CustomAnimation
{
	[System.Serializable]
	public class SkeletonDesc
	{
		public string root;
		public JointDesc[] jointDescs;
		public string[] jointSequence;
		public CPose referencePose;
	}


    [System.Serializable]
    public class CoordinateSystem
    {
        public Vector3 xAxis;
        public Vector3 yAxis;
    }

    [System.Serializable]
    public class PointCloudSkeletonDesc
    {
        public string[] jointSequence;
        public int[] childSequence; //stores child corresponding to joint in joint index
        public SkeletonDesc targetSkeleton;
        public string[] targetJointSequence;
        public CoordinateSystem[] targetSkeletonCosSequence;
    }
}
