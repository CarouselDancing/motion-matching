using System;
using System.Collections.Generic;
using UnityEngine;


namespace Carousel
{

namespace MotionMatching{

public class MMRuntimeRetargetingV2 : MonoBehaviour
{
    [Serializable]
    public class RetargetingMap
    {
        public HumanBodyBones bone;
        public Vector3 srcUp;
        public Vector3 srcSide;
        public Vector3 dstUp;
        public Vector3 dstSide;
        public Transform dstT;
        public bool active = true;

    }
    public MMPoseProvider src;
    public List<RetargetingMap> retargetingMap;
    public float visLength = 10;
    public string rootName;
    public Vector3 rootOffset;
    public bool bringToLocal = true;
    public bool mirror = false;
    public bool showSrcGizmos = false;
    public bool showDstGizmos = false;
    RetargetingMap rootMap;
    bool initialized = false;
    Animator anim;
    public bool automatic = false;
    // Start is called before the first frame update
    void Start()
    {
        anim = GetComponent<Animator>();
        SetTransforms();
    }
    public void ResetToIdle()
    {
        src.ResetToIdle();
    }

    public void SetTransforms()
    {
        

        for (int i =0;i <retargetingMap.Count; i++)
        {
            
            if (retargetingMap[i].bone == HumanBodyBones.LastBone)
            {

                retargetingMap[i].dstT = transform;
                rootMap = retargetingMap[i];
            }
            else
            {
                retargetingMap[i].dstT = anim.GetBoneTransform(retargetingMap[i].bone);
            }
            
        }
        initialized = true;
    }


    // Update is called once per frame
    void FixedUpdate()
    {

        //transform.position = src.GetGlobalPosition(Bones.Entity);
        if (initialized && automatic) OrientToTarget();
    }

    public void TriggerUpdate()
    {
        if (initialized)
        {
            OrientToTarget();
        }
    }

    public Quaternion RetargetRotation(Quaternion srcRotation, RetargetingMap m)
    {
     
        var globalSrcUp = srcRotation * m.srcUp;
        globalSrcUp = globalSrcUp.normalized;
        var globalSrcSide = srcRotation * m.srcSide;
        globalSrcSide = globalSrcSide.normalized;
        var rotation1 = Quaternion.FromToRotation(m.dstUp, globalSrcUp).normalized;
        var dstSide = rotation1 * m.dstSide;
        var rotation2 = Quaternion.FromToRotation(dstSide.normalized, globalSrcSide).normalized;
        var rotation = Quaternion.Normalize(rotation2 * rotation1);
        var dstUp = rotation * m.dstUp;
        var rotation3 = Quaternion.FromToRotation(dstUp.normalized, globalSrcUp).normalized;
        rotation = Quaternion.Normalize(rotation3 * rotation);
        return rotation;
    }

    public void OrientToTarget()
    {
        Quaternion srcRotation;
        Vector3 srcPosition;
        foreach (var m in retargetingMap)
        {
            if (m.bone == HumanBodyBones.LastBone) continue;
            if (! m.active) continue;

            var dstT = m.dstT;
            if (!src.GetGlobalRotation(m.bone, out srcRotation))
            {
                Debug.Log(m.bone.ToString() + "does not exist");
                continue;
            }
            var rotation = RetargetRotation(srcRotation, m);
            if (m.bone == HumanBodyBones.Hips)
            {
                dstT.rotation = rotation.normalized;
                src.GetGlobalPosition(m.bone, out srcPosition);
                dstT.position = srcPosition;
            }
            else
            {
                var invRot = Quaternion.identity;
                if (dstT.parent != null)
                {
                    invRot = Quaternion.Inverse(dstT.parent.rotation);
                }
                dstT.localRotation = invRot * rotation.normalized;
            }

            
        }
        
    }


    void OnDrawGizmos()
    {
        if (src.mm == null) return;
        if (!src.mm.initialized) return;
        if (!showSrcGizmos && !showDstGizmos) return;
        foreach (var m in retargetingMap)
        {
            if (m.dstT != null)
            {
                if (showSrcGizmos) { 
                    Quaternion srcRotation;
                    if (src.GetGlobalRotation(m.bone, out srcRotation))
                    {
                        var srcPosition = src.GetGlobalPosition(m.bone);
                        var globalSrcUp = srcRotation * m.srcUp * visLength;
                        var globalSrcSide = srcRotation * m.srcSide * visLength;
                        Debug.DrawLine(srcPosition, srcPosition + globalSrcUp, Color.green);
                        Debug.DrawLine(srcPosition, srcPosition + globalSrcSide, Color.red);
                    }
                    
                }
                if (showDstGizmos) { 
                    var globalDstUp = m.dstT.rotation * m.dstUp * visLength;
                    var globalDstSide = m.dstT.rotation * m.dstSide * visLength;
                    Debug.DrawLine(m.dstT.position, m.dstT.position + globalDstUp, Color.green);
                    Debug.DrawLine(m.dstT.position, m.dstT.position + globalDstSide, Color.red);
                }
            }
        }
    }
}

}
}