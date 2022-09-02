using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Carousel
{

namespace MotionMatching{
    
public class Utils
{
    static List<HumanBodyBones> UPPER_BODY_BONES = new List<HumanBodyBones>(){HumanBodyBones.Spine,    
    HumanBodyBones.Chest, 
    HumanBodyBones.UpperChest, 
    HumanBodyBones.Neck,
    HumanBodyBones.Head,
    HumanBodyBones.LeftShoulder,
    HumanBodyBones.LeftUpperArm,
    HumanBodyBones.LeftLowerArm,
    HumanBodyBones.LeftHand,
    HumanBodyBones.RightShoulder,
    HumanBodyBones.RightUpperArm,
    HumanBodyBones.RightLowerArm,
    HumanBodyBones.RightHand
        };
    public static float fast_negexpf(float x)
    {
        return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
    }


    public static float halflife_to_damping(float halflife, float eps = 1e-5f)
    {

        float LN2f = 0.69314718056f;
        return (4.0f * LN2f) / (halflife + eps);

    }

    public static List<int> CreateUpperBodyIndexList(Dictionary<HumanBodyBones, int> boneIndexMap){
        var upperBodyIndices = new List<int>();
        for(int i =0; i < UPPER_BODY_BONES.Count; i++){
            var bone = UPPER_BODY_BONES[i];
            if (boneIndexMap.ContainsKey(bone)){
                var boneIdx = boneIndexMap[bone];
                upperBodyIndices.Add(boneIdx);
            }
    }
    return upperBodyIndices;
    }
}

class Quat
{
    public static Quaternion quat_exp(Vector3 v, float eps = 1e-8f)
    {
        float halfangle = Mathf.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

        if (halfangle < eps)
        {
            return new Quaternion(v.x, v.y, v.z, 1.0f).normalized;
        }
        else
        {
            float c = Mathf.Cos(halfangle);
            float s = Mathf.Sin(halfangle) / halfangle;
            return new Quaternion(s * v.x, s * v.y, s * v.z, c);
        }
    }

    public static Vector3 quat_log(Quaternion q, float eps = 1e-8f)
    {
        float length = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z);

        if (length < eps)
        {
            return new Vector3(q.x, q.y, q.z);
        }
        else
        {
            float halfangle = Mathf.Acos(Mathf.Clamp(q.w, -1.0f, 1.0f));
            return halfangle * (new Vector3(q.x, q.y, q.z) / length);
        }
    }

    public static Quaternion from_scaled_angle_axis(Vector3 v, float eps = 1e-8f)
    {
        return quat_exp(v / 2.0f, eps);
    }

    public static Vector3 to_scaled_angle_axis(Quaternion q, float eps = 1e-8f)
    {
        return 2.0f * quat_log(q, eps);
    }
}


}
}