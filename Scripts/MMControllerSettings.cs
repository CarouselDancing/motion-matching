using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Carousel
{

namespace MotionMatching{


[Serializable]
public class MMControllerSettigs
{

    public float visScale = 0.05f;
    public float forceSearchTime = 0.1f;
    public float simulationVelocityHalflife = 0.27f;
    public float simulationRotationHalflife = 0.27f;
    public int predictionDistance = 20;
    public float maxSpeed = 2.5f;
    public float minSpeed = 0.1f;
    public int startFrameIdx = 0;
    public int speedFactor = 2;
    public bool useInterpolation = true;
    public float maxDegreesPerSecond = 90f;
    public List<int> startConstraint = new List<int>() {3, 4};
    public bool useStartConstraint;
    public bool adaptControlWeight;
}

}
}