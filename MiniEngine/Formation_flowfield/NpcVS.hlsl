struct InstanceData
{
    float3 position; // 타원체 중심 월드 위치
    float  scaleXZ;  // XZ 반지름
    float  scaleY;   // Y 반지름
    uint   colorType;
    uint2  pad;
};

cbuffer ViewProjCB : register(b0)
{
    float4x4 ViewProj;
};

StructuredBuffer<InstanceData> g_Instances : register(t0);

struct VSOutput
{
    float4 pos       : SV_Position;
    uint   colorType : COLOR;
};

VSOutput main(float3 localPos : POSITION, uint instanceID : SV_InstanceID)
{
    InstanceData inst = g_Instances[instanceID];

    float3 worldPos;
    worldPos.x = localPos.x * inst.scaleXZ + inst.position[0];
    worldPos.y = localPos.y * inst.scaleY + inst.position[1];
    worldPos.z = localPos.z * inst.scaleXZ + inst.position[2];

    VSOutput o;
    o.pos = mul(float4(worldPos, 1.0f), ViewProj);
    o.colorType = inst.colorType;
    return o;
}
