struct InstanceData
{
    float3   position;
    float    scale;
    uint     colorType;
    uint3    pad;
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

    float3 worldPos = localPos * inst.scale + inst.position;

    VSOutput o;
    o.pos = mul(float4(worldPos, 1.0f), ViewProj);
    o.colorType = inst.colorType;
    return o;
}