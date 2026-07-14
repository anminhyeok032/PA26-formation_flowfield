struct InstanceData
{
    float3 position;
    float  length;
    float3 direction;
    float  pad;
};

cbuffer ViewProjCB : register(b0)
{
    float4x4 ViewProj;
};

StructuredBuffer<InstanceData> g_Instances : register(t0);

struct VSOutput
{
    float4 pos : SV_Position;
};

VSOutput main(float3 localPos : POSITION, uint instanceID : SV_InstanceID)
{
    InstanceData inst = g_Instances[instanceID];

    float3 fwd = normalize(inst.direction);

    // 직교 기저 생성.
    // fwd가 월드 up과 거의 평행하면 cross가 0에 가까워지므로,
    // 그 경우 다른 축을 기준으로 삼아 예외를 피함.
    float3 worldUp = float3(0.0f, 1.0f, 0.0f);
    float3 right;
    if (abs(dot(fwd, worldUp)) > 0.99f)
        right = normalize(cross(float3(1.0f, 0.0f, 0.0f), fwd));
    else
        right = normalize(cross(worldUp, fwd));

    float3 up = cross(fwd, right);

    // 로컬 좌표(메시는 +Z를 향함)를 기저로 회전 + 길이 스케일
    float3 rotated = localPos.x * right
        + localPos.y * up
        + localPos.z * fwd;

    float3 worldPos = rotated * inst.length + inst.position;

    VSOutput o;
    o.pos = mul(float4(worldPos, 1.0f), ViewProj);
    return o;
}
