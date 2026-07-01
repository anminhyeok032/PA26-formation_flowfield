cbuffer MVPConstants : register(b0)
{
    float4x4 ModelViewProj;
};

float4 main(float3 pos : POSITION) : SV_Position
{
    return mul(ModelViewProj, float4(pos, 1.0f));
}