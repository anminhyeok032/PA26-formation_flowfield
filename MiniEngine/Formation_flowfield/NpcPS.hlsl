struct VSOutput
{
    float4 pos : SV_Position;
};

cbuffer WireConstants : register(b1)
{
    uint IsWireframePass;
};


float4 main(float4 pos : SV_Position, uint colorType : COLOR) : SV_Target
{
    if (IsWireframePass)
        return float4(0, 0, 0, 1);

    if (colorType == 2)
        return float4(0.3, 1.0, 0.3, 1.0);

    return float4(0.2, 0.5, 1.0, 1.0);
}
