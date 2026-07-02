struct VSOutput
{
    float4 pos : SV_Position;
};

float4 main(VSOutput input) : SV_Target
{
    return float4(0.2f, 0.5f, 1.0f, 1.0f);
}
