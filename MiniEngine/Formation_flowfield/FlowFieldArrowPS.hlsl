struct PSInput
{
    float4 pos : SV_Position;
};

float4 main(PSInput input) : SV_Target
{
    return float4(1.0f, 1.0f, 0.0f, 1.0f);   // 청록색 - 지형/경로 색과 구분됨
}
