#pragma once
#include <vector>
#include <string>

class HeightMap
{
public:
    // BMP 파일 로드 (24bit/8bit 지원)
    // R채널 픽셀값(0~255) → 높이값(0~maxHeight) 변환
    bool LoadFromBMP(const std::string& filePath, float maxHeight = 10.0f, float worldScale = 1.0f, float voxelSize = 1.0f);


    // 월드 좌표로 높이 샘플링 (바이리니어 보간)
    // VoxelGrid가 복셀 중심 월드 좌표로 높이를 물어볼 때 사용
    float SampleHeight(float worldX, float worldZ) const;

    // 높이 샘플링
    float GetHeight(int x, int z) const;

    // 맵 전체 크기 (월드 유닛)
    float GetWorldWidth() const { return m_Width * m_WorldScale; }
    float GetWorldDepth() const { return m_Depth * m_WorldScale; }

    float GetWorldScale() const { return m_WorldScale; }
    float GetVoxelSize()  const { return m_VoxelSize; }
    float GetMaxHeight()  const { return m_MaxHeight; }
    int   GetWidth()      const { return m_Width; }
    int   GetDepth()      const { return m_Depth; }
    bool  IsLoaded()      const { return !m_Heights.empty(); }

private:
    std::vector<float> m_Heights;   // 픽셀당 높이값 (월드 유닛)
    int   m_Width = 0;
    int   m_Depth = 0;
    float m_MaxHeight = 10.0f;
    float m_WorldScale = 1.0f;      // 픽셀당 월드 크기 (맵 크기 결정)
    float m_VoxelSize = 1.0f;       // 복셀 크기 (세밀도 결정)

    int Index(int x, int z) const { return z * m_Width + x; }
};
