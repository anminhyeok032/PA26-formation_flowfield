#pragma once
#include <vector>
#include <string>

class HeightMap
{
public:
    // BMP 파일 로드 (24bit/8bit 지원)
    // R채널 픽셀값(0~255) → 높이값(0~maxHeight) 변환
    bool LoadFromBMP(const std::string& filePath, float maxHeight = 10.0f, float cellSize = 1.0f);

    // 높이 샘플링
    float GetHeight(int x, int z) const;

    // 메타데이터
    int   GetWidth()     const { return m_Width; }
    int   GetDepth()     const { return m_Depth; }
    float GetCellSize()  const { return m_CellSize; }
    float GetMaxHeight() const { return m_MaxHeight; }
    bool  IsLoaded()     const { return !m_Heights.empty(); }

private:
    std::vector<float> m_Heights; // 픽셀당 높이값 (월드 유닛)
    int   m_Width = 0;
    int   m_Depth = 0;
    float m_MaxHeight = 10.0f;
    float m_CellSize = 1.0f;

    int Index(int x, int z) const { return z * m_Width + x; }
};
